// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_pxc_win.h"

#include "base/bind.h"
#include "base/command_line.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"
#include "media/base/media_switches.h"

namespace media {

// This needs to be aligned with definition in
// content/renderer/media/media_stream_impl.cc
const char kVirtualDepthDeviceId[] = "depth";

static const size_t kBytesPerPixelRGB32 = 4;

// TODO(nhu): the min and max depth value should be obtained from PCSDK API.
static const int kMinDepthValue = 150;
static const int kMaxDepthValue = 3000;

// Depth encoding in RGB32
const char kDepthEncodingGrayscale[] = "grayscale";
const char kDepthEncodingRaw[] = "raw";
const char kDepthEncodingAdaptive[] = "adaptive";

// Release a PXCSession will call CoUninitiliaze which causes calling thread
// (Audio Thread) to fail on COM API calling. Hold a global PXCSession to avoid
// this.
static PXCSmartPtr<PXCSession> g_session;

// Map the virtual depth camera ID to real one.
std::string g_real_depth_device_id;

bool VideoCaptureDevicePxcWin::PlatformSupported() {
  if (g_session.IsValid())
    return true;

  pxcStatus status = PXCSession_Create(&g_session);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to create a PXC Session.";
    return false;
  }
  return true;
}

void VideoCaptureDevicePxcWin::GetDeviceNames(Names* device_names) {
  device_names->clear();

  if (!VideoCaptureDevicePxcWin::PlatformSupported())
    return;

  pxcStatus status;
  PXCSession::ImplDesc video_capture_desc;
  memset(&video_capture_desc, 0, sizeof(video_capture_desc));
  video_capture_desc.group = PXCSession::IMPL_GROUP_SENSOR;
  video_capture_desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
  for (int module_index = 0; ; module_index++) {
    PXCSession::ImplDesc desc;
    status = g_session->QueryImpl(&video_capture_desc, module_index, &desc);
    if (status < PXC_STATUS_NO_ERROR)
      break;

    PXCSmartPtr<PXCCapture> capture;
    status = g_session->CreateImpl<PXCCapture>(&desc, &capture);
    if (status < PXC_STATUS_NO_ERROR)
      continue;

    for (int device_index = 0; ; device_index++) {
      PXCCapture::DeviceInfo device_info;
      status = capture->QueryDevice(device_index, &device_info);
      if (status < PXC_STATUS_NO_ERROR)
        break;

      PXCSmartPtr<PXCCapture::Device> capture_device;
      status = capture->CreateDevice(device_index, &capture_device);
      if (status < PXC_STATUS_NO_ERROR)
        continue;

      bool found_color_stream = false;
      bool found_depth_stream = false;
      for (int stream_index = 0; ; stream_index++) {
        PXCCapture::Device::StreamInfo stream_info;
        status = capture_device->QueryStream(stream_index, &stream_info);
        if (status < PXC_STATUS_NO_ERROR)
          break;

        if (stream_info.cuid == PXCCapture::VideoStream::CUID)
          if (stream_info.imageType == PXCImage::IMAGE_TYPE_COLOR) {
            found_color_stream = true;
          } else if (stream_info.imageType == PXCImage::IMAGE_TYPE_DEPTH) {
            found_depth_stream = true;
          }
      }

      if (found_color_stream) {
        Name name(base::WideToUTF8(std::wstring(device_info.name)),
                  base::WideToUTF8(std::wstring(device_info.did)),
                  Name::PXC_CAPTURE);
        DLOG(INFO) << "Color video capture device, " << name.name()
                   << " : " << name.id();
        device_names->push_back(name);
      }
      if (found_depth_stream) {
        Name name(base::WideToUTF8(std::wstring(device_info.name)),
                  kVirtualDepthDeviceId,
                  Name::PXC_CAPTURE);
        DLOG(INFO) << "Depth video capture device, " << name.name()
                   << " : " << name.id();
        g_real_depth_device_id = base::WideToUTF8(std::wstring(device_info.did));
        device_names->push_back(name);
      }
    }  // Enumerate devices.
  }  // Enumerate modules.
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : is_capturing_depth_(false),
      device_name_(device_name),
      state_(kIdle),
      pxc_capture_thread_("PxcCaptureThread") {
  DLOG(INFO) << device_name.name() << ": " << device_name.id();
  if (device_name.id() == kVirtualDepthDeviceId) {
    is_capturing_depth_ = true;
    const CommandLine* cmd_line = CommandLine::ForCurrentProcess();
    std::string encoding_option(
        cmd_line->GetSwitchValueASCII(switches::kDepthEncoding));
    if (encoding_option == kDepthEncodingGrayscale) {
      depth_encoding_ = kGrayscaleRGB32;
    } else if (encoding_option == kDepthEncodingRaw) {
      depth_encoding_ = kRawRGB32;
    } else if (encoding_option == kDepthEncodingAdaptive) {
      depth_encoding_ = kAdaptiveRGB32;
    } else {
      depth_encoding_ = kGrayscaleRGB32;
    }
  }
}

VideoCaptureDevicePxcWin::~VideoCaptureDevicePxcWin() {
}

bool VideoCaptureDevicePxcWin::Init() {
  pxcStatus status;
  PXCSession::ImplDesc video_capture_desc;
  memset(&video_capture_desc, 0, sizeof(video_capture_desc));
  video_capture_desc.group = PXCSession::IMPL_GROUP_SENSOR;
  video_capture_desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
  for (int module_index = 0; ; module_index++) {
    PXCSession::ImplDesc desc;
    status = g_session->QueryImpl(&video_capture_desc, module_index, &desc);
    if (status < PXC_STATUS_NO_ERROR) {
      // No more modules.
      break;
    }

    PXCSmartPtr<PXCCapture> capture;
    status = g_session->CreateImpl<PXCCapture>(&desc, &capture);
    if (status < PXC_STATUS_NO_ERROR) {
      continue;
    }

    for (int device_index = 0; ; device_index++) {
      PXCCapture::DeviceInfo device_info;
      status = capture->QueryDevice(device_index, &device_info);
      if (status < PXC_STATUS_NO_ERROR) {
        // No more devices.
        break;
      }

      std::string device_name = device_name_.name();
      std::string device_id = device_name_.id();
      if (is_capturing_depth_) {
        device_id = g_real_depth_device_id;
      }
      if (base::WideToUTF8(std::wstring(device_info.name)) != device_name ||
          base::WideToUTF8(std::wstring(device_info.did)) != device_id)
        continue;

      PXCSmartPtr<PXCCapture::Device> device;
      status = capture->CreateDevice(device_index, &device);
      if (status < PXC_STATUS_NO_ERROR)
        break;

      DLOG(INFO) << "Device is created: "
                 << device_info.name << device_info.did;
      device_ = device.ReleasePtr();
      return true;
    }  // Enumerate capture devices.
  }  // Enumerate the video capture modules.
  return false;
}

void VideoCaptureDevicePxcWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  if (pxc_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  pxc_capture_thread_.Start();
  pxc_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDevicePxcWin::OnAllocateAndStart,
                 base::Unretained(this),
                 params.requested_format.frame_size.width(),
                 params.requested_format.frame_size.height(),
                 params.requested_format.frame_rate,
                 base::Passed(&client)));
}

void VideoCaptureDevicePxcWin::StopAndDeAllocate() {
  if (!pxc_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  pxc_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDevicePxcWin::OnStopAndDeAllocate,
                 base::Unretained(this)));
  pxc_capture_thread_.Stop();
}

void VideoCaptureDevicePxcWin::OnAllocateAndStart(
    int width, int height, int frame_rate, scoped_ptr<Client> client) {
  DCHECK_EQ(pxc_capture_thread_.message_loop(), base::MessageLoop::current());

  client_ = client.Pass();

  if (!device_.IsValid()) {
    SetErrorState("No capture device is initialized.");
  }

  pxcStatus status;
  for (int stream_index = 0; ; stream_index++) {
    PXCCapture::Device::StreamInfo stream_info;
    status = device_->QueryStream(stream_index, &stream_info);
    if (status < PXC_STATUS_NO_ERROR) {
      // No more streams.
      break;
    }

    if (stream_info.cuid != PXCCapture::VideoStream::CUID)
      continue;

    if (!is_capturing_depth_ &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_COLOR)
      continue;

    if (is_capturing_depth_ &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_DEPTH)
      continue;

    PXCSmartPtr<PXCCapture::VideoStream> stream;
    status =
        device_->CreateStream<PXCCapture::VideoStream>(stream_index,
                                                              &stream);
    if (status < PXC_STATUS_NO_ERROR)
      continue;

    // Find the best profile.
    PXCCapture::VideoStream::ProfileInfo best_profile;
    bool best_profile_found = false;
    uint32 best = 0xFFFFFFFF;
    for (int porfile_index = 0; !stream_.IsValid(); porfile_index++) {
      PXCCapture::VideoStream::ProfileInfo video_profile;
      status = stream->QueryProfile(porfile_index, &video_profile);
      if (status < PXC_STATUS_NO_ERROR) {
        // No more profiles.
        break;
      }

      uint32 frameRateMin =
          video_profile.frameRateMin.denominator ?
              video_profile.frameRateMin.numerator /
                  video_profile.frameRateMin.denominator
                      : 0;
      uint32 frameRateMax =
          video_profile.frameRateMax.denominator ?
              video_profile.frameRateMax.numerator /
                  video_profile.frameRateMax.denominator
                      : 0;
      uint32 current =
          abs(static_cast<int>(width - video_profile.imageInfo.width)) +
          abs(static_cast<int>(height - video_profile.imageInfo.height)) +
          abs(static_cast<int>(frame_rate - frameRateMin)) +
          abs(static_cast<int>(frame_rate - frameRateMax));

      if (current < best) {
        best = current;
        best_profile_found = true;
        best_profile = video_profile;
      }
    }  // Enumerate profiles.

    if (!best_profile_found)
      continue;

    status = stream->SetProfile(&best_profile);
    if (status < PXC_STATUS_NO_ERROR)
      break;
    stream_ = stream.ReleasePtr();

    // TODO(nhu): fix the potential color conversions caused by hardcoding
    //            PIXEL_FORMAT_ARGB.
    DLOG(INFO) << "VideoCaptureFormat: "
               << " width = " << best_profile.imageInfo.width
               << " height = " << best_profile.imageInfo.height
               << " frame_rate = " << frame_rate
               << " color = " << PIXEL_FORMAT_ARGB;

      // Store our current width and height.
    capture_format_.frame_size.SetSize(best_profile.imageInfo.width,
                                       best_profile.imageInfo.height);
    capture_format_.frame_rate = frame_rate;
    capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

    if (is_capturing_depth_) {
      GetDepthDeviceProperties();
      depth_rgb32_image_.reset(
          new uint8[capture_format_.frame_size.width() *
                    capture_format_.frame_size.height() *
                    kBytesPerPixelRGB32]);
    }

    // Start capturing.
    state_ = kCapturing;
    pxc_capture_thread_.message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&VideoCaptureDevicePxcWin::OnCaptureTask,
                   base::Unretained(this)));
    return;
  }  // Enumrate streams.
  SetErrorState("Cannot find appropriate stream.");
}

void VideoCaptureDevicePxcWin::OnStopAndDeAllocate() {
  DCHECK_EQ(pxc_capture_thread_.message_loop(), base::MessageLoop::current());

  state_ = kIdle;
  stream_.ReleaseRef();
  device_.ReleaseRef();
  client_.reset();
  depth_rgb32_image_.reset();
}

void VideoCaptureDevicePxcWin::OnCaptureTask() {
  DCHECK_EQ(pxc_capture_thread_.message_loop(), base::MessageLoop::current());

  if (state_ != kCapturing || !stream_.IsValid())
    return;

  PXCSmartSP sp;
  PXCSmartPtr<PXCImage> image;
  pxcStatus status = stream_->ReadStreamAsync(&image, &sp);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Read stream error");
    return;
  }

  status = sp->Synchronize();
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Read synchronization EOF");
    return;
  }

  PXCImage::ImageInfo info;
  status = image->QueryInfo(&info);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Get image info error");
    return;
  }

  PXCImage::ImageData data;
  PXCImage::ColorFormat format = PXCImage::COLOR_FORMAT_RGB32;
  if (is_capturing_depth_)
    format = PXCImage::COLOR_FORMAT_DEPTH;
  status = image->AcquireAccess(
      PXCImage::ACCESS_READ, format, &data);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Access image data error");
    return;
  }

  DCHECK_EQ(data.type, PXCImage::SURFACE_TYPE_SYSTEM_MEMORY);

  if (!is_capturing_depth_) {
    CaptureColorImage(info, data);
  } else {
    CaptureDepthImage(info, data);
  }

  image->ReleaseAccess(&data);

  pxc_capture_thread_.message_loop()->PostTask(
    FROM_HERE,
    base::Bind(&VideoCaptureDevicePxcWin::OnCaptureTask,
               base::Unretained(this)));
}

void VideoCaptureDevicePxcWin::GetDepthDeviceProperties() {
  device_->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE,
      &depth_saturation_value_);
  device_->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE,
      &depth_low_confidence_value_);
  device_->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_UNIT,
      &depth_unit_in_micrometers_);
  device_->QueryPropertyAsRange(
      PXCCapture::Device::PROPERTY_DEPTH_SENSOR_RANGE,
      &depth_range_in_millimeters_);
  DLOG(INFO) << "Depth Device Properties: "
             << "\nPROPERTY_DEPTH_SATURATION_VALUE: "
             << depth_saturation_value_
             << "\nPROPERTY_DEPTH_LOW_CONFIDENCE_VALUE: "
             << depth_low_confidence_value_
             << "\nPROPERTY_DEPTH_UNIT: "
             << depth_unit_in_micrometers_
             << "\nPROPERTY_DEPTH_SENSOR_RANGE: "
             << depth_range_in_millimeters_.min << ":"
             << depth_range_in_millimeters_.max;
}


void VideoCaptureDevicePxcWin::SetErrorState(const std::string& reason) {
  DVLOG(1) << reason;
  state_ = kError;
  client_->OnError(reason);
}

void VideoCaptureDevicePxcWin::CaptureColorImage(
    const PXCImage::ImageInfo& info, const PXCImage::ImageData& data) {
  int length = info.width * info.height * kBytesPerPixelRGB32;
  client_->OnIncomingCapturedFrame(
      static_cast<uint8*> (data.planes[0]),
      length, base::TimeTicks::Now(), 0, capture_format_);
}

void VideoCaptureDevicePxcWin::CaptureDepthImage(
    const PXCImage::ImageInfo& info, const PXCImage::ImageData& data) {
  unsigned int length = info.width * info.height;
  uint8* rgb_data = depth_rgb32_image_.get();
  memset(rgb_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
  int16* depth_data = reinterpret_cast<int16*>(data.planes[0]);

  if (depth_encoding_ == kGrayscaleRGB32) {
    DepthToGrayscaleRGB32(depth_data, rgb_data, length);
  } else if (depth_encoding_ == kRawRGB32) {
    DepthToRawRGB32(depth_data, rgb_data, length);
  } else if (depth_encoding_ == kAdaptiveRGB32) {
    DepthToAdaptiveRGB32(depth_data, rgb_data, length);
  }

  client_->OnIncomingCapturedFrame(
      rgb_data,
      length, base::TimeTicks::Now(), 0, capture_format_);
}

void VideoCaptureDevicePxcWin::DepthToGrayscaleRGB32(
    int16* depth, uint8* rgb, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    int16 raw_depth_value = depth[i];
    if (raw_depth_value == depth_saturation_value_ ||
        raw_depth_value == depth_low_confidence_value_) {
      continue;
    }
    float depth_value_in_millimeters =
        raw_depth_value * depth_unit_in_micrometers_ / 1000;
    if (depth_value_in_millimeters > kMaxDepthValue) {
      continue;
    }

    // Implement the algorithm as equation (4) in paper
    // "3-D Video Representation Using Depth Maps".
    float value = 255.0 *
        ((1.0 / depth_value_in_millimeters -
          1.0 / kMaxDepthValue) /
         (1.0 / kMinDepthValue -
          1.0 / kMaxDepthValue));

    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 1] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 2] = static_cast<uint8>(value);
  }
}

void VideoCaptureDevicePxcWin::DepthToRawRGB32(
    int16* depth, uint8* rgb, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    int16 d = depth[i];
    if (d == depth_saturation_value_ ||
        d == depth_low_confidence_value_) {
      continue;
    }

    // Implement the BIT2 scheme in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] =
        static_cast<uint8>(((d & 0xFC00) >> 8) & 0xFF);
    rgb[i * kBytesPerPixelRGB32 + 1] =
        static_cast<uint8>(((d & 0x03E0) >> 2) & 0xFF);
    rgb[i * kBytesPerPixelRGB32 + 2] =
        static_cast<uint8>(((d & 0x001F) << 3) & 0xFF);
  }
}

void VideoCaptureDevicePxcWin::DepthToAdaptiveRGB32(
    int16* depth, uint8* rgb, unsigned int length) {
  const double np = 512.0;
  const double w = kMaxDepthValue;
  double p = np / w;
  for (unsigned int i = 0; i < length; i++) {
    int16 d = depth[i];
    if (d == depth_saturation_value_ ||
        d == depth_low_confidence_value_) {
      continue;
    }

    if (d > w) {
      continue;
    }

    // Implement the depth ecoding schema described in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
    double ld = (d + 0.5) / w;

    double ha = fmod(ld / (p / 2.0), 2.0);
    if (ha > 1.0)
      ha = 2.0 - ha;

    double hb = fmod((ld - p / 4.0) / (p / 2.0), 2.0);
    if (hb > 1.0)
      hb = 2.0 - hb;

    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] = static_cast<uint8>(255.0 * ld);
    rgb[i * kBytesPerPixelRGB32 + 1] = static_cast<uint8>(255.0 * ha);
    rgb[i * kBytesPerPixelRGB32 + 2] = static_cast<uint8>(255.0 * hb);
  }
}

}  // namespace media

