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
const char kVirtualRgbdDeviceId[] = "rgbd";

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
        Name name(WideToUTF8(std::wstring(device_info.name)),
                  WideToUTF8(std::wstring(device_info.did)),
                  Name::PXC_CAPTURE);
        DLOG(INFO) << "Color video capture device, " << name.name()
                   << " : " << name.id();
        device_names->push_back(name);
      }
      if (found_depth_stream) {
        Name name(WideToUTF8(std::wstring(device_info.name)),
                  kVirtualDepthDeviceId,
                  Name::PXC_CAPTURE);
        DLOG(INFO) << "Depth video capture device, " << name.name()
                   << " : " << name.id();
        g_real_depth_device_id = WideToUTF8(std::wstring(device_info.did));
        device_names->push_back(name);
      }
      if (found_color_stream && found_depth_stream) {
        Name name(WideToUTF8(std::wstring(device_info.name)),
          kVirtualRgbdDeviceId,
          Name::PXC_CAPTURE);
        DLOG(INFO) << "RGBD video capture device, " << name.name()
                   << " : " << name.id();
        g_real_depth_device_id = WideToUTF8(std::wstring(device_info.did));
        device_names->push_back(name);
      }
    }  // Enumerate devices.
  }  // Enumerate modules.
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : device_name_(device_name),
      state_(kIdle),
      pxc_capture_thread_("PxcCaptureThread") {
  DLOG(INFO) << device_name.name() << ": " << device_name.id();
  if (device_name.id() == kVirtualDepthDeviceId) {
    capture_mode_ = kCaptureDepth;
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
  } else if (device_name.id() == kVirtualRgbdDeviceId) {
    capture_mode_ = kCaptureRGBD;
  } else {
    capture_mode_ = kCaptureRGBA;
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
      if (capture_mode_ == kCaptureDepth || capture_mode_ == kCaptureRGBD) {
        device_id = g_real_depth_device_id;
      }
      if (WideToUTF8(std::wstring(device_info.name)) != device_name ||
          WideToUTF8(std::wstring(device_info.did)) != device_id)
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
    const VideoCaptureCapability& capture_format,
    scoped_ptr<Client> client) {
  if (pxc_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  pxc_capture_thread_.Start();
  pxc_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDevicePxcWin::OnAllocateAndStart,
                 base::Unretained(this),
                 capture_format.width,
                 capture_format.height,
                 capture_format.frame_rate,
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
  PXCCapture::VideoStream::ProfileInfo rgbd_profile;
  for (int stream_index = 0; ; stream_index++) {
    PXCCapture::Device::StreamInfo stream_info;
    status = device_->QueryStream(stream_index, &stream_info);
    if (status < PXC_STATUS_NO_ERROR) {
      // No more streams.
      break;
    }

    if (stream_info.cuid != PXCCapture::VideoStream::CUID)
      continue;

    if (capture_mode_ == kCaptureRGBA &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_COLOR)
      continue;

    if (capture_mode_ == kCaptureDepth &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_DEPTH)
      continue;

    if (capture_mode_ == kCaptureRGBD &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_DEPTH &&
        stream_info.imageType != PXCImage::IMAGE_TYPE_COLOR)
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
    for (int porfile_index = 0; ; porfile_index++) {
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

    if (capture_mode_ == kCaptureRGBD &&
        stream_info.imageType == PXCImage::IMAGE_TYPE_DEPTH)
      rgbd_profile = best_profile;

    if (stream_info.imageType == PXCImage::IMAGE_TYPE_COLOR)
      color_stream_ = stream.ReleasePtr();
    else if (stream_info.imageType == PXCImage::IMAGE_TYPE_DEPTH)
      depth_stream_ = stream.ReleasePtr();

    if (capture_mode_ == kCaptureRGBD &&
        (!color_stream_.IsValid() || !depth_stream_.IsValid()))
      continue;

    if (capture_mode_ == kCaptureRGBD)
      best_profile = rgbd_profile;

    // TODO(nhu): fix the potential color conversions caused by hardcoding
    //            PIXEL_FORMAT_ARGB.
    VideoCaptureCapability current_settings(
        best_profile.imageInfo.width,
        best_profile.imageInfo.height,
        frame_rate,
        PIXEL_FORMAT_ARGB,
        VariableResolutionVideoCaptureDevice);

    DLOG(INFO) << "VideoCaptureCapability: "
               << " width = " << current_settings.width
               << " height = " << current_settings.height
               << " frame_rate = " << current_settings.frame_rate
               << " color = " << current_settings.color;

    client_->OnFrameInfo(current_settings);

    if (capture_mode_ == kCaptureDepth || capture_mode_ == kCaptureRGBD) {
      GetDepthDeviceProperties();
      rgb32_image_.reset(
          new uint8[current_settings.width * current_settings.height *
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
  color_stream_.ReleaseRef();
  depth_stream_.ReleaseRef();
  device_.ReleaseRef();
  client_.reset();
  rgb32_image_.reset();
}

void VideoCaptureDevicePxcWin::OnCaptureTask() {
  DCHECK_EQ(pxc_capture_thread_.message_loop(), base::MessageLoop::current());

  if (state_ != kCapturing)
    return;

  PXCSmartSP color_sp, depth_sp;
  PXCSmartPtr<PXCImage> color_image, depth_image;
  pxcStatus status;

  if (capture_mode_ == kCaptureRGBA || capture_mode_ == kCaptureRGBD) {
    status = color_stream_->ReadStreamAsync(&color_image, &color_sp);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Read color stream error");
      return;
    }
  }

  if (capture_mode_ == kCaptureDepth || capture_mode_ == kCaptureRGBD) {
    status = depth_stream_->ReadStreamAsync(&depth_image, &depth_sp);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Read depth stream error");
      return;
    }
  }

  if (capture_mode_ == kCaptureRGBA || capture_mode_ == kCaptureRGBD) {
    status = color_sp->Synchronize();
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Color read synchronization EOF");
      return;
    }
  }

  if (capture_mode_ == kCaptureDepth || capture_mode_ == kCaptureRGBD) {
    status = depth_sp->Synchronize();
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Depth read synchronization EOF");
      return;
    }
  }

  PXCImage::ImageInfo color_info, depth_info;
  PXCImage::ImageData color_data, depth_data;
  if (capture_mode_ == kCaptureRGBA || capture_mode_ == kCaptureRGBD) {
    status = color_image->QueryInfo(&color_info);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Get color image info error");
      return;
    }
    status = color_image->AcquireAccess(
    PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &color_data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access color image data error");
      return;
    }
  }
  
  if (capture_mode_ == kCaptureDepth || capture_mode_ == kCaptureRGBD) {
    status = depth_image->QueryInfo(&depth_info);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Get depth image info error");
      return;
    }
    status = depth_image->AcquireAccess(
      PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_DEPTH, &depth_data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access depth image data error");
      return;
    }
  }

  if (capture_mode_ == kCaptureRGBA) {
    CaptureColorImage(color_info, color_data);
  } else if (capture_mode_ == kCaptureDepth) {
    CaptureDepthImage(depth_info, depth_data);
  } else if (capture_mode_ == kCaptureRGBD) {
    CaptureRgbdImage(color_info, color_data, depth_info, depth_data);
  }

  if (color_image.IsValid())
    color_image->ReleaseAccess(&color_data);

  if (depth_image.IsValid())
    depth_image->ReleaseAccess(&depth_data);

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
  client_->OnError();
}

void VideoCaptureDevicePxcWin::CaptureColorImage(
    const PXCImage::ImageInfo& info, const PXCImage::ImageData& data) {
  int length = info.width * info.height * kBytesPerPixelRGB32;
  client_->OnIncomingCapturedFrame(
      static_cast<uint8*> (data.planes[0]),
      length, base::Time::Now(), 0, false, false);
}

void VideoCaptureDevicePxcWin::CaptureDepthImage(
    const PXCImage::ImageInfo& info, const PXCImage::ImageData& data) {
  unsigned int length = info.width * info.height;
  uint8* rgb_data = rgb32_image_.get();
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
      length, base::Time::Now(), 0, false, false);
}

void VideoCaptureDevicePxcWin::CaptureRgbdImage(
    const PXCImage::ImageInfo& rgb_info, const PXCImage::ImageData& rgb_data,
    const PXCImage::ImageInfo& d_info, const PXCImage::ImageData& d_data) {
  unsigned int length = d_info.width * d_info.height;
  uint8* rgbd_data = rgb32_image_.get();
  memset(rgbd_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
  uint8* color_data = static_cast<uint8*>(rgb_data.planes[0]);
  int16* depth_data = reinterpret_cast<int16*>(d_data.planes[0]);
  float* uv_data = reinterpret_cast<float*>(d_data.planes[2]);
  for (unsigned int i = 0; i < length; i++) {
    int rgbd_index = i * kBytesPerPixelRGB32;
    float uv_x = uv_data[i * 2 + 0];
    float uv_y = uv_data[i * 2 + 1];
		int color_x = (int)(uv_x * rgb_info.width + 0.5f);
    int color_y = (int)(uv_y * rgb_info.height + 0.5f);
    if (color_x < 0 || color_x > static_cast<int>(rgb_info.width) ||
        color_y < 0 || color_y > static_cast<int>(rgb_info.height))
      continue;
    int color_index =
        (color_x + color_y * rgb_info.width) * kBytesPerPixelRGB32;
    rgbd_data[rgbd_index + 0] = color_data[color_index + 0];
    rgbd_data[rgbd_index + 1] = color_data[color_index + 1];
    rgbd_data[rgbd_index + 2] = color_data[color_index + 2];
  }
  client_->OnIncomingCapturedFrame(
    rgbd_data,
    length, base::Time::Now(), 0, false, false);
}

void VideoCaptureDevicePxcWin::DepthToGrayscaleRGB32(
    int16* depth, uint8* rgb, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    int raw_depth_value = depth[i];
    if (raw_depth_value == depth_saturation_value_ ||
        raw_depth_value == depth_low_confidence_value_) {
      continue;
    }
    float depth_value_in_millimeters =
        raw_depth_value * depth_unit_in_micrometers_ / 1000;
    if (depth_value_in_millimeters < kMinDepthValue ||
        depth_value_in_millimeters > kMaxDepthValue) {
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

