// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_pxc_win.h"

#include "base/bind.h"
#include "base/logging.h"
#include "base/strings/stringprintf.h"
#include "base/strings/string_number_conversions.h"
#include "base/strings/string_split.h"
#include "base/strings/string_util.h"
#include "base/strings/utf_string_conversions.h"

namespace media {
namespace {

// Release a PXCSession will call CoUninitiliaze which causes calling thread
// (Audio Thread) to fail on COM API calling. Hold a global PXCSession to avoid
// this.
static PXCSmartPtr<PXCSession> g_session;

const char kDepthCameraName[] = "depth";
const char kDepthCameraID[] = "PXCCaptureDepthCamera";

static const size_t kBytesPerPixelRGB32 = 4;

bool GetFirstDepthDeviceAndStream(
    PXCCapture::Device** depth_device,
    PXCCapture::VideoStream** depth_stream) {
  pxcStatus status;
  PXCSession::ImplDesc video_capture_desc;
  memset(&video_capture_desc, 0, sizeof(video_capture_desc));
  video_capture_desc.group = PXCSession::IMPL_GROUP_SENSOR;
  video_capture_desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
  for (int module_index = 0; ; module_index++) {
    PXCSession::ImplDesc desc;
    status = g_session->QueryImpl(&video_capture_desc, module_index, &desc);
    if (status < PXC_STATUS_NO_ERROR) {
      DVLOG(1) << "No more PXC modules.";
      break;
    }

    PXCSmartPtr<PXCCapture> capture;
    status = g_session->CreateImpl<PXCCapture>(&desc, &capture);
    if (status < PXC_STATUS_NO_ERROR) {
      DVLOG(2) << "Failed to create PXCCapture instance.";
      continue;
    }

    for (int device_index = 0; ; device_index++) {
      PXCCapture::DeviceInfo device_info;
      status = capture->QueryDevice(device_index, &device_info);
      if (status < PXC_STATUS_NO_ERROR) {
        DVLOG(1) << "No more PXC capture devices.";
        break;
      }

      PXCSmartPtr<PXCCapture::Device> device;
      status = capture->CreateDevice(device_index, &device);
      if (status < PXC_STATUS_NO_ERROR) {
        DVLOG(2) << "Failed to create PXCCapture::Device instance.";
        continue;
      }

      for (int stream_index = 0; ; stream_index++) {
        PXCCapture::Device::StreamInfo stream_info;
        status = device->QueryStream(stream_index, &stream_info);
        if (status < PXC_STATUS_NO_ERROR) {
          DVLOG(1) << "No more PXC streams.";
          break;
        }

        if (stream_info.cuid == PXCCapture::VideoStream::CUID &&
            stream_info.imageType == PXCImage::IMAGE_TYPE_DEPTH) {
          PXCSmartPtr<PXCCapture::VideoStream> stream;
          status =
              device->CreateStream<PXCCapture::VideoStream>(
                  stream_index, &stream);
          if (status < PXC_STATUS_NO_ERROR) {
            DVLOG(2) << "Failed to create PXCCapture::VideoStream instance.";
            break;
          }

          *depth_device = device.ReleasePtr();
          *depth_stream = stream.ReleasePtr();
          return true;
        }
      }  // Enumerate streams.
    }  // Enumerate devices.
  }  // Enumerate modules.
  return false;
}

}  // namespace

// static
bool VideoCaptureDevicePxcWin::PlatformSupported() {
  if (g_session.IsValid())
    return true;

  pxcStatus status = PXCSession_Create(&g_session);
  if (status < PXC_STATUS_NO_ERROR) {
    DVLOG(2) << "Failed to create a PXC Session.";
    return false;
  }
  return true;
}

// static
void VideoCaptureDevicePxcWin::AppendDeviceNames(Names* device_names) {
  if (!VideoCaptureDevicePxcWin::PlatformSupported())
    return;

  PXCSmartPtr<PXCCapture::Device> device;
  PXCSmartPtr<PXCCapture::VideoStream> stream;
  if (!GetFirstDepthDeviceAndStream(&device, &stream)) {
    DVLOG(2) << "Failed to get PXC depth device and stream.";
    return;
  }

  Name name(kDepthCameraName, kDepthCameraID, Name::PXC_CAPTURE);
  DVLOG(1) << "PXC depth video capture device, "
           << name.name() << " : " << name.id();
  device_names->push_back(name);
}

//static
bool VideoCaptureDevicePxcWin::IsDepthDevice(const Name& device_name) {
  return device_name.name() == kDepthCameraName;
}

//static
void VideoCaptureDevicePxcWin::GetDeviceSupportedFormats(
    const Name& device_name, VideoCaptureFormats* formats) {
  PXCSmartPtr<PXCCapture::Device> device;
  PXCSmartPtr<PXCCapture::VideoStream> stream;
  if (!GetFirstDepthDeviceAndStream(&device, &stream)) {
    DVLOG(2) << "Failed to get PXC depth device and stream.";
    return;
  }

  pxcStatus status;
  for (int porfile_index = 0; ; porfile_index++) {
    PXCCapture::VideoStream::ProfileInfo video_profile;
    status = stream->QueryProfile(porfile_index, &video_profile);
    if (status < PXC_STATUS_NO_ERROR) {
      DVLOG(1) << "No more PXC stream profiles.";
      break;
    }
    VideoCaptureFormat format;
    format.frame_size.SetSize(video_profile.imageInfo.width,
                              video_profile.imageInfo.height);
     uint32 frame_rate_min =
      video_profile.frameRateMin.denominator ?
          video_profile.frameRateMin.numerator /
              video_profile.frameRateMin.denominator
                  : 0;
    uint32 frame_rate_max =
        video_profile.frameRateMax.denominator ?
            video_profile.frameRateMax.numerator /
                video_profile.frameRateMax.denominator
                    : 0;
    format.frame_rate = (frame_rate_min + frame_rate_max) / 2;
    format.pixel_format = PIXEL_FORMAT_ARGB;
    formats->push_back(format);
    DVLOG(1) << device_name.name() << " resolution: "
         << format.frame_size.ToString() << ", fps: " << format.frame_rate
         << ", pixel format: " << format.pixel_format;
  }
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      pxc_capture_thread_("PxcCaptureThread") {
}

VideoCaptureDevicePxcWin::~VideoCaptureDevicePxcWin() {
}

bool VideoCaptureDevicePxcWin::Init() {
  return true;
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

  PXCSmartPtr<PXCCapture::Device> device;
  PXCSmartPtr<PXCCapture::VideoStream> stream;

  if (!GetFirstDepthDeviceAndStream(&device, &stream)) {
    SetErrorState("Failed to get PXC depth device and stream.");
    return;
  }

  pxcStatus status;
  // Try to find the best profile.
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

    uint32 frame_rate_min =
        video_profile.frameRateMin.denominator ?
            video_profile.frameRateMin.numerator /
                video_profile.frameRateMin.denominator
                    : 0;
    uint32 frame_rate_max =
        video_profile.frameRateMax.denominator ?
            video_profile.frameRateMax.numerator /
                video_profile.frameRateMax.denominator
                    : 0;
    uint32 current =
        abs(static_cast<int>(width - video_profile.imageInfo.width)) +
        abs(static_cast<int>(height - video_profile.imageInfo.height)) +
        abs(static_cast<int>(frame_rate - frame_rate_min)) +
        abs(static_cast<int>(frame_rate - frame_rate_max));

    if (current < best) {
      best = current;
      best_profile_found = true;
      best_profile = video_profile;
    }
  }  // Enumerate profiles.

  if (!best_profile_found) {
    SetErrorState("Cannot find appropriate stream.");
    return;
  }

  status = stream->SetProfile(&best_profile);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Failed to set stream profile.");
    return;
  }
  stream_ = stream.ReleasePtr();

  device->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE,
      &depth_saturation_value_);
  device->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE,
      &depth_low_confidence_value_);
  device->QueryProperty(
      PXCCapture::Device::PROPERTY_DEPTH_UNIT,
      &depth_unit_in_micrometers_);
  device->QueryPropertyAsRange(
      PXCCapture::Device::PROPERTY_DEPTH_SENSOR_RANGE,
      &depth_range_in_millimeters_);
  DVLOG(1) << "Depth Device Properties: "
           << "\nPROPERTY_DEPTH_SATURATION_VALUE: "
           << depth_saturation_value_
           << "\nPROPERTY_DEPTH_LOW_CONFIDENCE_VALUE: "
           << depth_low_confidence_value_
           << "\nPROPERTY_DEPTH_UNIT: "
           << depth_unit_in_micrometers_
           << "\nPROPERTY_DEPTH_SENSOR_RANGE: "
           << depth_range_in_millimeters_.min << ":"
           << depth_range_in_millimeters_.max;

  DVLOG(1) << "Allocate PXC Stream: "
           << " width = " << best_profile.imageInfo.width
           << " height = " << best_profile.imageInfo.height
           << " frame_rate = " << frame_rate
           << " color = " << PIXEL_FORMAT_ARGB;

  // Store our current width and height.
  capture_format_.frame_size.SetSize(best_profile.imageInfo.width,
                                     best_profile.imageInfo.height);
  capture_format_.frame_rate = frame_rate;
  capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

  depth_rgb32_image_.reset(
    new uint8[capture_format_.frame_size.width() *
              capture_format_.frame_size.height() *
              kBytesPerPixelRGB32]);

  // Start capturing.
  state_ = kCapturing;
  pxc_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDevicePxcWin::OnCaptureTask,
                 base::Unretained(this)));
  return;
}

void VideoCaptureDevicePxcWin::OnStopAndDeAllocate() {
  DCHECK_EQ(pxc_capture_thread_.message_loop(), base::MessageLoop::current());

  state_ = kIdle;
  stream_.ReleaseRef();
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
    SetErrorState("Failed to read stream.");
    return;
  }

  status = sp->Synchronize();
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Read synchronization EOF.");
    return;
  }

  PXCImage::ImageInfo info;
  status = image->QueryInfo(&info);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Failed to get image info.");
    return;
  }

  PXCImage::ImageData data;
  status = image->AcquireAccess(
      PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_DEPTH, &data);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Failed to acquire access to image data.");
    return;
  }

  DCHECK_EQ(data.type, PXCImage::SURFACE_TYPE_SYSTEM_MEMORY);

  unsigned int rgb_data_length = info.width * info.height * kBytesPerPixelRGB32;
  uint8* rgb_data = depth_rgb32_image_.get();
  memset(rgb_data, 0, sizeof(uint8) * rgb_data_length);
  int16* depth_data = reinterpret_cast<int16*>(data.planes[0]);
  DepthToGrayscaleRGB32(depth_data, rgb_data, info.width * info.height);

  client_->OnIncomingCapturedData(
      rgb_data, rgb_data_length, capture_format_, 0, base::TimeTicks::Now());

  image->ReleaseAccess(&data);

  pxc_capture_thread_.message_loop()->PostTask(
    FROM_HERE,
    base::Bind(&VideoCaptureDevicePxcWin::OnCaptureTask,
               base::Unretained(this)));
}

void VideoCaptureDevicePxcWin::SetErrorState(const std::string& reason) {
  DVLOG(1) << reason;
  state_ = kError;
  client_->OnError(reason);
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
    if (depth_value_in_millimeters > depth_range_in_millimeters_.max) {
      continue;
    }

    // The algorithm is based on equation (4) in
    // http://iphome.hhi.de/wiegand/assets/pdfs/3d-video-depth-maps.pdf
    float value = 255.0 *
        ((1.0 / depth_value_in_millimeters -
          1.0 / depth_range_in_millimeters_.max) /
         (1.0 / depth_range_in_millimeters_.min -
          1.0 / depth_range_in_millimeters_.max));

    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 1] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 2] = static_cast<uint8>(value);
  }
}

}  // namespace media
