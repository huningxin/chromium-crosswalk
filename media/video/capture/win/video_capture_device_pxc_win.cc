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

static const size_t kBytesPerPixelRGB32 = 4;

const char kPxcColorCameraDeviceName[] = "pxc_color_camera";
const char kPxcColorCameraDeviceId[] = "pxc_color_camera";
const char kPxcDepthCameraDeviceName[] = "pxc_depth_camera";
const char kPxcDepthCameraDeviceId[] = "pxc_depth_camera";

const char kPxcCaptureColorOption[] = "color";
const char kPxcCaptureDepthOption[] = "depth";

// Release a PXCSession will call CoUninitiliaze which causes calling thread
// (Audio Thread) to fail on COM API calling. Hold a global PXCSession to avoid
// this.
PXCSession* g_session = NULL;

bool VideoCaptureDevicePxcWin::PlatformSupported() {
  PXCSession *session = PXCSession::CreateInstance();
  if (!session) {
    DLOG(ERROR) << "Failed to create a PXC Session.";
    return false;
  }
  session->Release();
  return true;
}

void VideoCaptureDevicePxcWin::GetDeviceNames(Names* device_names) {
  device_names->clear();

  if (!VideoCaptureDevicePxcWin::PlatformSupported())
    return;

  Name color_camera(kPxcColorCameraDeviceName,
                    kPxcColorCameraDeviceId,
                    Name::PXC_CAPTURE);
  DLOG(INFO) << "Video capture device, " << color_camera.name()
             << " : " << color_camera.id();
  device_names->push_back(color_camera);

  Name depth_camera(kPxcDepthCameraDeviceName,
                    kPxcDepthCameraDeviceId,
                    Name::PXC_CAPTURE);
  DLOG(INFO) << "Depth capture device, " << depth_camera.name()
             << " : " << depth_camera.id();
  device_names->push_back(depth_camera);
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      pxc_sense_manager_(NULL),
      capture_type_(kCaptureColor) {
  const base::CommandLine* cmd_line = base::CommandLine::ForCurrentProcess();
  std::string type_option(
      cmd_line->GetSwitchValueASCII(switches::kPxcCaptureType));

  if (!type_option.empty()) {
    if (type_option == kPxcCaptureColorOption) {
      capture_type_ = kCaptureColor;
    } else if (type_option == kPxcCaptureDepthOption) {
      capture_type_ = kCaptureDepth;
    }
  } else {
    if (device_name_.id() == kPxcColorCameraDeviceId) {
      capture_type_ = kCaptureColor;
    } else if (device_name_.id() == kPxcDepthCameraDeviceId) {
      capture_type_ = kCaptureDepth;
    }
  }
}

VideoCaptureDevicePxcWin::~VideoCaptureDevicePxcWin() {
}

bool VideoCaptureDevicePxcWin::Init() {
  return true;
}

void VideoCaptureDevicePxcWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  DCHECK(CalledOnValidThread());
  if (state_ != kIdle)
    return;

  client_ = client.Pass();

  pxc_sense_manager_ = PXCSenseManager::CreateInstance();
  if (!pxc_sense_manager_) {
    SetErrorState("Can't create pxc sense manager.");
    return;
  }

  PXCCapture::StreamType stream_type = PXCCapture::STREAM_TYPE_COLOR;

  if (capture_type_ == kCaptureDepth) {
    stream_type = PXCCapture::STREAM_TYPE_DEPTH;
  }

  pxcStatus status =
      pxc_sense_manager_->EnableStream(
          stream_type,
          params.requested_format.frame_size.width(),
          params.requested_format.frame_size.height(),
          params.requested_format.frame_rate);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't enable stream.");
    return;
  }

  status = pxc_sense_manager_->Init(this);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't init pxc sense manager.");
    return;
  }

  status = pxc_sense_manager_->StreamFrames(false);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't stream frames.");
    return;
  }

  PXCCaptureManager* pxc_capture_manager = 
      pxc_sense_manager_->QueryCaptureManager();
  if (!pxc_capture_manager) {
    SetErrorState("Can't query capture manager.");
    return;
  }

  PXCCapture::Device* pxc_device = pxc_capture_manager->QueryDevice();
  if (!pxc_device) {
    SetErrorState("Can't query capture device.");
    return;
  }

  PXCCapture::Device::StreamProfileSet profiles = {};
  status = pxc_device->QueryStreamProfileSet(&profiles);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't query stream profiles.");
    return;
  }
  
  PXCCapture::Device::StreamProfile profile = profiles.color;

  if (capture_type_ == kCaptureDepth) {
    profile = profiles.depth;
  }

  if (!profile.imageInfo.format) {
    SetErrorState("Invalid image format.");
    return;
  }

  capture_format_.frame_size.SetSize(profile.imageInfo.width,
                                     profile.imageInfo.height);
  capture_format_.frame_rate = profile.frameRate.max;
  capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

  if (capture_type_ == kCaptureDepth) {
    depth_argb_image_.reset(
        new uint8[profile.imageInfo.width * profile.imageInfo.height *
                  kBytesPerPixelRGB32]);
  }
  state_ = kCapturing;
}

void VideoCaptureDevicePxcWin::StopAndDeAllocate() {
  DCHECK(CalledOnValidThread());
  if (state_ != kCapturing)
    return;

  pxc_sense_manager_->Close();
  pxc_sense_manager_->Release();

  pxc_sense_manager_ = NULL;
  client_.reset();
  depth_argb_image_.reset();
  state_ = kIdle;
}

void VideoCaptureDevicePxcWin::CaptureColor(PXCCapture::Sample* sample) {
  PXCImage* image = sample->color;
  if (image) {
    PXCImage::ImageInfo info = image->QueryInfo();

    PXCImage::ImageData data;
    pxcStatus status = image->AcquireAccess(
        PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access image data error");
      return;
    }

    int length = info.width * info.height * kBytesPerPixelRGB32;
    client_->OnIncomingCapturedData(
        static_cast<uint8*> (data.planes[0]),
        length, capture_format_, 0, base::TimeTicks::Now());

    image->ReleaseAccess(&data);
  }
}

void VideoCaptureDevicePxcWin::CaptureDepth(PXCCapture::Sample* sample) {
  PXCImage* image = sample->depth;
  if (image) {
    PXCImage::ImageInfo info = image->QueryInfo();

    PXCImage::ImageData data;
    pxcStatus status = image->AcquireAccess(
        PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access image data error");
      return;
    }

    int length = info.width * info.height;

    uint8* argb_data = depth_argb_image_.get();
    memset(argb_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
    int16* depth_data = reinterpret_cast<int16*>(data.planes[0]);

    for (int i = 0; i < length; ++i) {
      int16 depth_value = depth_data[i];
      // R for depth value low
      uint8 depth_value_low = static_cast<uint8>(depth_value & 0xFF);
      argb_data[i * kBytesPerPixelRGB32 + 2] = depth_value_low;
      // G for depth value high
      uint8 depth_value_high = static_cast<uint8>((depth_value >> 8) & 0xFF);
      argb_data[i * kBytesPerPixelRGB32 + 1] = depth_value_high;
    }

    client_->OnIncomingCapturedData(
        argb_data,
        length * kBytesPerPixelRGB32,
        capture_format_, 0, base::TimeTicks::Now());

    image->ReleaseAccess(&data);
  }
}

pxcStatus VideoCaptureDevicePxcWin::OnNewSample(
    pxcUID mid, PXCCapture::Sample* sample) {
  if (capture_type_ == kCaptureColor) {
    CaptureColor(sample);
  } else if (capture_type_ == kCaptureDepth) {
    CaptureDepth(sample);
  } else {
    NOTREACHED();
  }
  return PXC_STATUS_NO_ERROR;
}

void VideoCaptureDevicePxcWin::OnStatus(pxcUID mid, pxcStatus status) {
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("On error status.");
    return;
  }
}

void VideoCaptureDevicePxcWin::SetErrorState(const std::string& reason) {
  DCHECK(CalledOnValidThread());
  DVLOG(1) << reason;
  state_ = kError;
  client_->OnError(reason);
}

}  // namespace media

