// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_pxc_win.h"

#include "base/bind.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"

namespace media {

static const size_t kBytesPerPixelRGB32 = 4;

const char kPxcColorCameraDeviceName[] = "pxc_color_camera";
const char kPxcColorCameraDeviceId[] = "pxc_color_camera";

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
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      pxc_sense_manager_(NULL) {
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

  pxcStatus status =
      pxc_sense_manager_->EnableStream(
          PXCCapture::STREAM_TYPE_COLOR,
          params.requested_format.frame_size.width(),
          params.requested_format.frame_size.height(),
          params.requested_format.frame_rate);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't enable color stream.");
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
  if (!profile.imageInfo.format) {
    SetErrorState("Invalid image format.");
    return;
  }

  capture_format_.frame_size.SetSize(profile.imageInfo.width,
                                     profile.imageInfo.height);
  capture_format_.frame_rate = profile.frameRate.max;
  capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

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
  state_ = kIdle;
}

pxcStatus VideoCaptureDevicePxcWin::OnNewSample(
    pxcUID mid, PXCCapture::Sample* sample) {
  pxcStatus status;
  if (sample->color) {
    PXCImage* image = sample->color;
    PXCImage::ImageInfo info = image->QueryInfo();

    PXCImage::ImageData data;
    status = image->AcquireAccess(
        PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access image data error");
      return PXC_STATUS_NO_ERROR;
    }

    int length = info.width * info.height * kBytesPerPixelRGB32;
    client_->OnIncomingCapturedData(
        static_cast<uint8*> (data.planes[0]),
        length, capture_format_, 0, base::TimeTicks::Now());

    image->ReleaseAccess(&data);
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

