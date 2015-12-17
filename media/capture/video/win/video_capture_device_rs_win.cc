// Copyright (c) 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/capture/video/win/video_capture_device_rs_win.h"

#include "base/logging.h"
#include "base/memory/ref_counted.h"
#include "base/strings/utf_string_conversions.h"

namespace media {

static const size_t kBytesPerPixelRGB32 = 4;

const char kRsColorCameraName[] = "RS Color Camera";
const char kRsColorCameraId[] = "RS Color Camera ID";

void VideoCaptureDeviceRSWin::GetDeviceNames(Names* device_names) {
  device_names->push_back(
            Name(kRsColorCameraName,
                 kRsColorCameraId,
                 Name::RSSDK));
}

void VideoCaptureDeviceRSWin::GetDeviceSupportedFormats(
    const Name& device, VideoCaptureFormats* formats) {

}

class SenseManagerHandler : public PXCSenseManager::Handler {
 public:
  SenseManagerHandler(VideoCaptureDeviceRSWin* vcd_rs_win)
      : vcd_rs_win_(vcd_rs_win) {}
  virtual ~SenseManagerHandler() {}

  // PXCSenseManager::Handler implementation.
  virtual pxcStatus PXCAPI OnNewSample(pxcUID mid, PXCCapture::Sample* sample) {
    return vcd_rs_win_->OnNewSample(sample);
  }
  virtual void PXCAPI OnStatus(pxcUID mid, pxcStatus status) {
    if (status < PXC_STATUS_NO_ERROR) {
      vcd_rs_win_->SetErrorState("On error status.");
      return;
    }
  }
 private:
  VideoCaptureDeviceRSWin* vcd_rs_win_;
};

VideoCaptureDeviceRSWin::VideoCaptureDeviceRSWin(const Name& device_name)
    : name_(device_name),
      pxc_sense_manager_(NULL),
      sense_manager_handler_(NULL),
      capture_(false) {
  DetachFromThread();
}

VideoCaptureDeviceRSWin::~VideoCaptureDeviceRSWin() {
  DCHECK(CalledOnValidThread());
}

void VideoCaptureDeviceRSWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  DCHECK(CalledOnValidThread());

  base::AutoLock lock(lock_);

  client_ = client.Pass();
  DCHECK_EQ(capture_, false);

  pxc_sense_manager_ = PXCSenseManager::CreateInstance();
  if (!pxc_sense_manager_) {
    SetErrorState("Can't create sense manager.");
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

  sense_manager_handler_ = new SenseManagerHandler(this);

  status = pxc_sense_manager_->Init(sense_manager_handler_);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't init sense manager.");
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
  capture_format_.pixel_format = VIDEO_CAPTURE_PIXEL_FORMAT_ARGB;

  capture_ = true;
}

void VideoCaptureDeviceRSWin::StopAndDeAllocate() {
  DCHECK(CalledOnValidThread());
  {
    base::AutoLock lock(lock_);

    if (capture_) {
      capture_ = false;

      pxc_sense_manager_->Close();
      pxc_sense_manager_->Release();

      delete sense_manager_handler_;
      pxc_sense_manager_ = NULL;
    }
    client_.reset();
  }
}

pxcStatus VideoCaptureDeviceRSWin::OnNewSample(PXCCapture::Sample* sample) {
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

void VideoCaptureDeviceRSWin::SetErrorState(const std::string& reason) {
  DVLOG(1) << reason;
  client_->OnError(reason);
}

}  // namespace media
