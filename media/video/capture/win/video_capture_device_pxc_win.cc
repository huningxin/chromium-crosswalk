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

static const size_t kBytesPerPixelRGB32 = 4;

PXCCapture::VideoStream* GetFirstDepthStream() {
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

      PXCSmartPtr<PXCCapture::Device> capture_device;
      status = capture->CreateDevice(device_index, &capture_device);
      if (status < PXC_STATUS_NO_ERROR) {
        DVLOG(2) << "Failed to create PXCCapture::Device instance.";
        continue;
      }

      for (int stream_index = 0; ; stream_index++) {
        PXCCapture::Device::StreamInfo stream_info;
        status = capture_device->QueryStream(stream_index, &stream_info);
        if (status < PXC_STATUS_NO_ERROR) {
          DVLOG(1) << "No more PXC streams.";
          break;
        }

        if (stream_info.cuid == PXCCapture::VideoStream::CUID &&
            stream_info.imageType == PXCImage::IMAGE_TYPE_DEPTH) {
          PXCSmartPtr<PXCCapture::VideoStream> stream;
          status =
              capture_device->CreateStream<PXCCapture::VideoStream>(
                  stream_index, &stream);
          if (status < PXC_STATUS_NO_ERROR) {
            DVLOG(2) << "Failed to create PXCCapture::VideoStream instance.";
            break;
          }

          return stream.ReleasePtr(); 
        }
      }  // Enumerate streams.
    }  // Enumerate devices.
  }  // Enumerate modules.
  return NULL;
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

  PXCSmartPtr<PXCCapture::VideoStream> stream = GetFirstDepthStream();

  if (!stream.IsValid())
    return;

  Name name(kDepthCameraName, kDepthCameraName, Name::PXC_CAPTURE);
  DVLOG(1) << "PXC depth video capture device, " << name.name()
           << " : " << name.id();
  device_names->push_back(name);
}

//static
bool VideoCaptureDevicePxcWin::IsDepthDevice(const Name& device_name) {
  return device_name.name() == kDepthCameraName;
}

//static
void VideoCaptureDevicePxcWin::GetDeviceSupportedFormats(
    const Name& device, VideoCaptureFormats* formats) {
  PXCSmartPtr<PXCCapture::VideoStream> stream = GetFirstDepthStream();
  if (!stream.IsValid())
    return;

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
    DVLOG(1) << device.name() << " resolution: "
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

  PXCSmartPtr<PXCCapture::VideoStream> stream = GetFirstDepthStream();
  if (!stream.IsValid()) {
    SetErrorState("Failed to get PXC stream.");
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
      PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &data);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Failed to acquire access to image data.");
    return;
  }

  DCHECK_EQ(data.type, PXCImage::SURFACE_TYPE_SYSTEM_MEMORY);

  int length = info.width * info.height * kBytesPerPixelRGB32;
  client_->OnIncomingCapturedData(
      static_cast<uint8*> (data.planes[0]),
      length, capture_format_, 0, base::TimeTicks::Now());

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

}  // namespace media
