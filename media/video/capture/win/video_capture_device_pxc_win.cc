// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_pxc_win.h"

#include "base/bind.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"

namespace media {

static const size_t kBytesPerPixelRGB32 = 4;

// Release a PXCSession will call CoUninitiliaze which causes calling thread
// (Audio Thread) to fail on COM API calling. Hold a global PXCSession to avoid
// this.
static PXCSmartPtr<PXCSession> g_session;

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
      for (int stream_index = 0; ; stream_index++) {
        PXCCapture::Device::StreamInfo stream_info;
        status = capture_device->QueryStream(stream_index, &stream_info);
        if (status < PXC_STATUS_NO_ERROR)
          break;

        if (stream_info.cuid == PXCCapture::VideoStream::CUID &&
            stream_info.imageType == PXCImage::IMAGE_TYPE_COLOR) {
          found_color_stream = true;
          break;
        }
      }

      if (found_color_stream) {
        Name name(base::WideToUTF8(std::wstring(device_info.name)),
                  base::WideToUTF8(std::wstring(device_info.did)),
                  Name::PXC_CAPTURE);
        DLOG(INFO) << "Video capture device, " << name.name()
                   << " : " << name.id();
        device_names->push_back(name);
      }
    }  // Enumerate devices.
  }  // Enumerate modules.
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      pxc_capture_thread_("PxcCaptureThread") {
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

      if (base::WideToUTF8(std::wstring(device_info.name)) != device_name_.name() ||
          base::WideToUTF8(std::wstring(device_info.did)) != device_name_.id())
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

    if (stream_info.cuid != PXCCapture::VideoStream::CUID ||
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
  status = image->AcquireAccess(
      PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &data);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Access image data error");
    return;
  }

  DCHECK_EQ(data.type, PXCImage::SURFACE_TYPE_SYSTEM_MEMORY);

  int length = info.width * info.height * kBytesPerPixelRGB32;
  client_->OnIncomingCapturedFrame(
      static_cast<uint8*> (data.planes[0]),
      length, base::TimeTicks::Now(), 0, capture_format_);

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

