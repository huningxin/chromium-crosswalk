// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_nui_win.h"

#include "base/bind.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"

namespace media {

static const char kKinectColorDeviceName[] = "KinectColorStream";

bool VideoCaptureDeviceNuiWin::PlatformSupported() {
  int sensor_count = 0;
  HRESULT result = NuiGetSensorCount(&sensor_count);
  if (FAILED(result))
    return false;
  if (sensor_count <= 0)
    return false;
  return true;
}

void VideoCaptureDeviceNuiWin::GetDeviceNames(Names* device_names) {
  device_names->clear();

  int sensor_count = 0;
  HRESULT result = NuiGetSensorCount(&sensor_count);
  if (FAILED(result)) {
    return;
  }

  if (sensor_count <= 0)
    return;

  Name name(kKinectColorDeviceName,
            kKinectColorDeviceName,
            Name::NUI_CAPTURE);
  DLOG(INFO) << "Video capture device, " << name.name()
             << " : " << name.id();
  device_names->push_back(name);
}

VideoCaptureDeviceNuiWin::VideoCaptureDeviceNuiWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      nui_capture_thread_("NuiCaptureThread"),
      nui_sensor_(NULL),
      nui_stream_handle_(INVALID_HANDLE_VALUE),
      nui_nextframe_event_(INVALID_HANDLE_VALUE) {
}

VideoCaptureDeviceNuiWin::~VideoCaptureDeviceNuiWin() {
}

bool VideoCaptureDeviceNuiWin::Init() {
  if (device_name_.name() != std::string(kKinectColorDeviceName))
    return false;

  int sensor_count = 0;
  HRESULT result = NuiGetSensorCount(&sensor_count);
  if (FAILED(result)) {
    return false;
  }

  if (sensor_count <= 0)
    return false;

  INuiSensor *nui_sensor;
  for (int i = 0; i < sensor_count; ++i) {
    result = NuiCreateSensorByIndex(i, &nui_sensor);
    if (FAILED(result)) {
      continue;
    }

    result = nui_sensor->NuiStatus();
    if (S_OK == result) {
      nui_sensor_ = nui_sensor;
      break;
    }

    nui_sensor->Release();
  }

  if (nui_sensor_ == NULL)
    return false;

  return true;
}

void VideoCaptureDeviceNuiWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  if (nui_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  nui_capture_thread_.Start();
  nui_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDeviceNuiWin::OnAllocateAndStart,
                 base::Unretained(this),
                 params.requested_format.frame_size.width(),
                 params.requested_format.frame_size.height(),
                 params.requested_format.frame_rate,
                 base::Passed(&client)));
}

void VideoCaptureDeviceNuiWin::StopAndDeAllocate() {
  if (!nui_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  nui_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDeviceNuiWin::OnStopAndDeAllocate,
                 base::Unretained(this)));
  nui_capture_thread_.Stop();
}

void VideoCaptureDeviceNuiWin::OnAllocateAndStart(
    int width, int height, int frame_rate, scoped_ptr<Client> client) {
  DCHECK_EQ(nui_capture_thread_.message_loop(), base::MessageLoop::current());

  client_ = client.Pass();

  HRESULT result = nui_sensor_->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR); 
  if (SUCCEEDED(result)) {
    nui_nextframe_event_ = CreateEvent(NULL, TRUE, FALSE, NULL);
    result = nui_sensor_->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_COLOR,
        NUI_IMAGE_RESOLUTION_640x480,
        0,
        2,
        nui_nextframe_event_,
        &nui_stream_handle_);

    if (SUCCEEDED(result)) {
      // TODO(nhu): fix the potential color conversions caused by hardcoding
      //            PIXEL_FORMAT_ARGB.
      DLOG(INFO) << "VideoCaptureFormat: "
                 << " width = " << 640
                 << " height = " << 480
                 << " frame_rate = " << 30
                 << " color = " << PIXEL_FORMAT_ARGB;

        // Store our current width and height.
      capture_format_.frame_size.SetSize(640,
                                         480);
      capture_format_.frame_rate = 30;
      capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

      // Start capturing.
      state_ = kCapturing;
      nui_capture_thread_.message_loop()->PostTask(
          FROM_HERE,
          base::Bind(&VideoCaptureDeviceNuiWin::OnCaptureTask,
                     base::Unretained(this)));
      return;
    }
  }

  SetErrorState("Cannot find appropriate stream.");
}

void VideoCaptureDeviceNuiWin::OnStopAndDeAllocate() {
  DCHECK_EQ(nui_capture_thread_.message_loop(), base::MessageLoop::current());

  state_ = kIdle;
  if (nui_nextframe_event_ != INVALID_HANDLE_VALUE) {
    CloseHandle(nui_nextframe_event_);
  }

  if (nui_stream_handle_ != INVALID_HANDLE_VALUE) {
    CloseHandle(nui_stream_handle_);
  }
    
  if (nui_sensor_ != NULL) {
    nui_sensor_->NuiShutdown();
    nui_sensor_->Release();
    nui_sensor_ = NULL;
  }

  client_.reset();
}

void VideoCaptureDeviceNuiWin::OnCaptureTask() {
  DCHECK_EQ(nui_capture_thread_.message_loop(), base::MessageLoop::current());

  if (state_ != kCapturing)
    return;

  DWORD return_value;

  return_value = WaitForSingleObject(nui_nextframe_event_, INFINITE);

  if (return_value != WAIT_OBJECT_0) {
    SetErrorState("next frame event error");
    return;
  }

  NUI_IMAGE_FRAME nui_image_frame;
  HRESULT  hresult = nui_sensor_->NuiImageStreamGetNextFrame(
      nui_stream_handle_, 0, &nui_image_frame);
  if (FAILED(hresult)) {
    SetErrorState("get image frame error");
    return;
  }

  INuiFrameTexture *nui_frame_texture = nui_image_frame.pFrameTexture;
  NUI_LOCKED_RECT nui_lock_rect;

  nui_frame_texture->LockRect(0, &nui_lock_rect, NULL, 0);

  if (nui_lock_rect.Pitch != 0) {
    int length = nui_lock_rect.size;
    client_->OnIncomingCapturedFrame(
        static_cast<uint8*> (nui_lock_rect.pBits),
        length, base::TimeTicks::Now(), 0, capture_format_);
  }

  nui_frame_texture->UnlockRect(0);

  nui_sensor_->NuiImageStreamReleaseFrame(nui_stream_handle_, &nui_image_frame);

  nui_capture_thread_.message_loop()->PostTask(
    FROM_HERE,
    base::Bind(&VideoCaptureDeviceNuiWin::OnCaptureTask,
               base::Unretained(this)));
}

void VideoCaptureDeviceNuiWin::SetErrorState(const std::string& reason) {
  DVLOG(1) << reason;
  state_ = kError;
  client_->OnError(reason);
}

}  // namespace media

