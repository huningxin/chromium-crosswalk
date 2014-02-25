// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_
#define MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_

#include <Shlobj.h>
#include "NuiApi.h"

#include <string>

#include "base/threading/thread.h"
#include "media/video/capture/video_capture_device.h"
#include "media/video/capture/video_capture_types.h"

namespace media {

// NUI based implementation of VideoCaptureDevice.

class VideoCaptureDeviceNuiWin : public VideoCaptureDevice {
 public:
  explicit VideoCaptureDeviceNuiWin(const Name& device_name);
  virtual ~VideoCaptureDeviceNuiWin();

  // Create the device.
  // This function is used by the static VideoCaptureDevice::Create function.
  bool Init();

  // VideoCaptureDevice implementation.
  virtual void AllocateAndStart(const VideoCaptureParams& params,
                                scoped_ptr<Client> client) OVERRIDE;

  virtual void StopAndDeAllocate() OVERRIDE;

  static bool PlatformSupported();

  static void GetDeviceNames(Names* device_names);

 private:
  enum InternalState {
    kIdle,  // The device is created but the camera is not in use.
    kCapturing,  // Video is being captured.
    kError  // Error reported by NUI API.
  };

  // Called on the nui_capture_thread_.
  void OnAllocateAndStart(int width,
                          int height,
                          int frame_rate,
                          scoped_ptr<Client> client);
  void OnStopAndDeAllocate();
  void OnCaptureTask();

  void SetErrorState(const std::string& reason);

  InternalState state_;
  scoped_ptr<VideoCaptureDevice::Client> client_;
  Name device_name_;

  // Thread used for reading data from the device.
  base::Thread nui_capture_thread_;

  VideoCaptureFormat capture_format_;

  INuiSensor *nui_sensor_;
  HANDLE nui_stream_handle_;
  HANDLE nui_nextframe_event_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDeviceNuiWin);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_

