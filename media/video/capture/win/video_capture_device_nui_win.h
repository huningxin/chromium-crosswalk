// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_
#define MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_

#include <Shlobj.h>
#include "NuiApi.h"

#include <string>

#include "base/threading/thread.h"
#include "base/synchronization/waitable_event.h"
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
  friend class NuiCaptureHelper;

  enum CaptureMode {
    kCaptureRGBA,
    kCaptureDepth,
    kCaptureRGBD
  };

  void OnIncomingCapturedFrame(const uint8* bits, int length);
  void OnSetErrorState(const std::string& reason);

  CaptureMode capture_mode_;
  scoped_ptr<VideoCaptureDevice::Client> client_;
  Name device_name_;
  VideoCaptureFormat capture_format_;

  base::WaitableEvent stop_event_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDeviceNuiWin);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_NUI_WIN_H_

