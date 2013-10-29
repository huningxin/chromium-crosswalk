// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_
#define MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

#include <string>

#include "pxccapture.h"  // NOLINT(*)
#include "pxcgesture.h"  // NOLINT(*)
#include "pxcsmartptr.h"  // NOLINT(*)

#include "base/threading/thread.h"
#include "media/video/capture/video_capture_device.h"
#include "media/video/capture/video_capture_types.h"

namespace media {

// PXCCapture based implementation of VideoCaptureDevice.
// PXCCapture does not provide its own thread for capturing so this
// implementation uses a Chromium thread for fetching frames.

class VideoCaptureDevicePxcWin : public VideoCaptureDevice {
 public:
  explicit VideoCaptureDevicePxcWin(const Name& device_name);
  virtual ~VideoCaptureDevicePxcWin();

  // Create the PXCCapture::Device.
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
    kError  // Error reported by PXCCapture API.
  };

  // Called on the pxc_capture_thread_.
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
  base::Thread pxc_capture_thread_;

  PXCSmartPtr<PXCCapture::Device> device_;
  PXCSmartPtr<PXCCapture::VideoStream> stream_;

  VideoCaptureFormat capture_format_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDevicePxcWin);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

