// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_
#define MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

#include <string>

#include "base/threading/thread.h"
#include "media/video/capture/video_capture_device.h"
#include "media/video/capture/video_capture_types.h"
#include "third_party/libpxc/include/pxccapture.h"
#include "third_party/libpxc/include/pxcsmartptr.h"

namespace media {

// PXCCapture based implementation of VideoCaptureDevice for "depth" camera.
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
  static void AppendDeviceNames(Names* device_names);
  static bool IsDepthDevice(const Name& device_name);
  static void GetDeviceSupportedFormats(const Name& device,
                                        VideoCaptureFormats* formats);

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

  void DepthToGrayscaleRGB32(int16* depth, uint8* rgb, unsigned int length);

  InternalState state_;
  scoped_ptr<VideoCaptureDevice::Client> client_;
  Name device_name_;

  // Thread used for reading data from the device.
  base::Thread pxc_capture_thread_;

  PXCSmartPtr<PXCCapture::VideoStream> stream_;
  VideoCaptureFormat capture_format_;

  // Depth camera properties.
  pxcF32 depth_saturation_value_;
  pxcF32 depth_low_confidence_value_;
  pxcF32 depth_unit_in_micrometers_;
  PXCRangeF32 depth_range_in_millimeters_;

  // Depth image in RGB32 format.
  scoped_ptr<uint8[]> depth_rgb32_image_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDevicePxcWin);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

