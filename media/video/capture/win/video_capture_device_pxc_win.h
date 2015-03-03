// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_
#define MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

#include <string>

#include "pxcsensemanager.h"
#include "pxcprojection.h"

#include "base/threading/non_thread_safe.h"
#include "base/threading/thread.h"
#include "media/video/capture/video_capture_device.h"
#include "media/video/capture/video_capture_types.h"

namespace media {

// RSSDK based implementation of VideoCaptureDevice.
class VideoCaptureDevicePxcWin : 
    public base::NonThreadSafe,
    public VideoCaptureDevice,
    public PXCSenseManager::Handler {
 public:
  explicit VideoCaptureDevicePxcWin(const Name& device_name);
  virtual ~VideoCaptureDevicePxcWin();

  // Opens the device driver for this device.
  bool Init();

  // VideoCaptureDevice implementation.
  void AllocateAndStart(const VideoCaptureParams& params,
                        scoped_ptr<Client> client) override;
  void StopAndDeAllocate() override;

  static bool PlatformSupported();
  static void GetDeviceNames(Names* device_names);

  // PXCSenseManager::Handler implementation.
  virtual pxcStatus PXCAPI OnNewSample(pxcUID mid, PXCCapture::Sample* sample);
  virtual void PXCAPI OnStatus(pxcUID mid, pxcStatus status);

 private:
  enum InternalState {
    kIdle,  // The device driver is opened but camera is not in use.
    kCapturing,  // Video is being captured.
    kError  // Error accessing HW functions.
            // User needs to recover by destroying the object.
  };

  enum CaptureType {
    kCaptureColor,
    kCaptureDepth,
    kCaptureAlignedDepth
  };

  enum DepthEncoding {
    // Convert 16-bit depth value into 1 8-bit channel.
    kGrayscaleRGB32,
    // Map 16-bit depth value into 3 8-bit channels.
    kRawRGB32,
    // Implement the ecoding scheme proposed in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    kAdaptiveRGB32
  };

  void SetErrorState(const std::string& reason);

  void CaptureColor(PXCCapture::Sample* sample);
  void CaptureDepth(PXCCapture::Sample* sample);
  void CaptureAlignedDepth(PXCCapture::Sample* sample);
  void CaptureDepthCommon(PXCImage* image);

  void EncodeDepthToGrayscaleRGB32(uint8* argb, uint16 depth);
  void EncodeDepthToRawRGB32(uint8* argb, uint16 depth);
  void EncodeDepthToAdaptiveRGB32(uint8* argb, uint16 depth);

  Name device_name_;
  InternalState state_;
  scoped_ptr<VideoCaptureDevice::Client> client_;
  VideoCaptureFormat capture_format_;
  CaptureType capture_type_;
  DepthEncoding depth_encoding_;

  scoped_ptr<uint8[]> depth_argb_image_;

  PXCSenseManager* pxc_sense_manager_;
  PXCProjection* pxc_projection_;

  pxcF32 depth_low_confidence_value_;
  PXCRangeF32 depth_range_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDevicePxcWin);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_WIN_VIDEO_CAPTURE_DEVICE_PXC_WIN_H_

