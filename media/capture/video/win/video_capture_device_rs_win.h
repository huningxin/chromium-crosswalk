// Copyright (c) 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_CAPTURE_VIDEO_WIN_VIDEO_CAPTURE_DEVICE_RS_WIN_H_
#define MEDIA_CAPTURE_VIDEO_WIN_VIDEO_CAPTURE_DEVICE_RS_WIN_H_

#include <string>

#include "base/synchronization/lock.h"
#include "base/threading/non_thread_safe.h"
#include "media/base/media_export.h"
#include "media/capture/video/video_capture_device.h"
#include "third_party/libpxc/include/pxcsensemanager.h"

namespace media {

// Intel RealSense SDK based implementation of VideoCaptureDevice.
class MEDIA_EXPORT VideoCaptureDeviceRSWin : public base::NonThreadSafe,
                                             public VideoCaptureDevice {
 public:
  explicit VideoCaptureDeviceRSWin(const Name& device_name);
  virtual ~VideoCaptureDeviceRSWin() override;

  // VideoCaptureDevice implementation.
  void AllocateAndStart(const VideoCaptureParams& params,
                        scoped_ptr<Client> client) override;
  void StopAndDeAllocate() override;

  static void GetDeviceNames(Names* device_names);
  static void GetDeviceSupportedFormats(const Name& device,
                                        VideoCaptureFormats* formats);
 private:
  friend class SenseManagerHandler;

  pxcStatus OnNewSample(PXCCapture::Sample* sample);
  void SetErrorState(const std::string& reason);

  Name name_;
  PXCSenseManager* pxc_sense_manager_;
  SenseManagerHandler* sense_manager_handler_;

  base::Lock lock_;  // Used to guard the below variables.
  scoped_ptr<VideoCaptureDevice::Client> client_;
  VideoCaptureFormat capture_format_;
  bool capture_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDeviceRSWin);
};

}  // namespace media

#endif  // MEDIA_CAPTURE_VIDEO_WIN_VIDEO_CAPTURE_DEVICE_RS_WIN_H_
