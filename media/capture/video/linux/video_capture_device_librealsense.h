// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_LIBREALSENSE_H_
#define MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_LIBREALSENSE_H_

#include <stdint.h>

#include <string>

#include "base/files/file_util.h"
#include "base/files/scoped_file.h"
#include "base/macros.h"
#include "base/memory/ref_counted.h"
#include "base/threading/thread.h"
#include "media/base/video_capture_types.h"
#include "media/capture/video/video_capture_device.h"

namespace media {

class RsCaptureDelegate;

// librealsense implementation of VideoCaptureDevice.
class VideoCaptureDeviceLibrealsense : public VideoCaptureDevice {
 public:
  static void GetDeviceNames(Names* device_names);
  static void GetDeviceSupportedFormats(const Name& device,
                                        VideoCaptureFormats* formats);

  explicit VideoCaptureDeviceLibrealsense(const Name& device_name);
  ~VideoCaptureDeviceLibrealsense() override;
  

  // VideoCaptureDevice implementation.
  void AllocateAndStart(const VideoCaptureParams& params,
                        scoped_ptr<Client> client) override;
  void StopAndDeAllocate() override;

 private:
  base::Thread rs_thread_;  // Thread used for reading data from the device.

  const Name device_name_;

  scoped_refptr<RsCaptureDelegate> capture_impl_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDeviceLibrealsense);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_LIBREALSENSE_H_
