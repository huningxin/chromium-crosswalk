// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_DS_LINUX_H_
#define MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_DS_LINUX_H_

#include <string>

#include <DepthSense.hxx>

#include "base/memory/scoped_ptr.h"
#include "base/synchronization/waitable_event.h"
#include "base/threading/thread.h"
#include "media/video/capture/video_capture_device.h"
#include "media/video/capture/video_capture_types.h"

namespace media {

class VideoCaptureDeviceDsLinux : public VideoCaptureDevice {
 public:
  explicit VideoCaptureDeviceDsLinux(const Name& device_name);
  virtual ~VideoCaptureDeviceDsLinux();

  // VideoCaptureDevice implementation.
  virtual void AllocateAndStart(const VideoCaptureParams& params,
                                scoped_ptr<Client> client) OVERRIDE;

  virtual void StopAndDeAllocate() OVERRIDE;

  static bool PlatformSupported();

  static void GetDeviceNames(Names* device_names);

 private:
  // called from Depth Sense event loop.
  friend void OnNewColorSample(
    DepthSense::ColorNode node,
    DepthSense::ColorNode::NewSampleReceivedData data);
  friend void OnNewDepthSample(
    DepthSense::DepthNode node,
    DepthSense::DepthNode::NewSampleReceivedData data);
  friend void OnNewSyncColorSample(
    DepthSense::ColorNode node,
    DepthSense::ColorNode::NewSampleReceivedData data);
  friend void OnNewSyncDepthSample(
    DepthSense::DepthNode node,
    DepthSense::DepthNode::NewSampleReceivedData data);

  void OnAllocateAndStart();
  void OnColorImage(int length, const uint8* yuy2);
  void OnDepthImage(int length, const int16* depth, const DepthSense::UV* uv);
  void OnError(const std::string& reason);

 private:
  enum CaptureMode {
    kCaptureColor,
    kCaptureDepth,
    kCaptureRGBD
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



  CaptureMode capture_mode_;
  scoped_ptr<VideoCaptureDevice::Client> client_;
  Name device_name_;
  DepthEncoding depth_encoding_;

  bool is_capturing_;
  base::Thread ds_capture_thread_;

  VideoCaptureFormat capture_format_;

  // Working image in RGB32 format.
  scoped_ptr<uint8[]> rgb32_image_;
  scoped_ptr<uint8[]> yuy2_image_;

  DepthSense::Context ds_context_;
  DepthSense::ColorNode ds_color_node_;
  DepthSense::DepthNode ds_depth_node_;

  DISALLOW_IMPLICIT_CONSTRUCTORS(VideoCaptureDeviceDsLinux);
};

}  // namespace media

#endif  // MEDIA_VIDEO_CAPTURE_LINUX_VIDEO_CAPTURE_DEVICE_DS_LINUX_H_

