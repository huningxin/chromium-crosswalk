// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/linux/video_capture_device_ds_linux.h"

#include "base/bind.h"
#include "base/command_line.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"
#include "media/base/media_switches.h"

namespace media {

const char kDsColorDeviceName[] = "DepthSenseColorStream";
const char kDsDepthDeviceName[] = "DepthSenseDepthStream";

// This needs to be aligned with definition in
// content/renderer/media/media_stream_impl.cc
const char kVirtualColorDeviceId[] = "color";
const char kVirtualDepthDeviceId[] = "depth";
const char kVirtualRgbdDeviceId[] = "rgbd";

static const size_t kBytesPerPixelYUY2 = 2;
static const size_t kBytesPerPixelRGB32 = 4;

static const int16 kInvalidDepthValue = 32002;

// XXX: hardcode
static const int kWidth = 320;
static const int kHeight = 240;
static const int kFrameRate = 30;

// TODO(nhu): the min and max depth value should be obtained from PCSDK API.
static const int kMinDepthValue = 150;
static const int kMaxDepthValue = 3000;

// Depth encoding in RGB32
const char kDepthEncodingGrayscale[] = "grayscale";
const char kDepthEncodingRaw[] = "raw";
const char kDepthEncodingAdaptive[] = "adaptive";

// dirty hack
static VideoCaptureDeviceDsLinux* g_color_capture_device = NULL;
static VideoCaptureDeviceDsLinux* g_depth_capture_device = NULL;
static VideoCaptureDeviceDsLinux* g_rgbd_capture_device = NULL;

static void DepthToGrayscaleRGB32(
    const int16* depth, uint8* rgb, int length) {
  for (int i = 0; i < length; i++) {
    int depth_value = depth[i];
    if (depth_value == kInvalidDepthValue) {
      continue;
    }

    if (depth_value < kMinDepthValue ||
        depth_value > kMaxDepthValue) {
      continue;
    }

    // Implement the algorithm as equation (4) in paper
    // "3-D Video Representation Using Depth Maps".
    float value = 255.0 *
        ((1.0 / depth_value -
          1.0 / kMaxDepthValue) /
         (1.0 / kMinDepthValue -
          1.0 / kMaxDepthValue));

    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 1] = static_cast<uint8>(value);
    rgb[i * kBytesPerPixelRGB32 + 2] = static_cast<uint8>(value);
  }
}

static void DepthToRawRGB32(const int16* depth, uint8* rgb, int length) {
  for (int i = 0; i < length; i++) {
    int16 d = depth[i];
    if (d == kInvalidDepthValue) {
      continue;
    }

    // Implement the BIT2 scheme in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] =
        static_cast<uint8>(((d & 0xFC00) >> 8) & 0xFF);
    rgb[i * kBytesPerPixelRGB32 + 1] =
        static_cast<uint8>(((d & 0x03E0) >> 2) & 0xFF);
    rgb[i * kBytesPerPixelRGB32 + 2] =
        static_cast<uint8>(((d & 0x001F) << 3) & 0xFF);
  }
}

static void DepthToAdaptiveRGB32(
    const int16* depth, uint8* rgb, unsigned int length) {
  const double np = 512.0;
  const double w = kMaxDepthValue;
  double p = np / w;
  for (unsigned int i = 0; i < length; i++) {
    int16 d = depth[i];
    if (d == kInvalidDepthValue) {
      continue;
    }

    if (d > w) {
      continue;
    }

    // Implement the depth ecoding schema described in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
    double ld = (d + 0.5) / w;

    double ha = fmod(ld / (p / 2.0), 2.0);
    if (ha > 1.0)
      ha = 2.0 - ha;

    double hb = fmod((ld - p / 4.0) / (p / 2.0), 2.0);
    if (hb > 1.0)
      hb = 2.0 - hb;

    // Layout is BGRA.
    rgb[i * kBytesPerPixelRGB32 + 0] = static_cast<uint8>(255.0 * ld);
    rgb[i * kBytesPerPixelRGB32 + 1] = static_cast<uint8>(255.0 * ha);
    rgb[i * kBytesPerPixelRGB32 + 2] = static_cast<uint8>(255.0 * hb);
  }
}

void OnNewSyncColorSample(
    DepthSense::ColorNode node,
    DepthSense::ColorNode::NewSampleReceivedData data) {
  if (g_rgbd_capture_device) {
    if (!g_rgbd_capture_device->is_capturing_) {
      DLOG(INFO) << "quiting rgbd evnet loop.";
      g_rgbd_capture_device->ds_context_.quit();
    } else {
      g_rgbd_capture_device->OnColorImage(data.colorMap.size(), data.colorMap);
    }
  }
}

void OnNewSyncDepthSample(
    DepthSense::DepthNode node,
    DepthSense::DepthNode::NewSampleReceivedData data) {
  if (g_rgbd_capture_device && g_rgbd_capture_device->is_capturing_) {
    g_rgbd_capture_device->OnDepthImage(data.depthMap.size(),
                                        data.depthMap,
                                        data.uvMap);
  }
}

void OnNewColorSample(
    DepthSense::ColorNode node,
    DepthSense::ColorNode::NewSampleReceivedData data) {
  if (g_color_capture_device) {
    if (!g_color_capture_device->is_capturing_) {
      DLOG(INFO) << "quiting color evnet loop.";
      g_color_capture_device->ds_context_.quit();
    } else {
      g_color_capture_device->OnColorImage(data.colorMap.size(), data.colorMap);
    }
  }
}

void OnNewDepthSample(
    DepthSense::DepthNode node,
    DepthSense::DepthNode::NewSampleReceivedData data) {
  if (g_depth_capture_device) {
    if (!g_depth_capture_device->is_capturing_) {
      DLOG(INFO) << "quiting depth evnet loop.";
      g_depth_capture_device->ds_context_.quit();
    } else {
      g_depth_capture_device->OnDepthImage(data.depthMap.size(),
                                           data.depthMap,
                                           data.uvMap);
    }
  } 
}

bool VideoCaptureDeviceDsLinux::PlatformSupported() {
  DepthSense::Context context = DepthSense::Context::create();

  std::vector<DepthSense::Device> devices = context.getDevices();

  if (devices.size() >= 1)
    return true;
  return false;
}

void VideoCaptureDeviceDsLinux::GetDeviceNames(Names* device_names) {
  device_names->clear();

  if (!VideoCaptureDeviceDsLinux::PlatformSupported())
    return;
  Name color_device_name(kDsColorDeviceName,
                         kVirtualColorDeviceId);
  DLOG(INFO) << "Video capture device, " << color_device_name.name()
             << " : " << color_device_name.id();
  device_names->push_back(color_device_name);

  Name depth_device_name(kDsDepthDeviceName,
                         kVirtualDepthDeviceId);
  DLOG(INFO) << "Depth capture device, " << depth_device_name.name()
             << " : " << depth_device_name.id();
  device_names->push_back(depth_device_name);

  Name rgbd_device_name(kDsColorDeviceName,
                        kVirtualRgbdDeviceId);
  DLOG(INFO) << "RGBD capture device, " << rgbd_device_name.name()
             << " : " << rgbd_device_name.id();
  device_names->push_back(rgbd_device_name);
}

VideoCaptureDeviceDsLinux::VideoCaptureDeviceDsLinux(const Name& device_name)
    : device_name_(device_name),
      is_capturing_(false),
      ds_capture_thread_("DepthSenseCaptureThread") {
  DLOG(INFO) << device_name.name() << ": " << device_name.id();
  if (device_name.id() == kVirtualDepthDeviceId) {
    capture_mode_ = kCaptureDepth;
    const CommandLine* cmd_line = CommandLine::ForCurrentProcess();
    std::string encoding_option(
        cmd_line->GetSwitchValueASCII(switches::kDepthEncoding));
    if (encoding_option == kDepthEncodingGrayscale) {
      depth_encoding_ = kGrayscaleRGB32;
    } else if (encoding_option == kDepthEncodingRaw) {
      depth_encoding_ = kRawRGB32;
    } else if (encoding_option == kDepthEncodingAdaptive) {
      depth_encoding_ = kAdaptiveRGB32;
    } else {
      depth_encoding_ = kGrayscaleRGB32;
    }
    rgb32_image_.reset(new uint8[kWidth * kHeight * kBytesPerPixelRGB32]);
    capture_format_.frame_size.SetSize(kWidth, kHeight);
    capture_format_.frame_rate = kFrameRate;
    capture_format_.pixel_format = PIXEL_FORMAT_ARGB;
  } else if (device_name.id() == kVirtualRgbdDeviceId) {
    capture_mode_ = kCaptureRGBD;
    rgb32_image_.reset(new uint8[kWidth * kHeight * kBytesPerPixelRGB32]);
    yuy2_image_.reset(new uint8[kWidth * kHeight * kBytesPerPixelYUY2]);
    capture_format_.frame_size.SetSize(kWidth, kHeight);
    capture_format_.frame_rate = kFrameRate;
    capture_format_.pixel_format = PIXEL_FORMAT_YUY2;
  } else {
    capture_mode_ = kCaptureColor;
    capture_format_.frame_size.SetSize(kWidth, kHeight);
    capture_format_.frame_rate = kFrameRate;
    capture_format_.pixel_format = PIXEL_FORMAT_YUY2;
  }
}

VideoCaptureDeviceDsLinux::~VideoCaptureDeviceDsLinux() {
}

void VideoCaptureDeviceDsLinux::OnAllocateAndStart() {
  ds_context_ = DepthSense::Context::create();

  std::vector<DepthSense::Device> devices = ds_context_.getDevices();

  if (devices.size() == 0) {
    OnError("cannot get depth sense devices.");
    return;
  }

  std::vector<DepthSense::Node> nodes = devices[0].getNodes();

  for (unsigned int i = 0; i < nodes.size(); i++) {
    if (nodes[i].is<DepthSense::ColorNode>() &&
        (capture_mode_ == kCaptureColor ||
         capture_mode_ == kCaptureRGBD)) {
      if (ds_color_node_.isSet())
        continue;
      ds_color_node_ = nodes[i].as<DepthSense::ColorNode>();

      DepthSense::ColorNode::Configuration config =
          ds_color_node_.getConfiguration();
      config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
      config.compression = DepthSense::COMPRESSION_TYPE_YUY2;
      config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
      config.framerate = kFrameRate;

      ds_color_node_.setEnableColorMap(true);
      ds_context_.requestControl(ds_color_node_, 0);
      ds_color_node_.setConfiguration(config);
      ds_context_.releaseControl(ds_color_node_);

      ds_context_.registerNode(ds_color_node_);

      DLOG(INFO) << "color node is registered.";
    } else if (nodes[i].is<DepthSense::DepthNode>() &&
               (capture_mode_ == kCaptureDepth ||
                capture_mode_ == kCaptureRGBD)) {
      if (ds_depth_node_.isSet())
        continue;
      ds_depth_node_ = nodes[i].as<DepthSense::DepthNode>();

      DepthSense::DepthNode::Configuration config =
          ds_depth_node_.getConfiguration();
      config.framerate = kFrameRate;
      config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
      config.saturation = true;

      ds_depth_node_.setEnableDepthMap(true);
      ds_depth_node_.setEnableUvMap(true);
      ds_depth_node_.setEnableVerticesFloatingPoint(true);
      ds_context_.requestControl(ds_depth_node_, 0);
      ds_depth_node_.setConfiguration(config);
      ds_context_.releaseControl(ds_depth_node_);

      ds_context_.registerNode(ds_depth_node_);

      DLOG(INFO) << "depth node is registered.";
    }
  }

  if (ds_color_node_.isSet() || ds_depth_node_.isSet()) {
    is_capturing_ = true;

    if (capture_mode_ == kCaptureColor) {
      DCHECK(ds_color_node_.isSet());
      ds_color_node_.newSampleReceivedEvent().connect(&OnNewColorSample);
      g_color_capture_device = this;
    } else if (capture_mode_ == kCaptureDepth) {
      DCHECK(ds_depth_node_.isSet());
      ds_depth_node_.newSampleReceivedEvent().connect(&OnNewDepthSample);
      g_depth_capture_device = this;
    } else if (capture_mode_ == kCaptureRGBD) {
      DCHECK(ds_color_node_.isSet() && ds_depth_node_.isSet());
      ds_color_node_.newSampleReceivedEvent().connect(&OnNewSyncColorSample);
      ds_depth_node_.newSampleReceivedEvent().connect(&OnNewSyncDepthSample);
      g_rgbd_capture_device = this;
    }

    ds_context_.startNodes();
    ds_context_.run();
    DLOG(INFO) << "ds event loop exists.";

    ds_context_.stopNodes();

    if (ds_color_node_.isSet())
      ds_context_.unregisterNode(ds_color_node_);

    if (ds_depth_node_.isSet())
      ds_context_.unregisterNode(ds_depth_node_);

    if (capture_mode_ == kCaptureColor)
      g_color_capture_device = NULL;
    else if (capture_mode_ == kCaptureDepth)
      g_depth_capture_device = NULL;
    else if (capture_mode_ == kCaptureRGBD)
      g_rgbd_capture_device = NULL;

    ds_context_.unset();
    return;
  }

  OnError("cannot start depth sense nodes");
}
void VideoCaptureDeviceDsLinux::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  client_ = client.Pass();

  if (ds_capture_thread_.IsRunning()) {
    return;  // Wrong state.
  }
  ds_capture_thread_.Start();
  ds_capture_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&VideoCaptureDeviceDsLinux::OnAllocateAndStart,
                 base::Unretained(this)));
}

void VideoCaptureDeviceDsLinux::StopAndDeAllocate() {
  DLOG(INFO) << "stop and deallocate";
  is_capturing_ = false;
  ds_capture_thread_.Stop();
  DLOG(INFO) << device_name_.name() << "ds thread stops.";

  client_.reset();
}

void VideoCaptureDeviceDsLinux::OnError(const std::string& reason) {
  DVLOG(1) << reason;
  client_->OnError(reason);
}

void VideoCaptureDeviceDsLinux::OnColorImage(
    int length, const uint8* yuy2) {
  if (capture_mode_ == kCaptureColor) {
    client_->OnIncomingCapturedFrame(
        yuy2, length, base::TimeTicks::Now(), 0, capture_format_);
  } else if (capture_mode_ == kCaptureRGBD) {
    uint8* yuy2_data = yuy2_image_.get();
    memcpy(yuy2_data, yuy2, sizeof(uint8) * length);
  } 
}

void VideoCaptureDeviceDsLinux::OnDepthImage(
    int length, const int16* depth_data, const DepthSense::UV* uv) {
  int rgb_size = sizeof(uint8) * length * kBytesPerPixelRGB32;
  uint8* rgb_data = rgb32_image_.get();
  memset(rgb_data, 0, rgb_size);

  if (capture_mode_ == kCaptureDepth) {
    if (depth_encoding_ == kGrayscaleRGB32) {
      DepthToGrayscaleRGB32(depth_data, rgb_data, length);
    } else if (depth_encoding_ == kRawRGB32) {
      DepthToRawRGB32(depth_data, rgb_data, length);
    } else if (depth_encoding_ == kAdaptiveRGB32) {
      DepthToAdaptiveRGB32(depth_data, rgb_data, length);
    }
    client_->OnIncomingCapturedFrame(
        rgb_data, rgb_size, base::TimeTicks::Now(), 0, capture_format_);
  } else if (capture_mode_ == kCaptureRGBD) {
    int calibrated_size = sizeof(uint8) * length * kBytesPerPixelYUY2;
    uint8* calibrated_yuy2 = rgb32_image_.get();
    uint8* yuy2 = static_cast<uint8*>(yuy2_image_.get());
    for (int i = 0; i < length; i++) {
      int index = i * kBytesPerPixelYUY2;
      float uv_x = uv[i].u;
      float uv_y = uv[i].v;
      int color_x = (int)(uv_x * kWidth + 0.5f);
      int color_y = (int)(uv_y * kHeight + 0.5f);
      if (color_x < 0 || color_x > kWidth ||
          color_y < 0 || color_y > kHeight) {
        // set background as black.
        calibrated_yuy2[index + 0] = 0;
        calibrated_yuy2[index + 1] = 128;
        continue;
      }
      int yuy2_index =
        (color_x + color_y * kWidth) * kBytesPerPixelYUY2;
      calibrated_yuy2[index + 0] = yuy2[yuy2_index + 0];
      calibrated_yuy2[index + 1] = yuy2[yuy2_index + 1];
    }
    client_->OnIncomingCapturedFrame(
        calibrated_yuy2, calibrated_size,
         base::TimeTicks::Now(), 0, capture_format_);
  }
}

}  // namespace media

