// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_pxc_win.h"

#include "base/bind.h"
#include "base/command_line.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"
#include "media/base/media_switches.h"

namespace media {

static const size_t kBytesPerPixelRGB32 = 4;

const char kPxcColorCameraDeviceName[] = "pxc_color_camera";
const char kPxcColorCameraDeviceId[] = "color_camera_device_id";
const char kPxcDepthCameraDeviceName[] = "pxc_depth_camera";
const char kPxcDepthCameraDeviceId[] = "depth_camera_device_id";
const char kPxcAlignedDepthCameraDeviceName[] = "pxc_aligned_depth_camera";
const char kPxcAlignedDepthCameraDeviceId[] = "aligned_depth_camera_device_id";

// Capture type.
const char kPxcCaptureColorOption[] = "color";
const char kPxcCaptureDepthOption[] = "depth";
const char kPxcCaptureAlignedDepthOption[] = "aligned-depth";

// Depth encoding in RGB32.
const char kDepthEncodingGrayscale[] = "grayscale";
const char kDepthEncodingRaw[] = "raw";
const char kDepthEncodingAdaptive[] = "adaptive";

// TODO(nhu): the min and max depth value should be obtained from RSSDK API.
static const int kMinDepthValue = 100;
static const int kMaxDepthValue = 2047;

// Release a PXCSession will call CoUninitiliaze which causes calling thread
// (Audio Thread) to fail on COM API calling. Hold a global PXCSession to avoid
// this.
PXCSession* g_session = NULL;

bool VideoCaptureDevicePxcWin::PlatformSupported() {
  PXCSession *session = PXCSession::CreateInstance();
  if (!session) {
    DLOG(ERROR) << "Failed to create a PXC Session.";
    return false;
  }
  session->Release();
  return true;
}

void VideoCaptureDevicePxcWin::GetDeviceNames(Names* device_names) {
  device_names->clear();

  if (!VideoCaptureDevicePxcWin::PlatformSupported())
    return;

  Name color_camera(kPxcColorCameraDeviceName,
                    kPxcColorCameraDeviceId,
                    Name::PXC_CAPTURE);
  DLOG(INFO) << "Video capture device, " << color_camera.name()
             << " : " << color_camera.id();
  device_names->push_back(color_camera);

  Name depth_camera(kPxcDepthCameraDeviceName,
                    kPxcDepthCameraDeviceId,
                    Name::PXC_CAPTURE);
  DLOG(INFO) << "Depth capture device, " << depth_camera.name()
             << " : " << depth_camera.id();
  device_names->push_back(depth_camera);

  Name aligned_depth_camera(kPxcAlignedDepthCameraDeviceName,
                            kPxcAlignedDepthCameraDeviceId,
                            Name::PXC_CAPTURE);
  DLOG(INFO) << "Aligned depth capture device, " << aligned_depth_camera.name()
             << " : " << aligned_depth_camera.id();
  device_names->push_back(aligned_depth_camera);
}

VideoCaptureDevicePxcWin::VideoCaptureDevicePxcWin(const Name& device_name)
    : state_(kIdle),
      device_name_(device_name),
      pxc_sense_manager_(NULL),
      pxc_projection_(NULL),
      capture_type_(kCaptureColor),
      depth_encoding_(kGrayscaleRGB32) {
  const base::CommandLine* cmd_line = base::CommandLine::ForCurrentProcess();
  std::string type_option(
      cmd_line->GetSwitchValueASCII(switches::kPxcCaptureType));

  if (!type_option.empty()) {
    if (type_option == kPxcCaptureColorOption) {
      capture_type_ = kCaptureColor;
    } else if (type_option == kPxcCaptureDepthOption) {
      capture_type_ = kCaptureDepth;
    } else if (type_option == kPxcCaptureAlignedDepthOption) {
      capture_type_ = kCaptureAlignedDepth;
    }
  } else {
    if (device_name_.id() == kPxcColorCameraDeviceId) {
      capture_type_ = kCaptureColor;
    } else if (device_name_.id() == kPxcDepthCameraDeviceId) {
      capture_type_ = kCaptureDepth;
    } else if (device_name_.id() == kPxcAlignedDepthCameraDeviceId) {
      capture_type_ = kCaptureAlignedDepth;
    }
  }

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
}

VideoCaptureDevicePxcWin::~VideoCaptureDevicePxcWin() {
}

bool VideoCaptureDevicePxcWin::Init() {
  return true;
}

void VideoCaptureDevicePxcWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  DCHECK(CalledOnValidThread());
  if (state_ != kIdle)
    return;

  client_ = client.Pass();

  pxc_sense_manager_ = PXCSenseManager::CreateInstance();
  if (!pxc_sense_manager_) {
    SetErrorState("Can't create pxc sense manager.");
    return;
  }

  PXCVideoModule::DataDesc ddesc = {};
  if (capture_type_ == kCaptureColor || capture_type_ == kCaptureDepth) {
    PXCCapture::StreamType stream_type =
        capture_type_ == kCaptureColor ? 
            PXCCapture::STREAM_TYPE_COLOR : PXCCapture::STREAM_TYPE_DEPTH;
    ddesc.deviceInfo.streams = stream_type;
    PXCVideoModule::StreamDesc& sdesc = ddesc.streams[stream_type];
    sdesc.sizeMin.width = sdesc.sizeMax.width =
        params.requested_format.frame_size.width();
    sdesc.sizeMin.height = sdesc.sizeMax.height =
        params.requested_format.frame_size.height();
    sdesc.frameRate.min = sdesc.frameRate.max =
        params.requested_format.frame_rate;
  } else if (capture_type_ == kCaptureAlignedDepth) {
    ddesc.deviceInfo.streams =
        PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
    PXCVideoModule::StreamDesc& sdesc_color =
        ddesc.streams[PXCCapture::STREAM_TYPE_COLOR];
    PXCVideoModule::StreamDesc& sdesc_depth =
        ddesc.streams[PXCCapture::STREAM_TYPE_DEPTH];
    sdesc_color.sizeMin.width = sdesc_color.sizeMax.width =
        sdesc_depth.sizeMin.width = sdesc_depth.sizeMax.width =
            params.requested_format.frame_size.width();
    sdesc_color.sizeMin.height = sdesc_color.sizeMax.height =
        sdesc_depth.sizeMin.height = sdesc_depth.sizeMax.height =
            params.requested_format.frame_size.height();
    sdesc_color.frameRate.min = sdesc_color.frameRate.max =
        sdesc_depth.frameRate.min = sdesc_depth.frameRate.max =
            params.requested_format.frame_rate;
  }

  pxcStatus status = pxc_sense_manager_->EnableStreams(&ddesc);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't enable streams.");
    return;
  }

  status = pxc_sense_manager_->Init(this);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't init pxc sense manager.");
    return;
  }

  status = pxc_sense_manager_->StreamFrames(false);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't stream frames.");
    return;
  }

  PXCCaptureManager* pxc_capture_manager = 
      pxc_sense_manager_->QueryCaptureManager();
  if (!pxc_capture_manager) {
    SetErrorState("Can't query capture manager.");
    return;
  }

  PXCCapture::Device* pxc_device = pxc_capture_manager->QueryDevice();
  if (!pxc_device) {
    SetErrorState("Can't query capture device.");
    return;
  }

  PXCCapture::Device::StreamProfileSet profiles = {};
  status = pxc_device->QueryStreamProfileSet(&profiles);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Can't query stream profiles.");
    return;
  }
  
  PXCCapture::Device::StreamProfile profile = profiles.color;

  if (capture_type_ == kCaptureDepth) {
    profile = profiles.depth;
  }

  if (!profile.imageInfo.format) {
    SetErrorState("Invalid image format.");
    return;
  }

  capture_format_.frame_size.SetSize(profile.imageInfo.width,
                                     profile.imageInfo.height);
  capture_format_.frame_rate = profile.frameRate.max;
  capture_format_.pixel_format = PIXEL_FORMAT_ARGB;

  if (capture_type_ == kCaptureDepth || capture_type_ == kCaptureAlignedDepth) {
    depth_argb_image_.reset(
        new uint8[profile.imageInfo.width * profile.imageInfo.height *
                  kBytesPerPixelRGB32]);
  }

  if (capture_type_ == kCaptureAlignedDepth) {
    pxc_projection_ = pxc_device->CreateProjection();
    if (!pxc_projection_) {
      SetErrorState("Can't create pxc projection.");
      return;
    }
  }

  depth_low_confidence_value_ = pxc_device->QueryDepthLowConfidenceValue();
  depth_range_ = pxc_device->QueryDepthSensorRange();

  DLOG(INFO) << "depth_low_confidence_value_ = " << depth_low_confidence_value_
      << " depth_range_.min = " << depth_range_.min
      << " depth_range_.max = " << depth_range_.max;

  state_ = kCapturing;
}

void VideoCaptureDevicePxcWin::StopAndDeAllocate() {
  DCHECK(CalledOnValidThread());
  if (state_ != kCapturing)
    return;

  pxc_sense_manager_->Close();
  pxc_sense_manager_->Release();
  pxc_sense_manager_ = NULL;

  if (pxc_projection_) {
    pxc_projection_->Release();
    pxc_projection_ = NULL;
  }

  client_.reset();
  depth_argb_image_.reset();
  state_ = kIdle;
}

void VideoCaptureDevicePxcWin::CaptureColor(PXCCapture::Sample* sample) {
  PXCImage* image = sample->color;
  if (image) {
    PXCImage::ImageInfo info = image->QueryInfo();

    PXCImage::ImageData data;
    pxcStatus status = image->AcquireAccess(
        PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &data);
    if (status < PXC_STATUS_NO_ERROR) {
      SetErrorState("Access image data error");
      return;
    }

    int length = info.width * info.height * kBytesPerPixelRGB32;
    client_->OnIncomingCapturedData(
        static_cast<uint8*> (data.planes[0]),
        length, capture_format_, 0, base::TimeTicks::Now());

    image->ReleaseAccess(&data);
  }
}

void VideoCaptureDevicePxcWin::CaptureDepth(PXCCapture::Sample* sample) {
  PXCImage* depth = sample->depth;
  if (depth) {
    CaptureDepthCommon(depth);
  }
}

void VideoCaptureDevicePxcWin::CaptureAlignedDepth(PXCCapture::Sample* sample) {
  if (sample->color && sample->depth) {
    PXCImage* aligned_depth =
        pxc_projection_->CreateDepthImageMappedToColor(
            sample->depth, sample->color);
    if (!aligned_depth) {
      SetErrorState("Can't create depth image mapped to color.");
      return;
    }
    CaptureDepthCommon(aligned_depth);
    aligned_depth->Release();
  }
}

void VideoCaptureDevicePxcWin::CaptureDepthCommon(PXCImage* image) {
  PXCImage::ImageInfo info = image->QueryInfo();

  PXCImage::ImageData data;
  pxcStatus status = image->AcquireAccess(
      PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data);
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("Access image data error");
    return;
  }

  int length = info.width * info.height;

  uint8* argb_data = depth_argb_image_.get();
  memset(argb_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
  uint16* depth_data = reinterpret_cast<uint16*>(data.planes[0]);

  for (int i = 0; i < length; ++i) {
    uint16 depth = depth_data[i];
    uint8* pixel = argb_data + i * kBytesPerPixelRGB32;

    if (depth == depth_low_confidence_value_)
      continue;

    if (depth_encoding_ == kGrayscaleRGB32) {
      EncodeDepthToGrayscaleRGB32(pixel, depth);
    } else if (depth_encoding_ == kRawRGB32) {
      EncodeDepthToRawRGB32(pixel, depth);
    } else if (depth_encoding_ == kAdaptiveRGB32) {
      EncodeDepthToAdaptiveRGB32(pixel, depth);
    }

#if 0
    // Layout is:
    // Bitnumber 31...24 23...16 15...8 7...0
    // Component A       R       G      B
    // R for depth value low
    uint8 depth_value_low = static_cast<uint8>(depth_value & 0xFF);
    argb_data[i * kBytesPerPixelRGB32 + 2] = depth_value_low;
    // G for depth value high
    uint8 depth_value_high = static_cast<uint8>((depth_value >> 8) & 0xFF);
    argb_data[i * kBytesPerPixelRGB32 + 1] = depth_value_high;
#endif
  }

  client_->OnIncomingCapturedData(
      argb_data,
      length * kBytesPerPixelRGB32,
      capture_format_, 0, base::TimeTicks::Now());

  image->ReleaseAccess(&data);
}

void VideoCaptureDevicePxcWin::EncodeDepthToGrayscaleRGB32(
    uint8* argb, uint16 depth) {
  // Implement the algorithm as equation (4) in paper
  // "3-D Video Representation Using Depth Maps".
  float value = 255.0 *
      ((1.0 / depth -
        1.0 / kMaxDepthValue) /
       (1.0 / kMinDepthValue  -
        1.0 / kMaxDepthValue));
  argb[0] = static_cast<uint8>(value);
  argb[1] = static_cast<uint8>(value);
  argb[2] = static_cast<uint8>(value);
}

void VideoCaptureDevicePxcWin::EncodeDepthToRawRGB32(
    uint8* argb, uint16 depth) {
  // Implement the BIT2 scheme in paper
  // "Adapting Standard Video Codecs for Depth Streaming"
  // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
  argb[0] = static_cast<uint8>(((depth & 0xFC00) >> 8) & 0xFF);
  argb[1] = static_cast<uint8>(((depth & 0x03E0) >> 2) & 0xFF);
  argb[2] = static_cast<uint8>(((depth & 0x001F) << 3) & 0xFF);
}

void VideoCaptureDevicePxcWin::EncodeDepthToAdaptiveRGB32(
    uint8* argb, uint16 depth) {
    const double np = 512.0;
    const double w = kMaxDepthValue;
    double p = np / w;

    if (depth > w) {
      return;
    }

    // Implement the depth ecoding schema described in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    // (http://web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf)
    double ld = (depth + 0.5) / w;

    double ha = fmod(ld / (p / 2.0), 2.0);
    if (ha > 1.0)
      ha = 2.0 - ha;

    double hb = fmod((ld - p / 4.0) / (p / 2.0), 2.0);
    if (hb > 1.0)
      hb = 2.0 - hb;

    // Layout is BGRA.
    argb[0] = static_cast<uint8>(255.0 * ld);
    argb[1] = static_cast<uint8>(255.0 * ha);
    argb[2] = static_cast<uint8>(255.0 * hb);
}

pxcStatus VideoCaptureDevicePxcWin::OnNewSample(
    pxcUID mid, PXCCapture::Sample* sample) {
  if (capture_type_ == kCaptureColor) {
    CaptureColor(sample);
  } else if (capture_type_ == kCaptureDepth) {
    CaptureDepth(sample);
  } else if (capture_type_ == kCaptureAlignedDepth) {
    CaptureAlignedDepth(sample);
  } else {
    NOTREACHED();
  }
  return PXC_STATUS_NO_ERROR;
}

void VideoCaptureDevicePxcWin::OnStatus(pxcUID mid, pxcStatus status) {
  if (status < PXC_STATUS_NO_ERROR) {
    SetErrorState("On error status.");
    return;
  }
}

void VideoCaptureDevicePxcWin::SetErrorState(const std::string& reason) {
  DCHECK(CalledOnValidThread());
  DVLOG(1) << reason;
  state_ = kError;
  client_->OnError(reason);
}

}  // namespace media

