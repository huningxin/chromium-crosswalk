// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/video/capture/win/video_capture_device_nui_win.h"

#include "base/bind.h"
#include "base/command_line.h"
#include "base/logging.h"
#include "base/strings/utf_string_conversions.h"
#include "media/base/media_switches.h"


namespace media {

static const char kKinectColorDeviceName[] = "KinectColorStream";
static const char kKinectDepthDeviceName[] = "KinectDepthStream";

// This needs to be aligned with definition in
// content/renderer/media/media_stream_impl.cc
const char kVirtualDepthDeviceId[] = "depth";
const char kVirtualRgbdDeviceId[] = "rgbd";

static const size_t kBytesPerPixelRGB32 = 4;
static const int kWidth = 640;
static const int kHeight = 480;
static const int kFrameRate = 30;

static const int kMinDepthValue =
    NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
static const int kMaxDepthValue =
    NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

// Depth encoding in RGB32
const char kDepthEncodingGrayscale[] = "grayscale";
const char kDepthEncodingRaw[] = "raw";
const char kDepthEncodingAdaptive[] = "adaptive";

static void DepthToGrayscaleRGB32(
    const uint16* depth_pixel, uint8* rgb, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    uint16 depth_value = (depth_pixel[i] & 0xFFF8) >> 3;
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

static void DepthToRawRGB32(
    const uint16* depth_pixel, uint8* rgb, unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    uint16 d = (depth_pixel[i] & 0xFFF8) >> 3;

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
    const uint16* depth_pixel, uint8* rgb, unsigned int length) {
  const double np = 512.0;
  const double w = kMaxDepthValue;
  double p = np / w;
  for (unsigned int i = 0; i < length; i++) {
    uint16 d = (depth_pixel[i] & 0xFFF8) >> 3;

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

class NuiCaptureHelper {
 public:
  NuiCaptureHelper():
      nui_sensor_(NULL),
      nui_color_stream_handle_(INVALID_HANDLE_VALUE),
      nui_color_nextframe_event_(INVALID_HANDLE_VALUE),
      nui_depth_stream_handle_(INVALID_HANDLE_VALUE),
      nui_depth_nextframe_event_(INVALID_HANDLE_VALUE),
      is_capturing_(false) {
    rgb32_image_.reset(new uint8[kWidth * kHeight * kBytesPerPixelRGB32]);
    color_coordinates_.reset(new LONG[kWidth * kHeight * 2]);

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
  }
  ~NuiCaptureHelper() {}

  bool PlatformSupported() {
    int sensor_count = 0;
    HRESULT result = NuiGetSensorCount(&sensor_count);
    if (FAILED(result))
      return false;
    if (sensor_count <= 0)
      return false;
    return true;
  }

  void AllocateAndStart(VideoCaptureDeviceNuiWin* device) {
    if (nui_capture_thread_.get() == NULL) {
      nui_capture_thread_.reset(new base::Thread("NuiCaptureThread"));
      nui_capture_thread_->Start();
    }
  
    nui_capture_thread_->message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&NuiCaptureHelper::OnAllocateAndStart,
                   base::Unretained(this),
                   device));
  }

  void StopAndDeAllocate(VideoCaptureDeviceNuiWin* device) {
    loop_for_stop_thread_ = base::MessageLoop::current();
    nui_capture_thread_->message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&NuiCaptureHelper::OnStopAndDeAllocate,
                   base::Unretained(this),
                   device));
  }

  static NuiCaptureHelper* getInstance() {
    if (!instance_)
      instance_ = new NuiCaptureHelper;
    return instance_;
  }

 private:
  bool InitSensor() {
    if (nui_sensor_ != NULL)
      return true;

    int sensor_count = 0;
    HRESULT result = NuiGetSensorCount(&sensor_count);
    if (FAILED(result)) {
      return false;
    }

    if (sensor_count <= 0)
      return false;

    INuiSensor *nui_sensor;
    for (int i = 0; i < sensor_count; ++i) {
      result = NuiCreateSensorByIndex(i, &nui_sensor);
      if (FAILED(result)) {
        continue;
      }

      result = nui_sensor->NuiStatus();
      if (S_OK == result) {
        nui_sensor_ = nui_sensor;
        break;
      }

      nui_sensor->Release();
    }

    if (nui_sensor_ == NULL)
      return false;

    // One process can initialize one Nui Sensor.
    // So we need to initialize the sensor with both color and depth modes.
    DWORD flags = NUI_INITIALIZE_FLAG_USES_COLOR |
        NUI_INITIALIZE_FLAG_USES_DEPTH;
    result = nui_sensor_->NuiInitialize(flags);
    if (FAILED(result)) {
      return false;
    }
    return true;
  }

  void SetErrorState(const std::string& reason) {
    std::vector<VideoCaptureDeviceNuiWin*>::iterator itr;
    for (itr = color_devices_.begin(); itr != color_devices_.end(); itr++)
      (*itr)->OnSetErrorState(reason);
    for (itr = depth_devices_.begin(); itr != depth_devices_.end(); itr++)
      (*itr)->OnSetErrorState(reason);
    for (itr = rgbd_devices_.begin(); itr != rgbd_devices_.end(); itr++)
      (*itr)->OnSetErrorState(reason);
  }

  void OnStopCaptureThread() {
    nui_capture_thread_->Stop();
    nui_capture_thread_.reset();
  }
  
  void OnStopAndDeAllocate(VideoCaptureDeviceNuiWin* device) {
    DCHECK_EQ(nui_capture_thread_->message_loop(), base::MessageLoop::current());

    std::vector<VideoCaptureDeviceNuiWin*>::iterator iter;
    if (device->capture_mode_ == VideoCaptureDeviceNuiWin::kCaptureRGBA) {
      for(iter = color_devices_.begin(); iter != color_devices_.end(); ++iter) {
        if( *iter == device ) {
          color_devices_.erase(iter);
          break;
        }
      }
    } else if (device->capture_mode_ ==
               VideoCaptureDeviceNuiWin::kCaptureDepth) {
      for(iter = depth_devices_.begin(); iter != depth_devices_.end(); ++iter) {
        if( *iter == device ) {
          depth_devices_.erase(iter);
          break;
        }
      }
    } else if (device->capture_mode_ ==
               VideoCaptureDeviceNuiWin::kCaptureRGBD) {
      for(iter = rgbd_devices_.begin(); iter != rgbd_devices_.end(); ++iter) {
        if( *iter == device ) {
          rgbd_devices_.erase(iter);
          break;
        }
      }
    }

    device->stop_event_.Signal();
    
    if (color_devices_.size() == 0 && rgbd_devices_.size() == 0) {
      CloseHandle(nui_color_nextframe_event_);
      nui_color_nextframe_event_ = INVALID_HANDLE_VALUE;
      nui_color_stream_handle_ = INVALID_HANDLE_VALUE;
    }
    
    if (depth_devices_.size() == 0 && rgbd_devices_.size() == 0) {
      CloseHandle(nui_depth_nextframe_event_);
      nui_depth_nextframe_event_ = INVALID_HANDLE_VALUE;
      nui_depth_stream_handle_ = INVALID_HANDLE_VALUE;
    }

    if (nui_color_stream_handle_ == INVALID_HANDLE_VALUE &&
        nui_depth_stream_handle_ == INVALID_HANDLE_VALUE) {
      is_capturing_ = false;
      nui_sensor_->NuiShutdown();
      nui_sensor_->Release();
      nui_sensor_ = NULL;

      // Time to stop capture thread.
      loop_for_stop_thread_->PostTask(
          FROM_HERE,
          base::Bind(&NuiCaptureHelper::OnStopCaptureThread,
                     base::Unretained(this)));
    }
  }

  void OnAllocateAndStart(VideoCaptureDeviceNuiWin* device) {
    DCHECK_EQ(nui_capture_thread_->message_loop(), base::MessageLoop::current());

    if (!InitSensor()) {
      SetErrorState("NUI sensor is not initialized.");
    }

    bool request_color = false;
    bool request_depth = false;

    if (device->capture_mode_ == VideoCaptureDeviceNuiWin::kCaptureRGBA) {
      color_devices_.push_back(device);
    } else if (device->capture_mode_ ==
               VideoCaptureDeviceNuiWin::kCaptureDepth) {
      depth_devices_.push_back(device);
    } else if (device->capture_mode_ ==
               VideoCaptureDeviceNuiWin::kCaptureRGBD) {
      rgbd_devices_.push_back(device);
    }

    if (color_devices_.size() >= 1) {
      request_color = true;
    }
    if (depth_devices_.size() >= 1) {
      request_depth = true;
    } 
    if (rgbd_devices_.size() >= 1) {
      request_color = true;
      request_depth = true;
    }

    HRESULT result;
    if (nui_color_stream_handle_ == INVALID_HANDLE_VALUE && request_color) {
      nui_color_nextframe_event_ = CreateEvent(NULL, TRUE, FALSE, NULL);
      result = nui_sensor_->NuiImageStreamOpen(
          NUI_IMAGE_TYPE_COLOR,
          NUI_IMAGE_RESOLUTION_640x480,
          0,
          2,
          nui_color_nextframe_event_,
          &nui_color_stream_handle_);      
      if (FAILED(result)) {
        SetErrorState("Cannot open color stream.");
        return;
      }
    }

    if (nui_depth_stream_handle_ == INVALID_HANDLE_VALUE && request_depth) {
      nui_depth_nextframe_event_ = CreateEvent(NULL, TRUE, FALSE, NULL);
      result = nui_sensor_->NuiImageStreamOpen(
          NUI_IMAGE_TYPE_DEPTH,
          NUI_IMAGE_RESOLUTION_640x480,
          0,
          2,
          nui_depth_nextframe_event_,
          &nui_depth_stream_handle_);

      if (FAILED(result)) {
        SetErrorState("Cannot open depth stream.");
        return;
      }

      // Set near mode of Kinect depth sensor.
      nui_sensor_->NuiImageStreamSetImageFrameFlags(
          nui_depth_stream_handle_, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
    }

    if (!is_capturing_) {
      is_capturing_ = true;
      nui_capture_thread_->message_loop()->PostTask(
          FROM_HERE,
          base::Bind(&NuiCaptureHelper::OnCaptureTask,
                     base::Unretained(this)));
    }
    return;
  }

  void OnCaptureTask() {
    DCHECK_EQ(nui_capture_thread_->message_loop(), base::MessageLoop::current());

    if (is_capturing_ != true)
      return;

    DWORD return_value;
    HRESULT  hresult;
    NUI_IMAGE_FRAME nui_color_image_frame;
    NUI_IMAGE_FRAME nui_depth_image_frame;
    INuiFrameTexture *nui_color_frame_texture;
    INuiFrameTexture *nui_depth_frame_texture;

    NUI_LOCKED_RECT nui_color_lock_rect;
    NUI_LOCKED_RECT nui_depth_lock_rect;

    if (nui_color_stream_handle_ != INVALID_HANDLE_VALUE) {
      return_value = WaitForSingleObject(nui_color_nextframe_event_, INFINITE); 
      if (return_value != WAIT_OBJECT_0) {
        SetErrorState("next color frame event error");
        return;
      }
      hresult = nui_sensor_->NuiImageStreamGetNextFrame(
          nui_color_stream_handle_, 0, &nui_color_image_frame);
      if (FAILED(hresult)) {
        SetErrorState("get color image frame error");
        return;
      }
      nui_color_frame_texture = nui_color_image_frame.pFrameTexture;
      nui_color_frame_texture->LockRect(0, &nui_color_lock_rect, NULL, 0);
    }

    if (nui_depth_stream_handle_ != INVALID_HANDLE_VALUE) {
      return_value = WaitForSingleObject(nui_depth_nextframe_event_, INFINITE); 
      if (return_value != WAIT_OBJECT_0) {
        SetErrorState("next depth frame event error");
        return;
      }
      hresult = nui_sensor_->NuiImageStreamGetNextFrame(
          nui_depth_stream_handle_, 0, &nui_depth_image_frame);
      if (FAILED(hresult)) {
        SetErrorState("get depth image frame error");
        return;
      }
      nui_depth_frame_texture = nui_depth_image_frame.pFrameTexture;
      nui_depth_frame_texture->LockRect(0, &nui_depth_lock_rect, NULL, 0);
    }

    unsigned int length = kWidth * kHeight;
    if (color_devices_.size() != 0) {    
      if (nui_color_lock_rect.Pitch != 0) {
        CaptureColorImage(
            length, static_cast<uint8*>(nui_color_lock_rect.pBits));
      }
    }

    if (depth_devices_.size() != 0) {
      if (nui_depth_lock_rect.Pitch != 0) {
        CaptureDepthImage(
            length, static_cast<uint8*>(nui_depth_lock_rect.pBits));
      }
    }

    if (rgbd_devices_.size() != 0) {
      if ((nui_color_lock_rect.Pitch != 0) && (nui_depth_lock_rect.Pitch != 0)) {
        CaptureRgbdImage(length, static_cast<uint8*> (nui_color_lock_rect.pBits),
                         static_cast<uint8*> (nui_depth_lock_rect.pBits));
      }
    }

    if (nui_color_stream_handle_ != INVALID_HANDLE_VALUE) {
      nui_color_frame_texture->UnlockRect(0);
      nui_sensor_->NuiImageStreamReleaseFrame(
          nui_color_stream_handle_, &nui_color_image_frame);
    }

    if (nui_depth_stream_handle_ != INVALID_HANDLE_VALUE) {
      nui_depth_frame_texture->UnlockRect(0);
      nui_sensor_->NuiImageStreamReleaseFrame(
          nui_depth_stream_handle_, &nui_depth_image_frame);
    }

    nui_capture_thread_->message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&NuiCaptureHelper::OnCaptureTask,
                 base::Unretained(this)));
  }

  void CaptureColorImage(unsigned length, const uint8* bits) {
    int byte_length = length * kBytesPerPixelRGB32;
    std::vector<VideoCaptureDeviceNuiWin*>::iterator itr;
    for (itr = color_devices_.begin(); itr != color_devices_.end(); itr++)
      (*itr)->OnIncomingCapturedFrame(bits, length);
  }

  void CaptureDepthImage(unsigned length, const uint8* bits) {
    uint8* rgb_data = rgb32_image_.get();
    memset(rgb_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
    const uint16 *depth_pixel = reinterpret_cast<const uint16 *>(bits);

    if (depth_encoding_ == kGrayscaleRGB32) {
      DepthToGrayscaleRGB32(depth_pixel, rgb_data, length);
    } else if (depth_encoding_ == kRawRGB32) {
      DepthToRawRGB32(depth_pixel, rgb_data, length);
    } else if (depth_encoding_ == kAdaptiveRGB32) {
      DepthToAdaptiveRGB32(depth_pixel, rgb_data, length);
    }
    std::vector<VideoCaptureDeviceNuiWin*>::iterator itr;
    for (itr = depth_devices_.begin(); itr != depth_devices_.end(); itr++)
      (*itr)->OnIncomingCapturedFrame(rgb_data, length);
  }

  void CaptureRgbdImage(
      unsigned int length, const uint8* color_bits, const uint8* depth_bits) {
    uint8* rgbd_data = rgb32_image_.get();
    memset(rgbd_data, 0, sizeof(uint8) * length * kBytesPerPixelRGB32);
    LONG* color_coordinates = color_coordinates_.get();
    USHORT *depth_pixel =
        const_cast<USHORT*>(reinterpret_cast<const USHORT*>(depth_bits));
    nui_sensor_->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
        NUI_IMAGE_RESOLUTION_640x480,
        NUI_IMAGE_RESOLUTION_640x480,
        length,
        depth_pixel,
        length * 2,
        color_coordinates);
    for (unsigned int i = 0; i < length; i++) {
      int rgbd_index = i * kBytesPerPixelRGB32;
      int color_x = color_coordinates[i * 2 + 0];
      int color_y = color_coordinates[i * 2 + 1];
      if (color_x < 0 || color_x > static_cast<int>(kWidth) ||
          color_y < 0 || color_y > static_cast<int>(kHeight))
        continue;
      int color_index =
          (color_x + color_y * kWidth) * kBytesPerPixelRGB32;
      rgbd_data[rgbd_index + 0] = color_bits[color_index + 0];
      rgbd_data[rgbd_index + 1] = color_bits[color_index + 1];
      rgbd_data[rgbd_index + 2] = color_bits[color_index + 2];
    }
    std::vector<VideoCaptureDeviceNuiWin*>::iterator itr;
    for (itr = rgbd_devices_.begin(); itr != rgbd_devices_.end(); itr++)
      (*itr)->OnIncomingCapturedFrame(rgbd_data, length);
  }

 private:
  enum DepthEncoding {
    // Convert 16-bit depth value into 1 8-bit channel.
    kGrayscaleRGB32,
    // Map 16-bit depth value into 3 8-bit channels.
    kRawRGB32,
    // Implement the ecoding scheme proposed in paper
    // "Adapting Standard Video Codecs for Depth Streaming"
    kAdaptiveRGB32
  };

  DepthEncoding depth_encoding_;
  INuiSensor *nui_sensor_;
  HANDLE nui_color_stream_handle_;
  HANDLE nui_color_nextframe_event_;
  HANDLE nui_depth_stream_handle_;
  HANDLE nui_depth_nextframe_event_;

  scoped_ptr<base::Thread> nui_capture_thread_;

  std::vector<VideoCaptureDeviceNuiWin*> color_devices_;
  std::vector<VideoCaptureDeviceNuiWin*> depth_devices_;
  std::vector<VideoCaptureDeviceNuiWin*> rgbd_devices_;
  base::Lock lock_;

  scoped_ptr<uint8[]> rgb32_image_;
  scoped_ptr<LONG[]> color_coordinates_;

  bool is_capturing_;
  base::MessageLoop* loop_for_stop_thread_;

  static NuiCaptureHelper* instance_;
};

NuiCaptureHelper* NuiCaptureHelper::instance_ = NULL;

bool VideoCaptureDeviceNuiWin::PlatformSupported() {
  return NuiCaptureHelper::getInstance()->PlatformSupported();
}

void VideoCaptureDeviceNuiWin::GetDeviceNames(Names* device_names) {
  device_names->clear();

  if (!PlatformSupported())
    return;

  Name color_device_name(kKinectColorDeviceName,
                         kKinectColorDeviceName,
                         Name::NUI_CAPTURE);
  DLOG(INFO) << "Video capture device, " << color_device_name.name()
             << " : " << color_device_name.id();
  device_names->push_back(color_device_name);

  Name depth_device_name(kKinectDepthDeviceName,
                         kVirtualDepthDeviceId,
                         Name::NUI_CAPTURE);
  DLOG(INFO) << "Depth capture device, " << depth_device_name.name()
             << " : " << depth_device_name.id();
  device_names->push_back(depth_device_name);

  Name rgbd_device_name(kKinectColorDeviceName,
                        kVirtualRgbdDeviceId,
                        Name::NUI_CAPTURE);
  DLOG(INFO) << "RGBD capture device, " << rgbd_device_name.name()
             << " : " << rgbd_device_name.id();
  device_names->push_back(rgbd_device_name);
}

VideoCaptureDeviceNuiWin::VideoCaptureDeviceNuiWin(const Name& device_name)
    : device_name_(device_name),
      stop_event_(false, false) {
  DLOG(INFO) << device_name.name() << ": " << device_name.id();
  if (device_name.id() == kVirtualDepthDeviceId) {
    capture_mode_ = kCaptureDepth;
  } else if (device_name.id() == kVirtualRgbdDeviceId) {
    capture_mode_ = kCaptureRGBD;
  } else {
    capture_mode_ = kCaptureRGBA;
  }

  // Store our current width and height.
  capture_format_.frame_size.SetSize(kWidth, kHeight);
  capture_format_.frame_rate = kFrameRate;
  capture_format_.pixel_format = PIXEL_FORMAT_ARGB;
}

VideoCaptureDeviceNuiWin::~VideoCaptureDeviceNuiWin() {
}

bool VideoCaptureDeviceNuiWin::Init() {
  return true;
}

void VideoCaptureDeviceNuiWin::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<Client> client) {
  client_ = client.Pass();
  NuiCaptureHelper::getInstance()->AllocateAndStart(this);
}

void VideoCaptureDeviceNuiWin::StopAndDeAllocate() {
  NuiCaptureHelper::getInstance()->StopAndDeAllocate(this);
  stop_event_.Wait();
  client_.reset();
}

void VideoCaptureDeviceNuiWin::OnIncomingCapturedFrame(
    const uint8* bits, int length) {
  client_->OnIncomingCapturedFrame(
      bits, length, base::TimeTicks::Now(), 0, capture_format_);
}

void VideoCaptureDeviceNuiWin::OnSetErrorState(const std::string& reason) {
  DVLOG(1) << reason;
  client_->OnError(reason);
}

}  // namespace media

