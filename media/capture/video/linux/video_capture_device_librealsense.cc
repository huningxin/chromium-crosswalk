// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/capture/video/linux/video_capture_device_librealsense.h"

#include <stddef.h>

#include <list>

#include "base/bind.h"
#include "base/strings/stringprintf.h"
#include "build/build_config.h"

#include <librealsense/rs.h>  // NOLINT

namespace media {

inline void make_depth_histogram(uint8_t* rgb_image, const uint16_t depth_image[], int width, int height)
{
    static uint32_t histogram[0x10000];
    memset(histogram, 0, sizeof(histogram));

    for(int i = 0; i < width*height; ++i) ++histogram[depth_image[i]];
    for(int i = 2; i < 0x10000; ++i) histogram[i] += histogram[i-1]; // Build a cumulative histogram for the indices in [1,0xFFFF]
    for(int i = 0; i < width*height; ++i)
    {
        if(uint16_t d = depth_image[i])
        {
            int f = histogram[d] * 255 / histogram[0xFFFF]; // 0-255 based on histogram location
            rgb_image[i*3 + 0] = 255 - f;
            rgb_image[i*3 + 1] = 0;
            rgb_image[i*3 + 2] = f;
        }
        else
        {
            rgb_image[i*3 + 0] = 20;
            rgb_image[i*3 + 1] = 5;
            rgb_image[i*3 + 2] = 0;
        }
    }
}

class RsCaptureDelegate final
    : public base::RefCountedThreadSafe<RsCaptureDelegate> {
 public:
  RsCaptureDelegate(
      const scoped_refptr<base::SingleThreadTaskRunner>& task_runner) :
      task_runner_(task_runner),
      is_capturing_(false) {
    rs_context_ = rs_create_context(RS_API_VERSION, nullptr);
    int device_count = rs_get_device_count(rs_context_, nullptr);
    if (!device_count) {
      LOG(ERROR) << "No device detected. Is it plugged in?";
      return;
    }

    // Assume the first device.
    rs_device_ = rs_get_device(rs_context_, 0, nullptr);
  }

  void StartStream(rs_stream stream,
                   std::unique_ptr<VideoCaptureDevice::Client> client) {
    DCHECK(task_runner_->BelongsToCurrentThread());
    if (clients_[static_cast<int>(stream)].get()) {
      LOG(ERROR) << "Stream " << stream << " is already started";
      return;
    }
    DCHECK(client);

    if (rs_is_device_streaming(rs_device_, nullptr) != 0) {
      LOG(ERROR) << "Stop device";
      rs_stop_device(rs_device_, RS_SOURCE_ALL, nullptr);
    }

    clients_[static_cast<int>(stream)] = std::move(client);

    rs_enable_stream_preset(rs_device_, stream, RS_PRESET_BEST_QUALITY, nullptr);

    int width = rs_get_stream_width(rs_device_, stream, nullptr);
    int height = rs_get_stream_height(rs_device_, stream, nullptr);
    rs_format format = rs_get_stream_format(rs_device_, stream, nullptr);

    LOG(ERROR) << "Enabled Stream: " << width << " " << height << " " << format;

    capture_formats_[static_cast<int>(stream)].frame_size.SetSize(width, height);
    capture_formats_[static_cast<int>(stream)].frame_rate = rs_get_stream_framerate(rs_device_, stream, nullptr);
    capture_formats_[static_cast<int>(stream)].pixel_format = PIXEL_FORMAT_RGB24;

    if (format == RS_FORMAT_Z16) {
      rgb_.reset(new uint8_t[width * height * 3]);
    }

    rs_start_device(rs_device_, RS_SOURCE_ALL, nullptr);

    if (is_capturing_ == false) {
      is_capturing_ = true;
      task_runner_->PostTask(
          FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
    }
  }
  void StopStream(rs_stream stream) {
    DCHECK(task_runner_->BelongsToCurrentThread());

    LOG(ERROR) << "Stop Stream: " << stream;

    if (rs_is_device_streaming(rs_device_, nullptr) != 0) {
      LOG(ERROR) << "Stop device";
      rs_stop_device(rs_device_, RS_SOURCE_ALL, nullptr);
    }

    clients_[static_cast<int>(stream)].reset();
    rs_disable_stream(rs_device_, stream, nullptr);

    if (clients_[static_cast<int>(RS_STREAM_COLOR)].get() ||
        clients_[static_cast<int>(RS_STREAM_DEPTH)].get()) {
      rs_start_device(rs_device_, RS_SOURCE_ALL, nullptr);
      LOG(ERROR) << "Restart device";
    } else {
      is_capturing_ = false;
    }
  }

 private:
  friend class base::RefCountedThreadSafe<RsCaptureDelegate>;
  ~RsCaptureDelegate() {
    rs_delete_context(rs_context_, nullptr);
  }

  void DoCapture() {
    DCHECK(task_runner_->BelongsToCurrentThread());
    if (!is_capturing_) {
      LOG(ERROR) << "Stop capture";
      return;
    }

    rs_wait_for_frames(rs_device_, nullptr);

    if (clients_[static_cast<int>(RS_STREAM_COLOR)].get()) {
      if (rs_get_stream_format(rs_device_, RS_STREAM_COLOR, nullptr) == RS_FORMAT_RGB8) {
        clients_[static_cast<int>(RS_STREAM_COLOR)]->OnIncomingCapturedData(
            reinterpret_cast<const uint8_t *>(rs_get_frame_data(rs_device_, RS_STREAM_COLOR, nullptr)),
            capture_formats_[static_cast<int>(RS_STREAM_COLOR)].ImageAllocationSize(),
            capture_formats_[static_cast<int>(RS_STREAM_COLOR)], 0, base::TimeTicks::Now());
      }
    }

    if (clients_[static_cast<int>(RS_STREAM_DEPTH)].get()) {
      if (rs_get_stream_format(rs_device_, RS_STREAM_DEPTH, nullptr) == RS_FORMAT_Z16) {
        make_depth_histogram(rgb_.get(),
                             reinterpret_cast<const uint16_t *>(rs_get_frame_data(rs_device_, RS_STREAM_DEPTH, nullptr)),
                             rs_get_stream_width(rs_device_, RS_STREAM_DEPTH, nullptr), rs_get_stream_height(rs_device_, RS_STREAM_DEPTH, nullptr));
        clients_[static_cast<int>(RS_STREAM_DEPTH)]->OnIncomingCapturedData(
            rgb_.get(),
            capture_formats_[static_cast<int>(RS_STREAM_DEPTH)].ImageAllocationSize(),
            capture_formats_[static_cast<int>(RS_STREAM_DEPTH)], 0, base::TimeTicks::Now());
      }
    }

    task_runner_->PostTask(
        FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
  }

  const scoped_refptr<base::SingleThreadTaskRunner> task_runner_;
  const VideoCaptureDevice::Name device_name_;

  // depth = 0, color = 1
  std::unique_ptr<VideoCaptureDevice::Client> clients_[2];
  VideoCaptureFormat capture_formats_[2];

  bool is_capturing_;

  rs_context* rs_context_;
  rs_device* rs_device_;

  std::unique_ptr<uint8_t[]> rgb_;

  DISALLOW_COPY_AND_ASSIGN(RsCaptureDelegate);
};

class RsCaptureService {
 public:
  static RsCaptureService* GetInstance() {
    if (g_rs_capture_service_ == nullptr) {
      g_rs_capture_service_ = new RsCaptureService;
    }
    return g_rs_capture_service_;
  }
  RsCaptureService() : 
      rs_thread_("LibrealsenseCaptureThread") {
    rs_thread_.Start();
    capture_impl_ = new RsCaptureDelegate(rs_thread_.task_runner());
  }
  ~RsCaptureService() {
    // Check if the thread is running.
    // This means that the device has not been StopAndDeAllocate()d properly.
    DCHECK(!rs_thread_.IsRunning());
    rs_thread_.Stop();
  }

  void StartStream(rs_stream stream, std::unique_ptr<VideoCaptureDevice::Client> client) {
    DCHECK(client);
    rs_thread_.message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&RsCaptureDelegate::StartStream, capture_impl_,
                   stream,
                   base::Passed(&client)));
  }

  void StopStream(rs_stream stream) {
    rs_thread_.message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&RsCaptureDelegate::StopStream, capture_impl_,
                   stream));
  }
 
 private:
  static RsCaptureService* g_rs_capture_service_;

  scoped_refptr<RsCaptureDelegate> capture_impl_;

  base::Thread rs_thread_;
};

RsCaptureService* RsCaptureService::g_rs_capture_service_ = nullptr;

void VideoCaptureDeviceLibrealsense::GetDeviceNames(Names* device_names) {
  DCHECK(device_names->empty());
  LOG(ERROR) << "VideoCaptureDeviceLibrealsense::GetDeviceNames";

  
  rs_context* ctx = rs_create_context(RS_API_VERSION, nullptr);

  int device_count = rs_get_device_count(ctx, nullptr);
  if (!device_count) LOG(ERROR) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs_device* dev = rs_get_device(ctx, i, nullptr);
    LOG(ERROR) << "Device " << i << " - " << rs_get_device_name(dev, nullptr)
        << " Serial number: " << rs_get_device_serial(dev, nullptr) << " Firmware version: "
        << rs_get_device_firmware_version(dev, nullptr);

    // Show which streams are supported by this device
    for (int j = 0; j < RS_STREAM_COUNT; ++j)
    {
        // Determine number of available streaming modes (zero means stream is unavailable) 
        rs_stream strm = (rs_stream)j;
        int mode_count = rs_get_stream_mode_count(dev, strm, nullptr);
        if(mode_count == 0) continue;

        if (!(strm == RS_STREAM_DEPTH || strm == RS_STREAM_COLOR))
          continue;

        // Show each available mode for this stream
        LOG(ERROR) << " Stream " << strm << " - " << mode_count << "\n";

        std::string name = std::string(rs_get_device_name(dev, nullptr)) + std::string("-") + std::string(rs_stream_to_string((rs_stream)strm));
        std::string id = std::string(rs_get_device_serial(dev, nullptr)) + std::string("-") + std::string(rs_stream_to_string((rs_stream)strm));
        device_names->push_back(VideoCaptureDevice::Name(
          name, id, VideoCaptureDevice::Name::LIBREALSENSE));        
    }
  }

  rs_delete_context(ctx, nullptr);
}

void VideoCaptureDeviceLibrealsense::GetDeviceSupportedFormats(
    const Name& device,
    VideoCaptureFormats* formats) {
  rs_context* ctx = rs_create_context(RS_API_VERSION, nullptr);

  std::string delimiter("-");
  std::string name = device.name().substr(0, device.name().find(delimiter));
  std::string stream = device.name().substr(device.name().find(delimiter) + 1);

  rs_stream requested_stream;
  if (stream == "COLOR") {
    requested_stream = RS_STREAM_COLOR;
  } else if (stream == "DEPTH") {
    requested_stream = RS_STREAM_DEPTH;
  } else {
    NOTREACHED();
    return;
  }

  LOG(ERROR) << "VideoCaptureDeviceLibrealsense::GetDeviceSupportedFormats: " << name << " " << stream;

  int device_count = rs_get_device_count(ctx, nullptr);
  if (!device_count) LOG(ERROR) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs_device * dev = rs_get_device(ctx, i, nullptr);
    LOG(ERROR) << "Device " << i << " - " << rs_get_device_name(dev, nullptr) << ":\n";

    if (name != std::string(rs_get_device_name(dev, nullptr)))
      continue;

    int mode_count = rs_get_stream_mode_count(dev, requested_stream, nullptr);
    if (mode_count == 0) {
      LOG(ERROR) << "No modes found";
    }

    for(int k = 0; k < mode_count; ++k){
      // Show width, height, format, and framerate, the settings required to enable the stream in this mode
      int width, height, framerate;
      rs_format format;
      rs_get_stream_mode(dev, requested_stream, k, &width, &height, &format, &framerate, nullptr);

      if ((requested_stream == RS_STREAM_DEPTH && format != RS_FORMAT_Z16) ||
          (requested_stream == RS_STREAM_COLOR && format != RS_FORMAT_BGRA8))
        continue;

      VideoCaptureFormat supported_format;
      supported_format.pixel_format = PIXEL_FORMAT_RGB32;
      supported_format.frame_size.SetSize(width, height);
      supported_format.frame_rate = framerate;

      LOG(ERROR) << "  " << width << "\tx " << height << "\t@ " << framerate << "Hz\t" << format;
    }
  }
}

VideoCaptureDeviceLibrealsense::VideoCaptureDeviceLibrealsense(const Name& device_name)
    : device_name_(device_name){
  std::string delimiter("-");
  std::string name = device_name_.name().substr(0, device_name_.name().find(delimiter));
  std::string stream = device_name_.name().substr(device_name_.name().find(delimiter) + 1);

  if (stream == "COLOR") {
    stream_type_ = static_cast<int>(RS_STREAM_COLOR);
  } else if (stream == "DEPTH") {
    stream_type_ = static_cast<int>(RS_STREAM_DEPTH);
  } else {
    NOTREACHED();
    return;
  }
}

VideoCaptureDeviceLibrealsense::~VideoCaptureDeviceLibrealsense() {
}

void VideoCaptureDeviceLibrealsense::AllocateAndStart(
    const VideoCaptureParams& params,
    std::unique_ptr<VideoCaptureDevice::Client> client) {
  RsCaptureService::GetInstance()->StartStream(static_cast<rs_stream>(stream_type_), std::move(client));
}

void VideoCaptureDeviceLibrealsense::StopAndDeAllocate() {
  RsCaptureService::GetInstance()->StopStream(static_cast<rs_stream>(stream_type_));
}

}  // namespace media
