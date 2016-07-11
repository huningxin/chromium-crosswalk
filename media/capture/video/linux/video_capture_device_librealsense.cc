// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "media/capture/video/linux/video_capture_device_librealsense.h"

#include <stddef.h>

#include <list>

#include "base/bind.h"
#include "base/strings/stringprintf.h"
#include "build/build_config.h"

#include <librealsense/rs.hpp>  // NOLINT

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
    rs_context_ = new rs::context;
    int device_count = rs_context_->get_device_count();   
    if (!device_count) {
      LOG(ERROR) << "No device detected. Is it plugged in?";
      return;
    }

    // Assume the first device.
    rs_device_ = rs_context_->get_device(0);
  }

  void StartStream(rs::stream stream,
                   scoped_ptr<VideoCaptureDevice::Client> client) {
    DCHECK(task_runner_->BelongsToCurrentThread());
    if (clients_[static_cast<int>(stream)].get()) {
      LOG(ERROR) << "Stream " << stream << " is already started";
      return;
    }
    DCHECK(client);

    if (rs_device_->is_streaming()) {
      LOG(ERROR) << "Stop device";
      rs_device_->stop();
    }

    clients_[static_cast<int>(stream)] = std::move(client);

    rs_device_->enable_stream(stream, rs::preset::best_quality);

    LOG(ERROR) << "Enabled Stream: " << rs_device_->get_stream_width(stream) << " " 
        << rs_device_->get_stream_height(stream) << " " << rs_device_->get_stream_format(stream);

    capture_formats_[static_cast<int>(stream)].frame_size.SetSize(rs_device_->get_stream_width(stream),
                                       rs_device_->get_stream_height(stream));
    capture_formats_[static_cast<int>(stream)].frame_rate = rs_device_->get_stream_framerate(stream);
    capture_formats_[static_cast<int>(stream)].pixel_format = PIXEL_FORMAT_RGB24;

    if (rs_device_->get_stream_format(stream) == rs::format::z16) {
      rgb_.reset(new uint8_t[rs_device_->get_stream_width(stream) * rs_device_->get_stream_height(stream) * 3]);
    }

    rs_device_->start();

    if (is_capturing_ == false) {
      is_capturing_ = true;
      task_runner_->PostTask(
          FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
    }
  }
  void StopStream(rs::stream stream) {
    DCHECK(task_runner_->BelongsToCurrentThread());

    LOG(ERROR) << "Stop Stream: " << stream;

    if (rs_device_->is_streaming()) {
      LOG(ERROR) << "Stop device";
      rs_device_->stop();
    }

    clients_[static_cast<int>(stream)].reset();
    rs_device_->disable_stream(stream);

    if (clients_[static_cast<int>(rs::stream::color)].get() ||
        clients_[static_cast<int>(rs::stream::depth)].get()) {
      rs_device_->start();
      LOG(ERROR) << "Restart device";
    } else {
      is_capturing_ = false;
    }
  }

 private:
  friend class base::RefCountedThreadSafe<RsCaptureDelegate>;
  ~RsCaptureDelegate() {}

  void DoCapture() {
    DCHECK(task_runner_->BelongsToCurrentThread());
    if (!is_capturing_) {
      LOG(ERROR) << "Stop capture";
      return;
    }

    rs_device_->wait_for_frames();

    if (clients_[static_cast<int>(rs::stream::color)].get()) {
      if (rs_device_->get_stream_format(rs::stream::color) == rs::format::rgb8) {
        clients_[static_cast<int>(rs::stream::color)]->OnIncomingCapturedData(
            reinterpret_cast<const uint8_t *>(rs_device_->get_frame_data(rs::stream::color)),
            capture_formats_[static_cast<int>(rs::stream::color)].ImageAllocationSize(),
            capture_formats_[static_cast<int>(rs::stream::color)], 0, base::TimeTicks::Now());
      }
    }

    if (clients_[static_cast<int>(rs::stream::depth)].get()) {
      if (rs_device_->get_stream_format(rs::stream::depth) == rs::format::z16) {
        make_depth_histogram(rgb_.get(),
                             reinterpret_cast<const uint16_t *>(rs_device_->get_frame_data(rs::stream::depth)),
                             rs_device_->get_stream_width(rs::stream::depth), rs_device_->get_stream_height(rs::stream::depth));
        clients_[static_cast<int>(rs::stream::depth)]->OnIncomingCapturedData(
            rgb_.get(),
            capture_formats_[static_cast<int>(rs::stream::depth)].ImageAllocationSize(),
            capture_formats_[static_cast<int>(rs::stream::depth)], 0, base::TimeTicks::Now());
      }
    }

    task_runner_->PostTask(
        FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
  }

  const scoped_refptr<base::SingleThreadTaskRunner> task_runner_;
  const VideoCaptureDevice::Name device_name_;

  // depth = 0, color = 1
  scoped_ptr<VideoCaptureDevice::Client> clients_[2];
  VideoCaptureFormat capture_formats_[2];

  bool is_capturing_;

  rs::context* rs_context_;
  rs::device* rs_device_;

  scoped_ptr<uint8_t[]> rgb_;

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

  void StartStream(rs::stream stream, scoped_ptr<VideoCaptureDevice::Client> client) {
    DCHECK(client);
    rs_thread_.message_loop()->PostTask(
        FROM_HERE,
        base::Bind(&RsCaptureDelegate::StartStream, capture_impl_,
                   stream,
                   base::Passed(&client)));
  }

  void StopStream(rs::stream stream) {
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

  rs::context ctx;

  int device_count = ctx.get_device_count();   
  if (!device_count) LOG(ERROR) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs::device * dev = ctx.get_device(i);
    LOG(ERROR) << "Device " << i << " - " << dev->get_name()
        << " Serial number: " << dev->get_serial() << " Firmware version: "
        << dev->get_firmware_version();

    // Show which streams are supported by this device
    for (int j = 0; j < RS_STREAM_COUNT; ++j)
    {
        // Determine number of available streaming modes (zero means stream is unavailable) 
        rs::stream strm = (rs::stream)j;
        int mode_count = dev->get_stream_mode_count(strm);
        if(mode_count == 0) continue;

        if (!(strm == rs::stream::depth || strm == rs::stream::color))
          continue;

        // Show each available mode for this stream
        LOG(ERROR) << " Stream " << strm << " - " << mode_count << "\n";

        std::string name = std::string(dev->get_name()) + std::string("-") + std::string(rs_stream_to_string((rs_stream)strm));
        std::string id = std::string(dev->get_serial()) + std::string("-") + std::string(rs_stream_to_string((rs_stream)strm));
        device_names->push_back(VideoCaptureDevice::Name(
          name, id, VideoCaptureDevice::Name::LIBREALSENSE));        
    }
  }
}

void VideoCaptureDeviceLibrealsense::GetDeviceSupportedFormats(
    const Name& device,
    VideoCaptureFormats* formats) {
  rs::context ctx;

  std::string delimiter("-");
  std::string name = device.name().substr(0, device.name().find(delimiter));
  std::string stream = device.name().substr(device.name().find(delimiter) + 1);

  rs::stream requested_stream;
  if (stream == "COLOR") {
    requested_stream = rs::stream::color;
  } else if (stream == "DEPTH") {
    requested_stream = rs::stream::depth;
  } else {
    NOTREACHED();
    return;
  }

  LOG(ERROR) << "VideoCaptureDeviceLibrealsense::GetDeviceSupportedFormats: " << name << " " << stream;

  int device_count = ctx.get_device_count();   
  if (!device_count) LOG(ERROR) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs::device * dev = ctx.get_device(i);
    LOG(ERROR) << "Device " << i << " - " << dev->get_name() << ":\n";

    if (name != std::string(dev->get_name()))
      continue;

    int mode_count = dev->get_stream_mode_count(requested_stream);
    if (mode_count == 0) {
      LOG(ERROR) << "No modes found";
    }

    for(int k = 0; k < mode_count; ++k){
      // Show width, height, format, and framerate, the settings required to enable the stream in this mode
      int width, height, framerate;
      rs::format format;
      dev->get_stream_mode(requested_stream, k, width, height, format, framerate);

      if ((requested_stream == rs::stream::depth && format != rs::format::z16) ||
          (requested_stream == rs::stream::color && format != rs::format::bgra8))
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
    stream_type_ = static_cast<int>(rs::stream::color);
  } else if (stream == "DEPTH") {
    stream_type_ = static_cast<int>(rs::stream::depth);
  } else {
    NOTREACHED();
    return;
  }
}

VideoCaptureDeviceLibrealsense::~VideoCaptureDeviceLibrealsense() {
}

void VideoCaptureDeviceLibrealsense::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<VideoCaptureDevice::Client> client) {
  RsCaptureService::GetInstance()->StartStream(static_cast<rs::stream>(stream_type_), std::move(client));
}

void VideoCaptureDeviceLibrealsense::StopAndDeAllocate() {
  RsCaptureService::GetInstance()->StopStream(static_cast<rs::stream>(stream_type_));
}

}  // namespace media
