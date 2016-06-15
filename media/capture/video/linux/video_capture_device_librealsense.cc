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
      const VideoCaptureDevice::Name& device_name,
      const scoped_refptr<base::SingleThreadTaskRunner>& task_runner) :
      task_runner_(task_runner),
      device_name_(device_name),
      is_capturing_(false) {
    rs_context_.reset(new rs::context);
    std::string delimiter("-");
    name_ = device_name.name().substr(0, device_name.name().find(delimiter));
    std::string stream = device_name.name().substr(device_name.name().find(delimiter) + 1);
    if (stream == "COLOR")
      rs_stream_ = rs::stream::color;
    else if (stream == "DEPTH")
      rs_stream_ = rs::stream::depth;
    else
      NOTREACHED();

    DVLOG(1) << "RsCaptureDelegate::RsCaptureDelegate: " << name_ << " " << rs_stream_;
  }

  // Forward-to versions of VideoCaptureDevice virtual methods.
  void AllocateAndStart(int width,
                        int height,
                        float frame_rate,
                        scoped_ptr<VideoCaptureDevice::Client> client) {
    DCHECK(task_runner_->BelongsToCurrentThread());
    DCHECK(client);
    client_ = std::move(client);

    int device_count = rs_context_->get_device_count();   
    if (!device_count) {
      SetErrorState(FROM_HERE, "No device detected. Is it plugged in?");
      return;
    }

    for (int i = 0; i < device_count; ++i){
      // Show the device name and information
      rs::device * dev = rs_context_->get_device(i);
      DVLOG(1) << "Device " << i << " - " << dev->get_name() << ":\n";

      if (name_ != std::string(dev->get_name()))
        continue;

      rs_device_ = dev;

      rs_device_->enable_stream(rs_stream_, rs::preset::best_quality);

      DVLOG(1) << "Enabled Stream: " << rs_device_->get_stream_width(rs_stream_) << " " 
          << rs_device_->get_stream_height(rs_stream_) << " " << rs_device_->get_stream_format(rs_stream_);

      capture_format_.frame_size.SetSize(rs_device_->get_stream_width(rs_stream_),
                                         rs_device_->get_stream_height(rs_stream_));
      capture_format_.frame_rate = rs_device_->get_stream_framerate(rs_stream_);
      capture_format_.pixel_format = PIXEL_FORMAT_RGB24;

      if (rs_device_->get_stream_format(rs_stream_) == rs::format::z16) {
        rgb_.reset(new uint8_t[rs_device_->get_stream_width(rs_stream_) * rs_device_->get_stream_height(rs_stream_) * 3]);
      }

      rs_device_->start();

      break;
    }

    is_capturing_ = true;
    task_runner_->PostTask(
        FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
  }
  void StopAndDeAllocate() {
    DCHECK(task_runner_->BelongsToCurrentThread());
    rs_device_->stop();

    is_capturing_ = false;

    client_.reset();
  }

 private:
  friend class base::RefCountedThreadSafe<RsCaptureDelegate>;
  ~RsCaptureDelegate() {}

  void DoCapture() {
    DCHECK(task_runner_->BelongsToCurrentThread());
    if (!is_capturing_) {
      return;
    }

    rs_device_->wait_for_frames();

    if (rs_device_->get_stream_format(rs_stream_) == rs::format::rgb8) {
      client_->OnIncomingCapturedData(
          reinterpret_cast<const uint8_t *>(rs_device_->get_frame_data(rs_stream_)),
          capture_format_.ImageAllocationSize(),
          capture_format_, 0, base::TimeTicks::Now());
    } else if (rs_device_->get_stream_format(rs_stream_) == rs::format::z16) {
      make_depth_histogram(rgb_.get(),
                           reinterpret_cast<const uint16_t *>(rs_device_->get_frame_data(rs_stream_)),
                           rs_device_->get_stream_width(rs_stream_), rs_device_->get_stream_height(rs_stream_));
      client_->OnIncomingCapturedData(
          rgb_.get(),
          capture_format_.ImageAllocationSize(),
          capture_format_, 0, base::TimeTicks::Now());
    }

    task_runner_->PostTask(
        FROM_HERE, base::Bind(&RsCaptureDelegate::DoCapture, this));
  }

  void SetErrorState(const tracked_objects::Location& from_here,
                     const std::string& reason) {
    DCHECK(task_runner_->BelongsToCurrentThread());
    is_capturing_ = false;
    client_->OnError(from_here, reason);
  }

  const scoped_refptr<base::SingleThreadTaskRunner> task_runner_;
  const VideoCaptureDevice::Name device_name_;

  scoped_ptr<VideoCaptureDevice::Client> client_;

  bool is_capturing_;

  std::string name_;
  scoped_ptr<rs::context> rs_context_;
  rs::device* rs_device_;
  rs::stream rs_stream_;

  VideoCaptureFormat capture_format_;
  scoped_ptr<uint8_t[]> rgb_;

  DISALLOW_COPY_AND_ASSIGN(RsCaptureDelegate);
};

void VideoCaptureDeviceLibrealsense::GetDeviceNames(Names* device_names) {
  DCHECK(device_names->empty());
  DVLOG(1) << "VideoCaptureDeviceLibrealsense::GetDeviceNames";

  rs::context ctx;

  int device_count = ctx.get_device_count();   
  if (!device_count) DVLOG(1) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs::device * dev = ctx.get_device(i);
    DVLOG(1) << "Device " << i << " - " << dev->get_name()
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
        DVLOG(1) << " Stream " << strm << " - " << mode_count << "\n";

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

  DVLOG(1) << "VideoCaptureDeviceLibrealsense::GetDeviceSupportedFormats: " << name << " " << stream;

  int device_count = ctx.get_device_count();   
  if (!device_count) DVLOG(1) << "No device detected. Is it plugged in?";

  for (int i = 0; i < device_count; ++i){
    // Show the device name and information
    rs::device * dev = ctx.get_device(i);
    DVLOG(1) << "Device " << i << " - " << dev->get_name() << ":\n";

    if (name != std::string(dev->get_name()))
      continue;

    int mode_count = dev->get_stream_mode_count(requested_stream);
    if (mode_count == 0) {
      DVLOG(1) << "No modes found";
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

      DVLOG(1) << "  " << width << "\tx " << height << "\t@ " << framerate << "Hz\t" << format;
    }
  }
}

VideoCaptureDeviceLibrealsense::VideoCaptureDeviceLibrealsense(const Name& device_name)
    : rs_thread_("LibrealsenseCaptureThread"),
      device_name_(device_name){
}

VideoCaptureDeviceLibrealsense::~VideoCaptureDeviceLibrealsense() {
  // Check if the thread is running.
  // This means that the device has not been StopAndDeAllocate()d properly.
  DCHECK(!rs_thread_.IsRunning());
  rs_thread_.Stop();
}

void VideoCaptureDeviceLibrealsense::AllocateAndStart(
    const VideoCaptureParams& params,
    scoped_ptr<VideoCaptureDevice::Client> client) {
  if (rs_thread_.IsRunning())
    return;  // Wrong state.

  DCHECK(client);

  rs_thread_.Start();

  capture_impl_ = new RsCaptureDelegate(
      device_name_, rs_thread_.task_runner());
  if (!capture_impl_) {
    client->OnError(FROM_HERE, "Failed to create VideoCaptureDelegate");
    return;
  }
  rs_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&RsCaptureDelegate::AllocateAndStart, capture_impl_,
                 params.requested_format.frame_size.width(),
                 params.requested_format.frame_size.height(),
                 params.requested_format.frame_rate, base::Passed(&client)));
}

void VideoCaptureDeviceLibrealsense::StopAndDeAllocate() {
  if (!rs_thread_.IsRunning())
    return;  // Wrong state.
  rs_thread_.message_loop()->PostTask(
      FROM_HERE,
      base::Bind(&RsCaptureDelegate::StopAndDeAllocate, capture_impl_));
  rs_thread_.Stop();
  
}

}  // namespace media
