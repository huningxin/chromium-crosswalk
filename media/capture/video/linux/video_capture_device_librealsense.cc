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

        if (!(strm == RS_STREAM_DEPTH || 
              strm == RS_STREAM_COLOR || 
              strm == RS_STREAM_INFRARED ||
              strm == RS_STREAM_INFRARED2 ||
              strm == RS_STREAM_FISHEYE))
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
  } else if (stream == "INFRARED") {
    requested_stream = RS_STREAM_INFRARED;
  } else if (stream == "INFRARED2") {
    requested_stream = RS_STREAM_INFRARED2;
  } else if (stream == "FISHEYE") {
    requested_stream = RS_STREAM_FISHEYE;
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
          (requested_stream == RS_STREAM_COLOR && format != RS_FORMAT_BGRA8) ||
          (requested_stream == RS_STREAM_INFRARED && format != RS_FORMAT_Y8) ||
          (requested_stream == RS_STREAM_INFRARED2 && format != RS_FORMAT_Y8) ||
          (requested_stream == RS_STREAM_FISHEYE && format != RS_FORMAT_RAW8))
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
    stream_ = RS_STREAM_COLOR;
  } else if (stream == "DEPTH") {
    stream_ = RS_STREAM_DEPTH;
  } else if (stream == "INFRARED") {
    stream_ = RS_STREAM_INFRARED;
  } else if (stream == "INFRARED2") {
    stream_ = RS_STREAM_INFRARED2;
  } else if (stream == "FISHEYE") {
    stream_ = RS_STREAM_FISHEYE;
  } else {
    NOTREACHED();
    return;
  }

  rs_capture_service_ = RsCaptureService::GetInstance();
}

VideoCaptureDeviceLibrealsense::~VideoCaptureDeviceLibrealsense() {
}

void FrameCallback(rs_device* dev, rs_frame_ref* frame, void* user) {
  VideoCaptureDeviceLibrealsense* vcd = reinterpret_cast<VideoCaptureDeviceLibrealsense*>(user);

  vcd->OnFrame(frame);
}

void VideoCaptureDeviceLibrealsense::AllocateAndStart(
    const VideoCaptureParams& params,
    std::unique_ptr<VideoCaptureDevice::Client> client) {

  client_ = std::move(client);

  rs_capture_service_->StartStream(stream_, this);
  rs_device* device = rs_capture_service_->GetDevice();

  int width = rs_get_stream_width(device, stream_, nullptr);
  int height = rs_get_stream_height(device, stream_, nullptr);
  rs_format format = rs_get_stream_format(device, stream_, nullptr);

  LOG(ERROR) << "Enabled Stream: " << width << " " << height << " " << format;

  capture_format_.frame_size.SetSize(width, height);
  capture_format_.frame_rate = rs_get_stream_framerate(device, stream_, nullptr);
  capture_format_.pixel_format = PIXEL_FORMAT_RGB24;

  if (format == RS_FORMAT_Z16 ||
      format == RS_FORMAT_RAW8 ||
      format == RS_FORMAT_Y8) {
    rgb_.reset(new uint8_t[width * height * 3]);
  }
}

void VideoCaptureDeviceLibrealsense::StopAndDeAllocate() {
  rs_capture_service_->StopStream(stream_, this);
}

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

inline void make_raw8_rgb(uint8_t* rgb_image, const uint8_t raw8_image[], int width, int height) {
  for(int i = 0; i < width*height; ++i) {
        if(uint8_t v = raw8_image[i]) {
            rgb_image[i*3 + 0] = v;
            rgb_image[i*3 + 1] = v;
            rgb_image[i*3 + 2] = v;
        }
    }
}

void VideoCaptureDeviceLibrealsense::OnFrame(rs_frame_ref* frame) {
  const uint8_t* data = nullptr;
  rs_format format = rs_get_detached_frame_format(frame, nullptr);
  if (format == RS_FORMAT_Z16) {
    make_depth_histogram(rgb_.get(),
                         reinterpret_cast<const uint16_t *>(rs_get_detached_frame_data(frame, nullptr)),
                         rs_get_detached_frame_width(frame, nullptr), rs_get_detached_frame_height(frame, nullptr));
    data = rgb_.get();
  } else if (format == RS_FORMAT_RGB8) {
    data = reinterpret_cast<const uint8_t *>(rs_get_detached_frame_data(frame, nullptr));
  } else if (format == RS_FORMAT_RAW8) {
    make_raw8_rgb(rgb_.get(),
                  reinterpret_cast<const uint8_t *>(rs_get_detached_frame_data(frame, nullptr)),
                  rs_get_detached_frame_width(frame, nullptr), rs_get_detached_frame_height(frame, nullptr));
    data = rgb_.get();
  } else if (format == RS_FORMAT_Y8) {
    make_raw8_rgb(rgb_.get(),
                  reinterpret_cast<const uint8_t *>(rs_get_detached_frame_data(frame, nullptr)),
                  rs_get_detached_frame_width(frame, nullptr), rs_get_detached_frame_height(frame, nullptr));
    data = rgb_.get();
  } else {
    LOG(ERROR) << "Not support format " << format;
  }

  if (data) {
    client_->OnIncomingCapturedData(
        data,
        capture_format_.ImageAllocationSize(),
        capture_format_, 0, base::TimeTicks::Now());
  }
}

}  // namespace media
