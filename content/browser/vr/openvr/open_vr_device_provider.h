// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_OPENVR_DEVICE_PROVIDER_H
#define CONTENT_BROWSER_VR_OPENVR_DEVICE_PROVIDER_H

#include <map>

#include "content/browser/vr/vr_device.h"
#include "content/browser/vr/vr_device_provider.h"

namespace vr {

class IVRSystem;

}  // namespace vr

namespace content {

class GamepadConsumer;

class OpenVRDeviceProvider : public VRDeviceProvider {
 public:
  OpenVRDeviceProvider();
  ~OpenVRDeviceProvider() override;

  void GetDevices(std::vector<VRDevice*>* devices) override;
  void Initialize() override;

 private:
  void Shutdown();

  bool initialized_;

  VRDevice* device_;
  vr::IVRSystem* vr_system_;
  GamepadConsumer* gamepad_consumer_;
};

}  // namespace content

#endif  // CONTENT_BROWSER_VR_OPENVR_DEVICE_PROVIDER_H
