// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_OCULUS_VR_DEVICE_PROVIDER_H
#define CONTENT_BROWSER_VR_OCULUS_VR_DEVICE_PROVIDER_H

#include <map>

#include "content/browser/vr/vr_device.h"
#include "content/browser/vr/vr_device_provider.h"

struct ovrHmdStruct;

namespace content {

class GamepadConsumer;

class OculusVRDeviceProvider : public VRDeviceProvider {
 public:
  OculusVRDeviceProvider();
  ~OculusVRDeviceProvider() override;

  void GetDevices(std::vector<VRDevice*>* devices) override;
  void Initialize() override;

 private:
  void Shutdown();

  bool initialized_;

  VRDevice* device_; // Oculus API only supports one device
};

}  // namespace content

#endif  // CONTENT_BROWSER_VR_OCULUS_VR_DEVICE_PROVIDER_H
