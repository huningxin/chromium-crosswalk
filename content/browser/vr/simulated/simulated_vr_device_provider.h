// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_PROVIDER_H
#define CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_PROVIDER_H

#include <map>

#include "content/browser/vr/vr_device.h"
#include "content/browser/vr/vr_device_provider.h"

namespace content {

class SimulatedVRDeviceProvider : public VRDeviceProvider {
 public:
  SimulatedVRDeviceProvider();
  ~SimulatedVRDeviceProvider() override;

  void GetDevices(std::vector<VRDevice*>* devices) override;
  void Initialize() override;

 private:
  VRDevice* device_;
};

}  // namespace content

#endif  // CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_PROVIDER_H
