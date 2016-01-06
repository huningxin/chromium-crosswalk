// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/simulated/simulated_vr_device_provider.h"

#include "content/browser/vr/simulated/simulated_vr_device.h"

namespace content {

SimulatedVRDeviceProvider::SimulatedVRDeviceProvider()
    : device_(nullptr) {
}

SimulatedVRDeviceProvider::~SimulatedVRDeviceProvider() {
  if (device_) {
    delete device_;
    device_ = nullptr;
  }
}

void SimulatedVRDeviceProvider::GetDevices(std::vector<VRDevice*>* devices) {
  if (!device_)
    device_ = new SimulatedVRDevice(this);

  devices->push_back(device_);
}

void SimulatedVRDeviceProvider::Initialize() {

}

}  // namespace content
