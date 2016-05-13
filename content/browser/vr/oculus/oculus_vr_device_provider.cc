// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/oculus/oculus_vr_device_provider.h"

#include "base/bind.h"
#include "content/browser/gamepad/gamepad_consumer.h"
#include "content/browser/gamepad/gamepad_service.h"
#include "content/browser/vr/oculus/oculus_vr_device.h"
#include "content/public/browser/browser_thread.h"

#include <algorithm>
#include <memory>

namespace content {

OculusVRDeviceProvider::OculusVRDeviceProvider()
    : VRDeviceProvider()
    , initialized_(false)
    , device_(false) {
}

OculusVRDeviceProvider::~OculusVRDeviceProvider() {
  Shutdown();
}

void OculusVRDeviceProvider::GetDevices(std::vector<VRDevice*>* devices) {
  Initialize();

  if (!initialized_)
    return;

  if (!device_) {
    ovrSession session;
    ovrGraphicsLuid luid;
    ovrResult result = ovr_Create(&session, &luid);

    if (OVR_FAILURE(result)) {
      return;
    }

    device_ = new OculusVRDevice(this, session);
  }

  if (device_)
    devices->push_back(device_);
}

void OculusVRDeviceProvider::Initialize() {
  if (!initialized_) {
    if (OVR_SUCCESS(ovr_Initialize(nullptr)))
      initialized_ = true;
  }
}

void OculusVRDeviceProvider::Shutdown() {
  if (device_) {
    delete device_;
    device_ = nullptr;
  }

  if (initialized_) {
    ovr_Shutdown();
    initialized_ = false;
  }
}

}  // namespace content
