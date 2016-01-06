// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/openvr/open_vr_device_provider.h"

#include "base/logging.h"
#include "content/browser/gamepad/gamepad_consumer.h"
#include "content/browser/gamepad/gamepad_service.h"
#include "content/browser/vr/openvr/open_vr_device.h"
#include "content/browser/vr/openvr/open_vr_gamepad_data_fetcher.h"
#include "third_party/openvr/openvr/headers/openvr.h"

#include <algorithm>
#include <memory>

namespace content {

// Keeps the GamepadProvider alive while the OpenVRDeviceProvider is in use.
class OpenVRGamepadConsumer : public GamepadConsumer {
public:
  ~OpenVRGamepadConsumer() override {};
  void OnGamepadConnected(
      unsigned index,
      const blink::WebGamepad& gamepad) override {};
  void OnGamepadDisconnected(
      unsigned index,
      const blink::WebGamepad& gamepad) override {};
};

OpenVRDeviceProvider::OpenVRDeviceProvider()
    : VRDeviceProvider()
    , initialized_(false)
    , device_(nullptr)
    , vr_system_(nullptr) {
}

OpenVRDeviceProvider::~OpenVRDeviceProvider() {
  Shutdown();
}

void OpenVRDeviceProvider::GetDevices(std::vector<VRDevice*>* devices) {
  Initialize();

  if (!initialized_ || !vr_system_)
    return;

  vr::TrackedDevicePose_t tracked_devices_poses[vr::k_unMaxTrackedDeviceCount];
  vr_system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding,
      0, tracked_devices_poses, vr::k_unMaxTrackedDeviceCount);

  if (!device_) {
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
      if (vr_system_->GetTrackedDeviceClass(i) != vr::TrackedDeviceClass_HMD)
        continue;

      // Don't use OpenVR to track Oculus devices.
      vr::TrackedPropertyError error = vr::TrackedProp_Success;
      int32_t vendor_id = vr_system_->GetInt32TrackedDeviceProperty(i,
          vr::Prop_EdidVendorID_Int32, &error);
      if (error == vr::TrackedProp_Success && vendor_id == 0xD23E)
        continue;

      device_ = new OpenVRDevice(this, vr_system_, i);
      break;
    }
  }

  if (device_)
    devices->push_back(device_);
}

void OpenVRDeviceProvider::Initialize() {
  if (!initialized_) {
    // Does a fast check to see if any devices could possibly be connected.
    if (!vr::VR_IsHmdPresent())
      return;

    // Loading the SteamVR Runtime
    vr::EVRInitError error = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&error, vr::VRApplication_Scene);

    if (error != vr::VRInitError_None) {
      vr_system_ = nullptr;
      return;
    }

    GamepadService* gamepad_service = GamepadService::GetInstance();
    gamepad_consumer_ = new OpenVRGamepadConsumer();
    gamepad_service->ConsumerBecameActive(gamepad_consumer_);

    GamepadProvider* provider = gamepad_service->provider();
    if (provider) {
      provider->AddGamepadDataFetcher(std::unique_ptr<GamepadDataFetcher>(
          new OpenVRGamepadDataFetcher()));
      // TODO: Remove somewhere.
    } else {
      LOG(ERROR) << "No GamepadProvider available.";
    }

    initialized_ = true;
  }
}

void OpenVRDeviceProvider::Shutdown() {
  if (gamepad_consumer_) {
    GamepadService::GetInstance()->RemoveConsumer(gamepad_consumer_);
    //delete gamepad_consumer_; // EVIL BAD DEVELOPER! NO COOKIE!
    gamepad_consumer_ = nullptr;
  }

  if (device_) {
    delete device_;
    device_ = nullptr;
  }

  if (initialized_) {
    // TODO: Hm... when using the gamepad API we can't call shutdown until the
    // gamepad is aalso shut down, since it unloads the DLL and will cause
    // crashiness. For now just letting it live forever. :) Need to build some
    // type of ref-counting singleton.
    //vr::VR_Shutdown();
    vr_system_ = nullptr;
    initialized_ = false;
  }
}

}  // namespace content
