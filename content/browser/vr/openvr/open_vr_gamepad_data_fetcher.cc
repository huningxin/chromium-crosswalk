// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/openvr/open_vr_gamepad_data_fetcher.h"

#include "base/bind.h"
#include "base/bind_helpers.h"
#include "base/logging.h"
#include "base/strings/stringprintf.h"
#include "base/thread_task_runner_handle.h"
#include "third_party/WebKit/public/platform/WebGamepads.h"
#include "third_party/openvr/openvr/headers/openvr.h"
#include "ui/gfx/transform.h"
#include "ui/gfx/transform_util.h"

namespace content {

using namespace blink;

OpenVRGamepadDataFetcher::OpenVRGamepadDataFetcher()
  : vr_system_(nullptr) {

}

OpenVRGamepadDataFetcher::~OpenVRGamepadDataFetcher() {
  /*if (vr_system_) {
    vr::VR_Shutdown();
    vr_system_ = nullptr;
  }*/
}

void OpenVRGamepadDataFetcher::OnAddedToProvider() {
  // Loading the SteamVR Runtime
  vr::EVRInitError error = vr::VRInitError_None;
  vr_system_ = vr::VR_Init(&error, vr::VRApplication_Scene);

  if (error != vr::VRInitError_None) {
    vr_system_ = nullptr;
    return;
  }
}

void SetGamepadButton(WebGamepad& pad,
    vr::VRControllerState_t& controller_state, uint64_t supported_buttons,
    vr::EVRButtonId button_id) {
  uint64_t button_mask = vr::ButtonMaskFromId(button_id);
  if ((supported_buttons & button_mask) != 0) {
    bool button_pressed = (controller_state.ulButtonPressed & button_mask) != 0;
    bool button_touched = (controller_state.ulButtonTouched & button_mask) != 0;
    pad.buttons[pad.buttonsLength].touched = button_touched;
    pad.buttons[pad.buttonsLength].pressed = button_pressed;
    pad.buttons[pad.buttonsLength].value = button_pressed ? 1.0 : 0.0;
    pad.buttonsLength++;
  }
}

void OpenVRGamepadDataFetcher::GetGamepadData(
    bool devices_changed_hint) {
  if (!vr_system_)
    return;

  vr::TrackedDevicePose_t tracked_devices_poses[vr::k_unMaxTrackedDeviceCount];
  vr_system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated,
      0.04f, tracked_devices_poses, vr::k_unMaxTrackedDeviceCount);

  for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
    if (vr_system_->GetTrackedDeviceClass(i) !=
        vr::TrackedDeviceClass_Controller)
        continue;

    PadState* state = GetPadState(i);
    if (!state)
      continue; // No available slot.

    WebGamepad& pad = state->data;

    vr::VRControllerState_t controller_state;
    if (vr_system_->GetControllerState(i, &controller_state)) {
      pad.timestamp = controller_state.unPacketNum;
      pad.connected = true;

      // TODO: vr::Prop_AttachedDeviceId_String
      swprintf(pad.id, WebGamepad::idLengthCap, L"OpenVR Gamepad");

      uint64_t supported_buttons = vr_system_->GetUint64TrackedDeviceProperty(
          i, vr::Prop_SupportedButtons_Uint64);

      pad.buttonsLength = 0;
      pad.axesLength = 0;

      for (int j = 0; j < vr::k_unControllerStateAxisCount; ++j) {
        int32_t axis_type = vr_system_->GetInt32TrackedDeviceProperty(
            i, static_cast<vr::TrackedDeviceProperty>(
                vr::Prop_Axis0Type_Int32+j));
        switch (axis_type) {
        case vr::k_eControllerAxis_Joystick:
        case vr::k_eControllerAxis_TrackPad:
          pad.axes[pad.axesLength++] = controller_state.rAxis[j].x;
          pad.axes[pad.axesLength++] = controller_state.rAxis[j].y;

          SetGamepadButton(pad, controller_state, supported_buttons,
              static_cast<vr::EVRButtonId>(vr::k_EButton_Axis0+j));

          break;
        case vr::k_eControllerAxis_Trigger:
          pad.buttons[pad.buttonsLength].value = controller_state.rAxis[j].x;

          uint64_t button_mask = vr::ButtonMaskFromId(
              static_cast<vr::EVRButtonId>(vr::k_EButton_Axis0+j));
          if ((supported_buttons & button_mask) != 0) {
            pad.buttons[pad.buttonsLength].pressed =
                (controller_state.ulButtonPressed & button_mask) != 0;
          }

          pad.buttonsLength++;
          break;
        }
      }

      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_A);
      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_Grip);
      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_ApplicationMenu);
      //SetGamepadButton(pad, controller_state, supported_buttons,
      //  vr::k_EButton_System);

      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_DPad_Left);
      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_DPad_Up);
      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_DPad_Right);
      SetGamepadButton(pad, controller_state, supported_buttons,
          vr::k_EButton_DPad_Down);
    }

    const vr::TrackedDevicePose_t &pose = tracked_devices_poses[i];
    if (pose.bPoseIsValid) {
      const vr::HmdMatrix34_t& mat = pose.mDeviceToAbsoluteTracking;
      gfx::Transform transform(
          mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
          mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
          mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
          0, 0, 0, 1);

      gfx::DecomposedTransform decomposed_transform;
      gfx::DecomposeTransform(&decomposed_transform, transform);

      pad.pose.isNull = false;

      pad.pose.orientation.isNull = false;
      pad.pose.orientation.x = decomposed_transform.quaternion[0];
      pad.pose.orientation.y = decomposed_transform.quaternion[1];
      pad.pose.orientation.z = decomposed_transform.quaternion[2];
      pad.pose.orientation.w = decomposed_transform.quaternion[3];

      pad.pose.position.isNull = false;
      pad.pose.position.x = decomposed_transform.translate[0];
      pad.pose.position.y = decomposed_transform.translate[1];
      pad.pose.position.z = decomposed_transform.translate[2];
      pad.pose.position.w = 0.0f;

      //state->angularVelocity = HmdVector3ToWebVR(pose.vAngularVelocity);
      //state->linearVelocity = HmdVector3ToWebVR(pose.vVelocity);
    } else {
      pad.pose.isNull = true;
      pad.pose.orientation.isNull = true;
      pad.pose.position.isNull = true;
    }
  }
}

void OpenVRGamepadDataFetcher::PauseHint(bool paused) {
  // TODO
}

void OpenVRGamepadDataFetcher::Vibrate(int source_id, int duration) {
  if (duration > 3) {
    vr_system_->TriggerHapticPulse(source_id, 0, 3000);

    // The maximum duration value we can pass into TriggerHapticPulse is something
    // around 3500, or 3.5ms. In order to keep the vibration going for the
    // requested duration we have to refresh is every so often.

    base::ThreadTaskRunnerHandle::Get()->PostDelayedTask(
        FROM_HERE, base::Bind(&OpenVRGamepadDataFetcher::Vibrate,
            base::Unretained(this), source_id, duration - 6),
        base::TimeDelta::FromMilliseconds(6));
  } else {
    vr_system_->TriggerHapticPulse(source_id, 0, duration * 1000);
  }
}

}  // namespace content
