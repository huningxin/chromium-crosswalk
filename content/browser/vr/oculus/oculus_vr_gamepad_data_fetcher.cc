// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/oculus/oculus_vr_gamepad_data_fetcher.h"

#include "base/logging.h"
#include "base/strings/stringprintf.h"
#include "third_party/WebKit/public/platform/WebGamepads.h"
#include "third_party/libovr/LibOVR/Include/OVR_CAPI.h"

namespace content {

using namespace blink;

OculusVRGamepadDataFetcher::OculusVRGamepadDataFetcher(ovrHmdStruct* session)
  : session_(session) {

}

OculusVRGamepadDataFetcher::~OculusVRGamepadDataFetcher() {
  if (session_) {
    ovr_Destroy(session_);
    session_ = NULL;
  }
}

void OculusVRGamepadDataFetcher::OnAddedToProvider() {
  if (!session_) {
    ovrGraphicsLuid luid;
    if (OVR_FAILURE(ovr_Create(&session_, &luid))) {
      LOG(ERROR) << "OculusVRGamepadDataFetcher failed ovr_Create";
    }
  }
}

void SetGamepadButton(WebGamepad& pad,
    const ovrInputState& input_state, ovrButton button_id) {
  bool pressed = (input_state.Buttons & button_id) != 0;
  pad.buttons[pad.buttonsLength].pressed = pressed;
  pad.buttons[pad.buttonsLength].value = pressed ? 1.0f : 0.0f;
  pad.buttonsLength++;
}

void OculusVRGamepadDataFetcher::GetGamepadData(bool devices_changed_hint) {
  if (!session_)
    return;

  ovrInputState input_state;
  if ((OVR_SUCCESS(ovr_GetInputState(session_, ovrControllerType_Remote,
      &input_state)))) {

    PadState* state = GetPadState(0);
    if (state) {
      WebGamepad& pad = state->data;

      if (state->active_state == GAMEPAD_NEWLY_ACTIVE) {
        swprintf(pad.id, WebGamepad::idLengthCap, L"Oculus Remote");
        pad.connected = true;
      }

      pad.timestamp = input_state.TimeInSeconds;
      pad.axesLength = 0;
      pad.buttonsLength = 0;

      SetGamepadButton(pad, input_state, ovrButton_Enter);
      SetGamepadButton(pad, input_state, ovrButton_Back);
      SetGamepadButton(pad, input_state, ovrButton_Up);
      SetGamepadButton(pad, input_state, ovrButton_Down);
      SetGamepadButton(pad, input_state, ovrButton_Left);
      SetGamepadButton(pad, input_state, ovrButton_Right);
    }
  }
}

void OculusVRGamepadDataFetcher::PauseHint(bool paused) {
  // TODO
}

}  // namespace content
