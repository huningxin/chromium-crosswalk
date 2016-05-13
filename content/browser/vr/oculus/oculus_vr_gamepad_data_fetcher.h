// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_OCULUS_VR_GAMEPAD_DATA_FETCHER_H_
#define CONTENT_BROWSER_VR_OCULUS_VR_GAMEPAD_DATA_FETCHER_H_

#include <string>

#include "content/browser/gamepad/gamepad_data_fetcher.h"

struct ovrHmdStruct;

namespace content {

class OculusVRGamepadDataFetcher : public GamepadDataFetcher {
 public:
  OculusVRGamepadDataFetcher(ovrHmdStruct* session);
  ~OculusVRGamepadDataFetcher() override;

  GamepadSource source() { return GAMEPAD_SOURCE_OCULUS; }

  void GetGamepadData(bool devices_changed_hint) override;
  void PauseHint(bool paused) override;
  void OnAddedToProvider() override;

 private:
  ovrHmdStruct* session_;

  DISALLOW_COPY_AND_ASSIGN(OculusVRGamepadDataFetcher);
};

}  // namespace content
#endif // CONTENT_BROWSER_VR_OCULUS_R_GAMEPAD_DATA_FETCHER_H_
