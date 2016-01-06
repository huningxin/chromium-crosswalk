// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_OPENVR_GAMEPAD_DATA_FETCHER_H_
#define CONTENT_BROWSER_VR_OPENVR_GAMEPAD_DATA_FETCHER_H_

#include <string>

#include "content/browser/gamepad/gamepad_data_fetcher.h"
#include "third_party/openvr/openvr/headers/openvr.h"

namespace content {

class OpenVRGamepadDataFetcher : public GamepadDataFetcher {
 public:
  OpenVRGamepadDataFetcher();
  ~OpenVRGamepadDataFetcher() override;

  GamepadSource source() { return GAMEPAD_SOURCE_OPENVR; }

  void GetGamepadData(bool devices_changed_hint) override;
  void PauseHint(bool paused) override;
  void OnAddedToProvider() override;

  void Vibrate(int source_id, int duration) override;

 private:
  vr::IVRSystem* vr_system_;

  DISALLOW_COPY_AND_ASSIGN(OpenVRGamepadDataFetcher);
};

}  // namespace content
#endif // CONTENT_BROWSER_VR_OPENVR_GAMEPAD_DATA_FETCHER_H_
