// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_GAMEPAD_GAMEPAD_PLATFORM_DATA_FETCHER_WIN_H_
#define CONTENT_BROWSER_GAMEPAD_GAMEPAD_PLATFORM_DATA_FETCHER_WIN_H_

#include <memory>

#include "build/build_config.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <stdlib.h>
#include <Unknwn.h>
#include <WinDef.h>
#include <windows.h>
#include <XInput.h>

#include "base/compiler_specific.h"
#include "base/macros.h"
#include "base/memory/weak_ptr.h"
#include "base/message_loop/message_loop.h"
#include "base/scoped_native_library.h"
#include "content/browser/gamepad/gamepad_data_fetcher.h"
#include "content/browser/gamepad/gamepad_standard_mappings.h"
#include "third_party/WebKit/public/platform/WebGamepads.h"

namespace content {

class GamepadPlatformDataFetcherWin : public GamepadDataFetcher {
 public:
  GamepadPlatformDataFetcherWin();
  ~GamepadPlatformDataFetcherWin() override;

  GamepadSource source() { return GAMEPAD_SOURCE_WIN_XINPUT; }

  void GetGamepadData(bool devices_changed_hint) override;

  void Vibrate(int source_id, int duration) override;
  void CancelVibration(int source_id) override;

 private:
  void OnAddedToProvider() override;

  // The three function types we use from xinput1_3.dll.
  typedef void (WINAPI *XInputEnableFunc)(BOOL enable);
  typedef DWORD (WINAPI *XInputGetCapabilitiesFunc)(
    DWORD dwUserIndex, DWORD dwFlags, XINPUT_CAPABILITIES* pCapabilities);
  typedef DWORD (WINAPI *XInputGetStateFunc)(
      DWORD dwUserIndex, XINPUT_STATE* pState);
  typedef DWORD (WINAPI *XInputSetStateFunc)(
      DWORD dwUserIndex, XINPUT_VIBRATION* pState);

  // Get functions from dynamically loading the xinput dll.
  // Returns true if loading was successful.
  bool GetXInputDllFunctions();

  // Scan for connected XInput and DirectInput gamepads.
  void EnumerateDevices();
  void GetXInputPadData(int i);

  base::ScopedNativeLibrary xinput_dll_;
  bool xinput_available_;

  // Function pointers to XInput functionality, retrieved in
  // |GetXinputDllFunctions|.
  XInputGetCapabilitiesFunc xinput_get_capabilities_;
  XInputGetStateFunc xinput_get_state_;
  XInputSetStateFunc xinput_set_state_;

  bool xinuput_connected_[XUSER_MAX_COUNT];

  DISALLOW_COPY_AND_ASSIGN(GamepadPlatformDataFetcherWin);
};

}  // namespace content

#endif  // CONTENT_BROWSER_GAMEPAD_GAMEPAD_PLATFORM_DATA_FETCHER_WIN_H_
