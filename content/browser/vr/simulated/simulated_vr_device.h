// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_H
#define CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_H

#include "content/browser/vr/vr_device.h"

namespace content {

class SimulatedVRDevice : public VRDevice {
 public:
  SimulatedVRDevice(VRDeviceProvider* provider);
  ~SimulatedVRDevice() override;

  blink::mojom::VRDisplayPtr GetVRDevice() override;
  blink::mojom::VRPosePtr GetPose() override;
  void ResetPose() override;

 private:
  mojo::Array<float> lastQuat;
  mojo::Array<float> nextQuat;

  mojo::Array<float> lastPos;
  mojo::Array<float> nextPos;
  float t;
};

}  // namespace content

#endif  // CONTENT_BROWSER_VR_SIMULATED_VR_DEVICE_H
