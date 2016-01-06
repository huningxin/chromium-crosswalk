// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/simulated/simulated_vr_device.h"

#include <algorithm>
#include <math.h>

#include "base/strings/stringprintf.h"
#include "base/time/time.h"

namespace content {

namespace {

const float degToRad = 3.14159265358979323846f / 180.0f;

mojo::Array<float> randomPos() {
  mojo::Array<float> out = mojo::Array<float>::New(3);

  out[0] = (static_cast<float>(rand() % 10) / 10.0f) - 0.5f;
  out[1] = (static_cast<float>(rand() % 10) / 10.0f) - 0.5f;
  out[2] = (static_cast<float>(rand() % 10) / 10.0f) - 0.5f;

  return out;
}

mojo::Array<float> lerpPos(const mojo::Array<float>& a,
                           const mojo::Array<float>& b,
                           float t) {
  mojo::Array<float> out = mojo::Array<float>::New(3);

  out[0] = a[0] + t * (b[0] - a[0]);
  out[1] = a[1] + t * (b[1] - a[1]);
  out[2] = a[2] + t * (b[2] - a[2]);

  return out;
}

float randomAngle(float min, float max) {
  float range = (max - min) / 360.0f;
  float deg = min + (static_cast<float>(rand() % 360) * range);
  return deg * degToRad;
};

// Generates a random orientation that tries to be a semi-realistic head pose
// (So no upside down views or anything)
mojo::Array<float> randomQuat() {
  mojo::Array<float> out = mojo::Array<float>::New(4);
  out[3] = 1.0;

  float ax, ay, az, aw;
  float bx, by, bw;
  float rad;

  // Rotate on Y axis
  rad = randomAngle(-80.0f, 80.0f);
  by = sinf(rad);
  bw = cosf(rad);
  ax = out[0]; ay = out[1]; az = out[2]; aw = out[3];
  out[0] = ax * bw - az * by;
  out[1] = ay * bw + aw * by;
  out[2] = az * bw + ax * by;
  out[3] = aw * bw - ay * by;

  // Rotate on X axis (Doesn't do quite what you'd think)
  rad = randomAngle(-20.0f, 20.0f);
  bx = sinf(rad);
  bw = cosf(rad);
  ax = out[0]; ay = out[1]; az = out[2]; aw = out[3];
  out[0] = ax * bw + aw * bx;
  out[1] = ay * bw + az * bx;
  out[2] = az * bw - ay * bx;
  out[3] = aw * bw - ax * bx;

  // Normalize
  float len = out[0]*out[0] + out[1]*out[1] + out[2]*out[2] + out[3]*out[3];
  if (len > 0.0f) {
      len = 1.0f / sqrtf(len);
      out[0] *= len;
      out[1] *= len;
      out[2] *= len;
      out[3] *= len;
  }

  return out;
}

mojo::Array<float> slerpQuat(const mojo::Array<float>& a,
                             const mojo::Array<float>& b,
                             float t) {
  mojo::Array<float> out = mojo::Array<float>::New(4);

  float ax = a[0], ay = a[1], az = a[2], aw = a[3];
  float bx = b[0], by = b[1], bz = b[2], bw = b[3];
  float omega, sinom, scale0, scale1;

  // calc cosine
  float cosom = ax * bx + ay * by + az * bz + aw * bw;
  // adjust signs (if necessary)
  if ( cosom < 0.0 ) {
      cosom = -cosom;
      bx = - bx;
      by = - by;
      bz = - bz;
      bw = - bw;
  }
  // calculate coefficients
  if ( (1.0f - cosom) > 0.00001f ) {
      // standard case (slerp)
      omega  = acos(cosom);
      sinom  = sinf(omega);
      scale0 = sinf((1.0f - t) * omega) / sinom;
      scale1 = sinf(t * omega) / sinom;
  } else {
      // "from" and "to" quaternions are very close
      //  ... so we can do a linear interpolation
      scale0 = 1.0f - t;
      scale1 = t;
  }
  // calculate final values
  out[0] = scale0 * ax + scale1 * bx;
  out[1] = scale0 * ay + scale1 * by;
  out[2] = scale0 * az + scale1 * bz;
  out[3] = scale0 * aw + scale1 * bw;

  return out;
}

} // namespace

SimulatedVRDevice::SimulatedVRDevice(VRDeviceProvider* provider)
    : VRDevice(provider)
    , t(0.0f)
{
  lastQuat = mojo::Array<float>::New(4);
  lastQuat[3] = 1.0;
  nextQuat = randomQuat();

  lastPos = mojo::Array<float>::New(3);
  nextPos = randomPos();
}

SimulatedVRDevice::~SimulatedVRDevice() {
}

mojom::VRDisplayPtr SimulatedVRDevice::GetVRDevice() {
  mojom::VRDisplayPtr device = mojom::VRDisplay::New();

  device->displayName = "Simulated VR Device";
  device->compositorType = mojom::VRDisplay::CompositorType::NONE;

  device->capabilities = mojom::VRDisplayCapabilities::New();
  device->capabilities->hasOrientation = false;
  device->capabilities->hasPosition = false;
  device->capabilities->hasExternalDisplay = false;

  device->leftEye = mojom::VREyeParameters::New();
  device->rightEye = mojom::VREyeParameters::New();
  mojom::VREyeParametersPtr& leftEye = device->leftEye;
  mojom::VREyeParametersPtr& rightEye = device->rightEye;

  leftEye->fieldOfView = mojom::VRFieldOfView::New();
  leftEye->fieldOfView->upDegrees = 45.0f;
  leftEye->fieldOfView->downDegrees = 45.0f;
  leftEye->fieldOfView->leftDegrees = 45.0f;
  leftEye->fieldOfView->rightDegrees = 35.0f;

  rightEye->fieldOfView = mojom::VRFieldOfView::New();
  rightEye->fieldOfView->upDegrees = 45.0f;
  rightEye->fieldOfView->downDegrees = 45.0f;
  rightEye->fieldOfView->leftDegrees = 35.0f;
  rightEye->fieldOfView->rightDegrees = 45.0f;

  leftEye->offset = mojo::Array<float>::New(3);
  leftEye->offset[0] = -0.03f;
  leftEye->offset[1] = 0.0f;
  leftEye->offset[2] = 0.0f;

  rightEye->offset = mojo::Array<float>::New(3);
  rightEye->offset[0] = 0.03f;
  rightEye->offset[1] = 0.0f;
  rightEye->offset[2] = 0.0f;

  leftEye->renderWidth = 640;
  leftEye->renderHeight = 720;

  rightEye->renderWidth = 640;
  rightEye->renderHeight = 720;

  device->stageParameters = mojom::VRStageParameters::New();
  device->stageParameters->sizeX = 1.5;
  device->stageParameters->sizeZ = 1.5;
  device->stageParameters->standingTransform = mojo::Array<float>::New(16);
  mojo::Array<float>& transform =
      device->stageParameters->standingTransform;
  transform[0] = 1.0f;
  transform[1] = 0.0f;
  transform[2] = 0.0f;
  transform[3] = 0.0f;
  transform[4] = 0.0f;
  transform[5] = 1.0f;
  transform[6] = 0.0f;
  transform[7] = 0.0f;
  transform[8] = 0.0f;
  transform[9] = 0.0f;
  transform[10] = 1.0f;
  transform[11] = 0.0f;
  transform[12] = 0.0f;
  transform[13] = 1.6f; // 1.6 meters height
  transform[14] = 0.0f;
  transform[15] = 1.0f;

  return device;
}

mojom::VRPosePtr SimulatedVRDevice::GetPose() {
  mojom::VRPosePtr state = mojom::VRPose::New();

  state->timestamp = base::Time::Now().ToJsTime();

  t += 0.01f;

  if (t >= 1.0f) {
    lastQuat = nextQuat.Clone();
    nextQuat = randomQuat();

    lastPos = nextPos.Clone();
    nextPos = randomPos();

    t -= 1.0f;
  }

  state->orientation = slerpQuat(lastQuat, nextQuat, t);
  state->position = lerpPos(lastPos, nextPos, t);

  return state;
}

void SimulatedVRDevice::ResetPose() {

}

}  // namespace content
