// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/oculus/oculus_vr_device.h"

#include <algorithm>
#include <math.h>

#include "base/strings/stringprintf.h"
#include "base/time/time.h"
#include "third_party/libovr/LibOVR/Include/OVR_CAPI.h"

namespace content {

namespace {
const float radToDeg = 180.0f / 3.14159265358979323846f;
}

OculusVRDevice::OculusVRDevice(VRDeviceProvider* provider, ovrSession session)
    : VRDevice(provider)
    , session_(session)
{
}

OculusVRDevice::~OculusVRDevice() {
  if (session_) {
    ovr_Destroy(session_);
    session_ = NULL;
  }
}

blink::mojom::VRFieldOfViewPtr ovrFovPortToWebVR(const ovrFovPort& fov) {
  blink::mojom::VRFieldOfViewPtr out = blink::mojom::VRFieldOfView::New();
  out->upDegrees = atanf(fov.UpTan) * radToDeg;
  out->downDegrees = atanf(fov.DownTan) * radToDeg;
  out->leftDegrees = atanf(fov.LeftTan) * radToDeg;
  out->rightDegrees = atanf(fov.RightTan) * radToDeg;
  return out;
}

mojo::Array<float> ovrQuatfToWebVR(const ovrQuatf& quat) {
  mojo::Array<float> out = mojo::Array<float>::New(4);
  out[0] = quat.x;
  out[1] = quat.y;
  out[2] = quat.z;
  out[3] = quat.w;
  return out;
}

mojo::Array<float> ovrVector3fToWebVR(const ovrVector3f& vec) {
  mojo::Array<float> out = mojo::Array<float>::New(3);
  out[0] = vec.x;
  out[1] = vec.y;
  out[2] = vec.z;
  return out;
}

blink::mojom::VRDisplayPtr OculusVRDevice::GetVRDevice() {
  blink::mojom::VRDisplayPtr device = blink::mojom::VRDisplay::New();

  ovrHmdDesc session_desc = ovr_GetHmdDesc(session_);

  device->displayName = base::StringPrintf("%s, %s",
      session_desc.ProductName, session_desc.Manufacturer);

  device->compositorType = blink::mojom::VRDisplay::CompositorType::OCULUS;

  device->capabilities = blink::mojom::VRDisplayCapabilities::New();
  device->capabilities->hasOrientation = true;
  device->capabilities->hasPosition = true;
  device->capabilities->hasExternalDisplay = true;
  device->capabilities->canPresent = true;

  device->leftEye = blink::mojom::VREyeParameters::New();
  device->rightEye = blink::mojom::VREyeParameters::New();
  blink::mojom::VREyeParametersPtr& leftEye = device->leftEye;
  blink::mojom::VREyeParametersPtr& rightEye = device->rightEye;

  leftEye->fieldOfView = ovrFovPortToWebVR(
      session_desc.DefaultEyeFov[ovrEye_Left]);
  rightEye->fieldOfView = ovrFovPortToWebVR(
      session_desc.DefaultEyeFov[ovrEye_Right]);

  ovrEyeRenderDesc leftEyeDesc = ovr_GetRenderDesc(session_, ovrEye_Left,
      session_desc.DefaultEyeFov[ovrEye_Left]);
  ovrEyeRenderDesc rightEyeDesc = ovr_GetRenderDesc(session_, ovrEye_Right,
      session_desc.DefaultEyeFov[ovrEye_Right]);

  leftEye->offset = ovrVector3fToWebVR(leftEyeDesc.HmdToEyeOffset);
  rightEye->offset = ovrVector3fToWebVR(rightEyeDesc.HmdToEyeOffset);

  ovrSizei targetLeft = ovr_GetFovTextureSize(session_, ovrEye_Left,
      session_desc.DefaultEyeFov[ovrEye_Left], 1.0f);
  ovrSizei targetRight = ovr_GetFovTextureSize(session_, ovrEye_Right,
      session_desc.DefaultEyeFov[ovrEye_Right], 1.0f);

  leftEye->renderWidth = targetLeft.w;
  leftEye->renderHeight = targetLeft.h;

  rightEye->renderWidth = targetRight.w;
  rightEye->renderHeight = targetRight.h;

  device->stageParameters = blink::mojom::VRStageParameters::New();
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
  transform[13] = GetFloorHeight();
  transform[14] = 0.0f;
  transform[15] = 1.0f;

  device->stageParameters->sizeX = 0.0f;
  device->stageParameters->sizeZ = 0.0f;

  return device;
}

blink::mojom::VRPosePtr OculusVRDevice::GetPose() {
  blink::mojom::VRPosePtr pose = blink::mojom::VRPose::New();

  ovrTrackingState ovr_state = ovr_GetTrackingState(session_,
      ovr_GetTimeInSeconds(), true);

  ovrPoseStatef pose_state = ovr_state.HeadPose;

  pose->timestamp = base::Time::Now().ToJsTime();

  // TODO: ovr_state.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)
  pose->orientation = ovrQuatfToWebVR(pose_state.ThePose.Orientation);
  pose->position = ovrVector3fToWebVR(pose_state.ThePose.Position);
  pose->angularVelocity = ovrVector3fToWebVR(pose_state.AngularVelocity);
  pose->linearVelocity = ovrVector3fToWebVR(pose_state.LinearVelocity);
  pose->angularAcceleration = ovrVector3fToWebVR(
      pose_state.AngularAcceleration);
  pose->linearAcceleration = ovrVector3fToWebVR(pose_state.LinearAcceleration);

  return pose;
}

void OculusVRDevice::ResetPose() {
  ovr_RecenterTrackingOrigin(session_);
}

float OculusVRDevice::GetFloorHeight() {
  double query_time = ovr_GetTimeInSeconds();

  ovr_SetTrackingOriginType(session_, ovrTrackingOrigin_FloorLevel);
  ovrTrackingState ovr_state = ovr_GetTrackingState(session_, query_time, true);
  ovrVector3f floor_position = ovr_state.HeadPose.ThePose.Position;

  ovr_SetTrackingOriginType(session_, ovrTrackingOrigin_EyeLevel);

  return floor_position.y;
}

}  // namespace content
