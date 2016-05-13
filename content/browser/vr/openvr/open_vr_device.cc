// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/vr/openvr/open_vr_device.h"

#include <algorithm>
#include <math.h>

#include "base/logging.h"
#include "base/strings/stringprintf.h"
#include "base/time/time.h"
#include "third_party/openvr/openvr/headers/openvr.h"
#include "ui/gfx/transform.h"
#include "ui/gfx/transform_util.h"

namespace content {

namespace {

const float radToDeg = 180.0f / 3.14159265358979323846f;

blink::mojom::VRFieldOfViewPtr openVRFovToWebVR(vr::IVRSystem* vr_system,
                                         vr::Hmd_Eye eye) {
  blink::mojom::VRFieldOfViewPtr out = blink::mojom::VRFieldOfView::New();

  float upTan, downTan, leftTan, rightTan;
  vr_system->GetProjectionRaw(eye, &leftTan, &rightTan, &upTan, &downTan);
  out->upDegrees = -(atanf(upTan) * radToDeg);
  out->downDegrees = atanf(downTan) * radToDeg;
  out->leftDegrees = -(atanf(leftTan) * radToDeg);
  out->rightDegrees = atanf(rightTan) * radToDeg;
  return out;
}

mojo::Array<float> HmdVector3ToWebVR(const vr::HmdVector3_t& vec) {
  mojo::Array<float> out = mojo::Array<float>::New(3);
  out[0] = vec.v[0];
  out[1] = vec.v[1];
  out[2] = vec.v[2];
  return out;
}

} // namespace

OpenVRDevice::OpenVRDevice(VRDeviceProvider* provider,
    vr::IVRSystem* vr_system,
    vr::TrackedDeviceIndex_t device_index)
    : VRDevice(provider)
    , vr_system_(vr_system)
    , device_index_(device_index)
{
}

OpenVRDevice::~OpenVRDevice() {
}

std::string OpenVRDevice::getOpenVRString(vr::TrackedDeviceProperty prop) {
  std::string out;

  vr::TrackedPropertyError error = vr::TrackedProp_Success;
  char openvr_string[vr::k_unTrackingStringSize];
  vr_system_->GetStringTrackedDeviceProperty(
    device_index_,
    prop,
    openvr_string,
    vr::k_unTrackingStringSize,
    &error);

  if (error == vr::TrackedProp_Success)
    out = openvr_string;

  return out;
}

blink::mojom::VRDisplayPtr OpenVRDevice::GetVRDevice() {
  blink::mojom::VRDisplayPtr device = blink::mojom::VRDisplay::New();

  device->displayName = getOpenVRString(vr::Prop_ManufacturerName_String) + " "
                     + getOpenVRString(vr::Prop_ModelNumber_String);

  vr::TrackedDeviceClass device_class = vr_system_->GetTrackedDeviceClass(
      device_index_);

  device->compositorType = blink::mojom::VRDisplay::CompositorType::OPENVR;

  // Only populate the HMD info if this is actually an HMD
  if (device_class == vr::TrackedDeviceClass_HMD) {
    device->capabilities = blink::mojom::VRDisplayCapabilities::New();
    device->capabilities->hasOrientation = true;
    device->capabilities->hasPosition = true;
    device->capabilities->hasExternalDisplay = true;
    device->capabilities->canPresent = true;

    device->leftEye = blink::mojom::VREyeParameters::New();
    device->rightEye = blink::mojom::VREyeParameters::New();
    blink::mojom::VREyeParametersPtr& leftEye = device->leftEye;
    blink::mojom::VREyeParametersPtr& rightEye = device->rightEye;

    leftEye->fieldOfView = openVRFovToWebVR(vr_system_, vr::Eye_Left);
    rightEye->fieldOfView = openVRFovToWebVR(vr_system_, vr::Eye_Right);

    vr::TrackedPropertyError error = vr::TrackedProp_Success;
    float ipd = vr_system_->GetFloatTrackedDeviceProperty(device_index_,
        vr::Prop_UserIpdMeters_Float, &error);
    if (error != vr::TrackedProp_Success)
      ipd = 0.06f; // Default average IPD

    leftEye->offset = mojo::Array<float>::New(3);
    leftEye->offset[0] = -ipd * 0.5;
    leftEye->offset[1] = 0.0f;
    leftEye->offset[2] = 0.0f;

    rightEye->offset = mojo::Array<float>::New(3);
    rightEye->offset[0] = ipd * 0.5;
    rightEye->offset[1] = 0.0;
    rightEye->offset[2] = 0.0;

    uint32_t width, height;
    // This is the render size per-eye. OpenVR docs don't make that clear.
    vr_system_->GetRecommendedRenderTargetSize(&width, &height);

    leftEye->renderWidth = width;
    leftEye->renderHeight = height;

    rightEye->renderWidth = width;
    rightEye->renderHeight = height;

    device->stageParameters = blink::mojom::VRStageParameters::New();
    vr::HmdMatrix34_t mat =
        vr_system_->GetSeatedZeroPoseToStandingAbsoluteTrackingPose();
    device->stageParameters->standingTransform = mojo::Array<float>::New(16);
    mojo::Array<float>& transform =
        device->stageParameters->standingTransform;
    transform[0] = mat.m[0][0];
    transform[1] = mat.m[1][0];
    transform[2] = mat.m[2][0];
    transform[3] = 0.0f;
    transform[4] = mat.m[0][1];
    transform[5] = mat.m[1][1];
    transform[6] = mat.m[2][1];
    transform[7] = 0.0f;
    transform[8] = mat.m[0][2];
    transform[9] = mat.m[1][2];
    transform[10] = mat.m[2][2];
    transform[11] = 0.0f;
    transform[12] = mat.m[0][3];
    transform[13] = mat.m[1][3];
    transform[14] = mat.m[2][3];
    transform[15] = 1.0f;

    // Values needed for room-scale experiences
    vr::IVRChaperone* chaperone = vr::VRChaperone();
    if (chaperone) {
      chaperone->GetPlayAreaSize(&device->stageParameters->sizeX,
                                  &device->stageParameters->sizeZ);
    } else {
      device->stageParameters->sizeX = 0.0f;
      device->stageParameters->sizeZ = 0.0f;
    }
  }

  return device;
}

blink::mojom::VRPosePtr OpenVRDevice::GetPose() {
  blink::mojom::VRPosePtr state = blink::mojom::VRPose::New();

  state->timestamp = base::Time::Now().ToJsTime();

  vr::TrackedDevicePose_t tracked_devices_poses[vr::k_unMaxTrackedDeviceCount];
  vr_system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated,
      0.04f, tracked_devices_poses, vr::k_unMaxTrackedDeviceCount);

  const vr::TrackedDevicePose_t &pose = tracked_devices_poses[device_index_];
  if (pose.bPoseIsValid) {
    const vr::HmdMatrix34_t& mat = pose.mDeviceToAbsoluteTracking;
    gfx::Transform transform(
        mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
        mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
        mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
        0, 0, 0, 1);

    gfx::DecomposedTransform decomposed_transform;
    gfx::DecomposeTransform(&decomposed_transform, transform);

    state->orientation = mojo::Array<float>::New(4);
    state->orientation[0] = decomposed_transform.quaternion[0];
    state->orientation[1] = decomposed_transform.quaternion[1];
    state->orientation[2] = decomposed_transform.quaternion[2];
    state->orientation[3] = decomposed_transform.quaternion[3];

    state->position = mojo::Array<float>::New(3);
    state->position[0] = decomposed_transform.translate[0];
    state->position[1] = decomposed_transform.translate[1];
    state->position[2] = decomposed_transform.translate[2];

    state->angularVelocity = HmdVector3ToWebVR(pose.vAngularVelocity);
    state->linearVelocity = HmdVector3ToWebVR(pose.vVelocity);
  }

  return state;
}

void OpenVRDevice::ResetPose() {
  vr_system_->ResetSeatedZeroPose();
}

}  // namespace content
