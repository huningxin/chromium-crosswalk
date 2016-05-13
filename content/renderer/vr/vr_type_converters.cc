// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/renderer/vr/vr_type_converters.h"

#include <string.h>

#include <algorithm>

using content::VRFieldOfViewPtr;
using content::VREyeParametersPtr;
using content::VRDisplayPtr;
using content::VRPosePtr;
using content::VRDisplayCapabilitiesPtr;
using content::VRStageParametersPtr;

namespace mojo {

namespace {
void MojoFloatArrayToFloatArray(const mojo::Array<float>& mojoArray, float* array) {
  for (int i = 0; i < mojoArray.size(); ++i) {
    array[i] = mojoArray[i];
  }
}
}

// static
blink::WebVRFieldOfView
TypeConverter<blink::WebVRFieldOfView, VRFieldOfViewPtr>::Convert(
    const VRFieldOfViewPtr& input) {
  blink::WebVRFieldOfView output;
  output.upDegrees = input->upDegrees;
  output.downDegrees = input->downDegrees;
  output.leftDegrees = input->leftDegrees;
  output.rightDegrees = input->rightDegrees;
  return output;
}

// static
blink::WebVREyeParameters
TypeConverter<blink::WebVREyeParameters, VREyeParametersPtr>::Convert(
    const VREyeParametersPtr& input) {
  blink::WebVREyeParameters output;
  output.fieldOfView =
      input->fieldOfView.To<blink::WebVRFieldOfView>();
  MojoFloatArrayToFloatArray(input->offset, &(output.offset[0]));
  output.renderWidth = input->renderWidth;
  output.renderHeight = input->renderHeight;
  return output;
}

// static
blink::WebVRDisplayCapabilities
TypeConverter<blink::WebVRDisplayCapabilities, VRDisplayCapabilitiesPtr>::Convert(
    const VRDisplayCapabilitiesPtr& input) {
  blink::WebVRDisplayCapabilities output;
  output.hasOrientation = input->hasOrientation;
  output.hasPosition = input->hasPosition;
  output.hasExternalDisplay = input->hasExternalDisplay;
  output.canPresent = input->canPresent;
  return output;
}

// static
blink::WebVRStageParameters
TypeConverter<blink::WebVRStageParameters, VRStageParametersPtr>::Convert(
    const VRStageParametersPtr& input) {
  blink::WebVRStageParameters output;
  MojoFloatArrayToFloatArray(input->standingTransform, &(output.standingTransform[0]));
  output.sizeX = input->sizeX;
  output.sizeZ = input->sizeZ;
  return output;
}

// static
blink::WebVRDisplay TypeConverter<blink::WebVRDisplay, VRDisplayPtr>::Convert(
    const VRDisplayPtr& input) {
  blink::WebVRDisplay output;
  memset(&output, 0, sizeof(blink::WebVRDisplay));

  output.index = input->index;
  output.displayName = blink::WebString::fromUTF8(input->displayName.data(),
                                                  input->displayName.size());
  output.compositorType = static_cast<blink::WebVRCompositorType>(input->compositorType);
  output.capabilities =
      input->capabilities.To<blink::WebVRDisplayCapabilities>();
  output.hasStageParameters = !input->stageParameters.is_null();
  if (output.hasStageParameters) {
      output.stageParameters =
          input->stageParameters.To<blink::WebVRStageParameters>();
  }
  output.leftEye =
      input->leftEye.To<blink::WebVREyeParameters>();
  output.rightEye =
      input->rightEye.To<blink::WebVREyeParameters>();
  return output;
}

// static
blink::WebVRPose
TypeConverter<blink::WebVRPose, VRPosePtr>::Convert(
    const VRPosePtr& input) {
  blink::WebVRPose output;
  output.timestamp = input->timestamp;
  output.flags = 0;

  if (!input->orientation.is_null()) {
    output.flags |= blink::WebVRPoseOrientation;
    MojoFloatArrayToFloatArray(input->orientation, &(output.orientation[0]));
  }
  if (!input->position.is_null()) {
    output.flags |= blink::WebVRPosePosition;
    MojoFloatArrayToFloatArray(input->position, &(output.position[0]));
  }
  if (!input->angularVelocity.is_null()) {
    output.flags |= blink::WebVRPoseAngularVelocity;
    MojoFloatArrayToFloatArray(input->angularVelocity, &(output.angularVelocity[0]));
  }
  if (!input->linearVelocity.is_null()) {
    output.flags |= blink::WebVRPoseLinearVelocity;
    MojoFloatArrayToFloatArray(input->linearVelocity, &(output.linearVelocity[0]));
  }
  if (!input->angularAcceleration.is_null()) {
    output.flags |= blink::WebVRPoseAngularAcceleration;
    MojoFloatArrayToFloatArray(input->angularAcceleration, &(output.angularAcceleration[0]));
  }
  if (!input->linearAcceleration.is_null()) {
    output.flags |= blink::WebVRPoseLinearAcceleration;
    MojoFloatArrayToFloatArray(input->linearAcceleration, &(output.linearAcceleration[0])); 
  }

  return output;
}

}  // namespace mojo
