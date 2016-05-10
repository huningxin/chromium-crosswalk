// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CHROME_CONTENT_RENDERER_VR_VR_TYPE_CONVERTERS_H_
#define CHROME_CONTENT_RENDERER_VR_VR_TYPE_CONVERTERS_H_

#include "content/common/vr_service.mojom.h"
#include "mojo/common/common_type_converters.h"
#include "third_party/WebKit/public/platform/modules/vr/WebVR.h"

namespace mojo {

// Type/enum conversions from WebVR data types to Mojo data types
// and vice versa.

template <>
struct TypeConverter<blink::WebVRFieldOfView, content::VRFieldOfViewPtr> {
  static blink::WebVRFieldOfView Convert(
      const content::VRFieldOfViewPtr& input);
};

template <>
struct TypeConverter<blink::WebVREyeParameters, content::VREyeParametersPtr> {
  static blink::WebVREyeParameters Convert(
      const content::VREyeParametersPtr& input);
};

template <>
struct TypeConverter<blink::WebVRDisplayCapabilities, content::VRDisplayCapabilitiesPtr> {
  static blink::WebVRDisplayCapabilities Convert(
    const content::VRDisplayCapabilitiesPtr& input);
};

template <>
struct TypeConverter<blink::WebVRStageParameters, content::VRStageParametersPtr> {
  static blink::WebVRStageParameters Convert(
    const content::VRStageParametersPtr& input);
};

template <>
struct TypeConverter<blink::WebVRDisplay, content::VRDisplayPtr> {
  static blink::WebVRDisplay Convert(const content::VRDisplayPtr& input);
};

template <>
struct TypeConverter<blink::WebVRPose, content::VRPosePtr> {
  static blink::WebVRPose Convert(
      const content::VRPosePtr& input);
};

}  // namespace mojo

#endif  // CHROME_CONTENT_RENDERER_VR_VR_TYPE_CONVERTERS_H_