// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef WebVR_h
#define WebVR_h

#include "public/platform/WebCommon.h"
#include "public/platform/WebString.h"

#if BLINK_IMPLEMENTATION
#include "wtf/Assertions.h"
#endif

namespace blink {

// A field of view, given by 4 degrees describing the view from a center point.
struct WebVRFieldOfView {
    float upDegrees;
    float downDegrees;
    float leftDegrees;
    float rightDegrees;
};

// Bit flags to indicate which fields of an WebHMDPose are valid.
enum WebVRPoseFlags {
    WebVRPoseOrientation = 1 << 1,
    WebVRPosePosition = 1 << 2,
    WebVRPoseAngularVelocity = 1 << 3,
    WebVRPoseLinearVelocity = 1 << 4,
    WebVRPoseAngularAcceleration = 1 << 5,
    WebVRPoseLinearAcceleration = 1 << 6,
    WebVRPoseComplete = (1 << 7) - 1 // All previous states combined.
};

// A bitfield of WebVRPoseFlags.
typedef int WebVRPoseMask;

// A display's position, orientation, velocity, and acceleration state at the
// given timestamp.
struct WebVRPose {
    double timestamp;
    WebVRPoseMask flags;
    float orientation[4];
    float position[3];
    float angularVelocity[3];
    float linearVelocity[3];
    float angularAcceleration[3];
    float linearAcceleration[3];
};

struct WebVRDisplayCapabilities {
    bool hasOrientation;
    bool hasPosition;
    bool hasExternalDisplay;
    bool canPresent;
};

// Information about the optical properties for an eye in a Display.
struct WebVREyeParameters {
    WebVRFieldOfView fieldOfView;
    float offset[3];
    unsigned renderWidth;
    unsigned renderHeight;
};

struct WebVRStageParameters {
    float standingTransform[16];
    float sizeX;
    float sizeZ;
};

enum WebVRCompositorType {
    VR_COMPOSITOR_NONE,
    VR_COMPOSITOR_CARDBOARD,
    VR_COMPOSITOR_OCULUS,
    VR_COMPOSITOR_OPENVR
};

// Describes a single VR hardware unit. May describe multiple capabilities,
// such as position sensors or head mounted display metrics.
struct WebVRDisplay {
    // Index for this hardware unit.
    unsigned index;
    // Friendly display name.
    WebString displayName;
    WebVRCompositorType compositorType;
    WebVRDisplayCapabilities capabilities;
    bool hasStageParameters;
    WebVRStageParameters stageParameters;
    WebVREyeParameters leftEye;
    WebVREyeParameters rightEye;
};

}

#endif // WebVR_h
