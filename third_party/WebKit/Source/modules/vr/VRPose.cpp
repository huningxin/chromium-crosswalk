// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/vr/VRPose.h"

namespace blink {

namespace {

DOMFloat32Array* arrayToFloat32Array(const float* array, int length, bool valid)
{
    if (valid) {
        DOMFloat32Array* out = DOMFloat32Array::create(array, length).get();
        return out;
    }
    return nullptr;
}

} // namespace

VRPose::VRPose()
    : m_timeStamp(0.0)
{
}

void VRPose::setPose(const WebVRPose &pose)
{
    m_timeStamp = pose.timestamp;
    m_orientation = arrayToFloat32Array(&(pose.orientation[0]), 4, pose.flags & WebVRPoseOrientation);
    m_position = arrayToFloat32Array(&(pose.position[0]), 3, pose.flags & WebVRPosePosition);
    m_angularVelocity = arrayToFloat32Array(&(pose.angularVelocity[0]), 3, pose.flags & WebVRPoseAngularVelocity);
    m_linearVelocity = arrayToFloat32Array(&(pose.linearVelocity[0]), 3, pose.flags & WebVRPoseLinearVelocity);
    m_angularAcceleration = arrayToFloat32Array(&(pose.angularAcceleration[0]), 3, pose.flags & WebVRPoseAngularAcceleration);
    m_linearAcceleration = arrayToFloat32Array(&(pose.linearAcceleration[0]), 3, pose.flags & WebVRPoseLinearAcceleration);
}

} // namespace blink
