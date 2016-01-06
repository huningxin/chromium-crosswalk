// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/vr/VRPose.h"

namespace blink {

namespace {

DOMFloat32Array* mojoArrayToFloat32Array(const mojo::WTFArray<float>& vec)
{
    if (!vec.is_null()) {
        return DOMFloat32Array::create(&(vec.front()), vec.size());
    }
    return nullptr;
}

DOMFloat32Array* vecToFloat32Array(const WebGamepadVector& vec, int elements)
{
    if (!vec.isNull) {
        DOMFloat32Array* out = DOMFloat32Array::create(elements);
        out->data()[0] = vec.x;
        out->data()[1] = vec.y;
        out->data()[2] = vec.z;
        if (elements >= 4)
            out->data()[3] = vec.w;
        return out;
    }
    return nullptr;
}

} // namespace

VRPose::VRPose()
    : m_timeStamp(0.0)
{
}

void VRPose::setPose(const mojom::blink::VRPosePtr& state)
{
    if (state.is_null())
        return;

    m_timeStamp = state->timestamp;
    m_orientation = mojoArrayToFloat32Array(state->orientation);
    m_position = mojoArrayToFloat32Array(state->position);
    m_angularVelocity = mojoArrayToFloat32Array(state->angularVelocity);
    m_linearVelocity = mojoArrayToFloat32Array(state->linearVelocity);
    m_angularAcceleration = mojoArrayToFloat32Array(state->angularAcceleration);
    m_linearAcceleration = mojoArrayToFloat32Array(state->linearAcceleration);
}

void VRPose::setPose(const WebGamepadPose &state)
{
    m_timeStamp = 0;
    m_orientation = vecToFloat32Array(state.orientation, 4);
    m_position = vecToFloat32Array(state.position, 3);
    m_angularVelocity = nullptr;
    m_linearVelocity = nullptr;
    m_angularAcceleration = nullptr;
    m_linearAcceleration = nullptr;
}

DEFINE_TRACE(VRPose)
{
    visitor->trace(m_orientation);
    visitor->trace(m_position);
    visitor->trace(m_angularVelocity);
    visitor->trace(m_linearVelocity);
    visitor->trace(m_angularAcceleration);
    visitor->trace(m_linearAcceleration);
}

} // namespace blink
