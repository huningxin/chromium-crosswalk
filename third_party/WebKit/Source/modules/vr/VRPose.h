// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VRPose_h
#define VRPose_h

#include "bindings/core/v8/ScriptWrappable.h"
#include "core/dom/DOMTypedArray.h"
#include "platform/heap/Handle.h"
#include "public/platform/modules/vr/WebVR.h"
#include "wtf/Forward.h"

namespace blink {

class VRPose final : public GarbageCollectedFinalized<VRPose>, public ScriptWrappable {
    DEFINE_WRAPPERTYPEINFO();
public:
    static VRPose* create()
    {
        return new VRPose();
    }

    double timeStamp() const { return m_timeStamp; }
    DOMFloat32Array* orientation() const { return m_orientation.get(); }
    DOMFloat32Array* position() const { return m_position.get(); }
    DOMFloat32Array* angularVelocity() const { return m_angularVelocity.get(); }
    DOMFloat32Array* linearVelocity() const { return m_linearVelocity.get(); }
    DOMFloat32Array* angularAcceleration() const { return m_angularAcceleration.get(); }
    DOMFloat32Array* linearAcceleration() const { return m_linearAcceleration.get(); }

    void setPose(const WebHMDSensorState&);

    DEFINE_INLINE_TRACE() { }

private:
    VRPose();

    double m_timeStamp;
    RefPtr<DOMFloat32Array> m_orientation;
    RefPtr<DOMFloat32Array> m_position;
    RefPtr<DOMFloat32Array> m_angularVelocity;
    RefPtr<DOMFloat32Array> m_linearVelocity;
    RefPtr<DOMFloat32Array> m_angularAcceleration;
    RefPtr<DOMFloat32Array> m_linearAcceleration;
};

} // namespace blink

#endif // VRPose_h
