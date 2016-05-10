// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef WebVRClient_h
#define WebVRClient_h

#include "public/platform/WebCallbacks.h"
#include "public/platform/WebVector.h"
#include "public/platform/modules/vr/WebVR.h"

namespace blink {

// Success and failure callbacks for getDevices.
using WebVRGetDisplaysCallback = WebCallbacks<const WebVector<WebVRDisplay>&, void>;

// Client handling VR device communication for a given WebFrame.
class WebVRClient {
public:
    virtual ~WebVRClient() { }

    virtual void getDisplays(WebVRGetDisplaysCallback*) = 0;

    virtual void getPose(unsigned index, blink::WebVRPose& pose) = 0;

    virtual void resetPose(unsigned index) = 0;
};

} // namespace blink

#endif // WebVRClient_h
