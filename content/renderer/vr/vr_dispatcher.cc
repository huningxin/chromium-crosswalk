// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/renderer/vr/vr_dispatcher.h"

#include <stddef.h>

#include "content/public/common/service_registry.h"
#include "content/renderer/vr/vr_type_converters.h"

namespace content {

VRDispatcher::VRDispatcher(ServiceRegistry* service_registry)
    : service_registry_(service_registry) {
}

VRDispatcher::~VRDispatcher() {
}

VRServicePtr& VRDispatcher::GetVRServicePtr() {
  if (!vr_service_) {
    service_registry_->ConnectToRemoteService(mojo::GetProxy(&vr_service_));
  }
  return vr_service_;
}

void VRDispatcher::getDisplays(blink::WebVRGetDisplaysCallback* callback) {
  int request_id = pending_requests_.Add(callback);
  GetVRServicePtr()->GetDisplays(base::Bind(&VRDispatcher::OnGetDisplays,
                                            base::Unretained(this), request_id));
}

void VRDispatcher::getPose(unsigned int index,
                           blink::WebVRPose& pose) {
  GetVRServicePtr()->GetPose(
      index,
      base::Bind(&VRDispatcher::OnGetPose, base::Unretained(&pose)));

  // This call needs to return results synchronously in order to be useful and
  // provide the lowest latency results possible.
  GetVRServicePtr().WaitForIncomingResponse();
}

void VRDispatcher::resetPose(unsigned int index) {
  GetVRServicePtr()->ResetPose(index);
}

void VRDispatcher::OnGetDisplays(int request_id,
                                 const mojo::Array<VRDisplayPtr>& displays) {
  blink::WebVector<blink::WebVRDisplay> web_displays(displays.size());

  blink::WebVRGetDisplaysCallback* callback =
      pending_requests_.Lookup(request_id);
  if (!callback)
    return;

  for (size_t i = 0; i < displays.size(); ++i) {
    web_displays[i] = displays[i].To<blink::WebVRDisplay>();
  }

  callback->onSuccess(web_displays);
  pending_requests_.Remove(request_id);
}

void VRDispatcher::OnGetPose(blink::WebVRPose* pose,
                             const VRPosePtr& mojo_pose) {
  *pose = mojo_pose.To<blink::WebVRPose>();
}

}  // namespace content
