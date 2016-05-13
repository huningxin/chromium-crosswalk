// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gpu/command_buffer/service/vr/openvr/open_vr_compositor.h"

#include "base/logging.h"
#include "ui/gl/gl_bindings.h"

namespace gpu {

OpenVRCompositor::OpenVRCompositor()
    : VRCompositor()
    , initialized_(false)
    , vr_system_(nullptr)
    , compositor_(nullptr) {
  Initialize();
}

OpenVRCompositor::~OpenVRCompositor() {
  if (compositor_) {
    compositor_->ClearLastSubmittedFrame();
    vr::VR_Shutdown();
  }
}

void OpenVRCompositor::SubmitFrame(GLuint frame_texture,
    GLfloat x, GLfloat y, GLfloat z, GLfloat w) {
  if (!initialized_)
    return;

  vr::Texture_t vr_texture;
  vr_texture.handle = reinterpret_cast<void*>(frame_texture);
  vr_texture.eType = vr::API_OpenGL;
  vr_texture.eColorSpace = vr::ColorSpace_Linear;

  compositor_->Submit(vr::Eye_Left, &vr_texture, &left_bounds_);
  compositor_->Submit(vr::Eye_Right, &vr_texture, &right_bounds_);

  compositor_->PostPresentHandoff();
}

void OpenVRCompositor::OnSwapFrame() {
  if (!initialized_)
    return;

  vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
  compositor_->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, NULL, 0);
}

void OpenVRCompositor::TextureBounds(GLuint eye, GLfloat x, GLfloat y,
    GLfloat width, GLfloat height) {
  if (eye == 0) { // Left Eye
    left_bounds_.uMin = x;
    left_bounds_.vMin = y;
    left_bounds_.uMax = x + width;
    left_bounds_.vMax = y + height;
  } else if (eye == 1) { // Right eye
    right_bounds_.uMin = x;
    right_bounds_.vMin = y;
    right_bounds_.uMax = x + width;
    right_bounds_.vMax = y + height;
  }
}

void OpenVRCompositor::ResetPose() {
  // Nothing to do for OpenVR. It syncs pose resets across processes.
}

void OpenVRCompositor::Initialize() {
  if (!initialized_) {
    vr::EVRInitError error = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&error, vr::VRApplication_Scene);

    if (error != vr::VRInitError_None) {
      LOG(ERROR) << "Error creating OpenVR System: " << vr::VR_GetVRInitErrorAsEnglishDescription(error);
      vr_system_ = nullptr;
      return;
    }

    compositor_ = static_cast<vr::IVRCompositor*>(
        vr::VR_GetGenericInterface(vr::IVRCompositor_Version, &error));

    if (error != vr::VRInitError_None) {
      LOG(ERROR) << "Error creating OpenVR Compositor: " << vr::VR_GetVRInitErrorAsEnglishDescription(error);
      compositor_ = nullptr;
      return;
    }

    left_bounds_.uMin = 0.0f;
    left_bounds_.vMin = 0.0f;
    left_bounds_.uMax = 0.5f;
    left_bounds_.vMax = 1.0f;

    right_bounds_.uMin = 0.5f;
    right_bounds_.vMin = 0.0f;
    right_bounds_.uMax = 1.0f;
    right_bounds_.vMax = 1.0f;

    initialized_ = true;
  }
}

}  // namespace gpu
