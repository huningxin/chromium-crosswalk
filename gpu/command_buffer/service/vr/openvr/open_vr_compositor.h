// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GPU_COMMAND_BUFFER_SERVICE_VR_OPEN_VR_COMPOSITOR_H_
#define GPU_COMMAND_BUFFER_SERVICE_VR_OPEN_VR_COMPOSITOR_H_

#include "gpu/command_buffer/service/vr/vr_compositor.h"
#include "third_party/openvr/openvr/headers/openvr.h"
#include "ui/gl/gl_bindings.h"

namespace gpu {

class OpenVRCompositor : public VRCompositor {
 public:
  OpenVRCompositor();
  ~OpenVRCompositor() override;

  void SubmitFrame(GLuint frame_texture,
      GLfloat x, GLfloat y, GLfloat z, GLfloat w) override;

  void OnSwapFrame() override;

  void TextureBounds(GLuint eye, GLfloat x, GLfloat y,
    GLfloat width, GLfloat height) override;

  void ResetPose() override;

 private:
  void Initialize();
  void WaitGetPoses();

  bool initialized_;
  vr::IVRSystem* vr_system_;
  vr::IVRCompositor* compositor_;
  vr::VRTextureBounds_t left_bounds_;
  vr::VRTextureBounds_t right_bounds_;
};

}  // namespace gpu

#endif  // GPU_COMMAND_BUFFER_SERVICE_VR_OPEN_VR_COMPOSITOR_H_
