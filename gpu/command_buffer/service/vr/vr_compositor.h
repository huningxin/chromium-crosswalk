// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GPU_COMMAND_BUFFER_SERVICE_VR_VR_COMPOSITOR_H_
#define GPU_COMMAND_BUFFER_SERVICE_VR_VR_COMPOSITOR_H_

#include "ui/gl/gl_bindings.h"

namespace gpu {

class VRCompositor {
 public:
  VRCompositor() { }
  virtual ~VRCompositor() { }

  virtual bool needsStateRestore() { return false; }

  virtual void SubmitFrame(GLuint frame_texture,
      GLfloat x, GLfloat y, GLfloat z, GLfloat w) = 0;

  virtual void OnSwapFrame() {};

  virtual void TextureBounds(GLuint eye, GLfloat x, GLfloat y,
    GLfloat width, GLfloat height) = 0;

  virtual void ResetPose() = 0;
};

}  // namespace gpu

#endif  // GPU_COMMAND_BUFFER_SERVICE_VR_VR_COMPOSITOR_H_
