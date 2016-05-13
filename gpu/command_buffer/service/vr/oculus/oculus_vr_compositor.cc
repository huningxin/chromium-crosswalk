// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gpu/command_buffer/service/vr/oculus/oculus_vr_compositor.h"

#include "base/logging.h"
#include "third_party/libovr/LibOVR/Include/OVR_CAPI_GL.h"
#include "ui/gl/gl_bindings.h"

#define SHADER(src)                     \
  "#ifdef GL_ES\n"                      \
  "precision mediump float;\n"          \
  "#define TexCoordPrecision mediump\n" \
  "#else\n"                             \
  "#define TexCoordPrecision\n"         \
  "#endif\n" #src

namespace gpu {

#define OCULUS_INIT_DEBUG 1

const char* vertex_shader_source = SHADER(
  attribute vec2 a_position;
  varying TexCoordPrecision vec2 v_uv;
  void main(void) {
    gl_Position = vec4(a_position, 0.0, 1.0);
    v_uv = a_position * vec2(0.5, 0.5) + vec2(0.5, 0.5);
  });

const char* fragment_shader_source = SHADER(
  uniform sampler2D u_sampler;
  varying TexCoordPrecision vec2 v_uv;
  void main(void) {
    gl_FragColor = texture2D(u_sampler, v_uv);
  });

void CompileShader(GLuint shader, const char* shader_source) {
  glShaderSource(shader, 1, &shader_source, 0);
  glCompileShader(shader);

  GLint compile_status;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
  if (GL_TRUE != compile_status)
    LOG(ERROR) << "OculusVRCompositor CompileShader: shader compilation failure.";
}

OculusVRCompositor::OculusVRCompositor()
    : VRCompositor()
    , initialized_(false)
    , session_(nullptr)
    , fail_count_(0) {
  Initialize();
}

OculusVRCompositor::~OculusVRCompositor() {
  if (initialized_) {
    glDeleteFramebuffersEXT(1, &framebuffer_);
    framebuffer_ = 0;

    ovr_DestroyTextureSwapChain(session_, swap_chain_);
    ovr_Destroy(session_);
  }
}

void OculusVRCompositor::SubmitFrame(GLuint frame_texture,
    GLfloat x, GLfloat y, GLfloat z, GLfloat w) {
  if (!initialized_ || fail_count_ >= 3)
    return;

  // Increment to use next texture, just before writing
  int curIndex;
  GLuint chainTexId;
  ovr_GetTextureSwapChainCurrentIndex(session_, swap_chain_, &curIndex);
  ovr_GetTextureSwapChainBufferGL(session_, swap_chain_, curIndex, &chainTexId);

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, framebuffer_);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0,
      GL_TEXTURE_2D, chainTexId, 0);

  glViewport(0, 0, width_, height_);

  glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glUseProgram(program_);

  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 8, 0);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, frame_texture);
  glUniform1i(sampler_location_, 0);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_CULL_FACE);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDepthMask(GL_FALSE);
  glDisable(GL_BLEND);
  glDisable(GL_SCISSOR_TEST);

  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

  glBindTexture(GL_TEXTURE_2D, 0);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

  ovr_CommitTextureSwapChain(session_, swap_chain_);

  ovrPosef head_pose;
  head_pose.Position.x = 0.0;
  head_pose.Position.y = 0.0;
  head_pose.Position.z = 0.0;

  head_pose.Orientation.x = x;
  head_pose.Orientation.y = y;
  head_pose.Orientation.z = z;
  head_pose.Orientation.w = w;

  layer.Header.Type  = ovrLayerType_EyeFov;
  layer.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft; // Because OpenGL.
  layer.SensorSampleTime = 0;

  layer.ColorTexture[ovrEye_Left] = swap_chain_;
  layer.Viewport[ovrEye_Left] = left_viewport_;
  layer.Fov[ovrEye_Left] = left_fov_;
  layer.RenderPose[ovrEye_Left] = head_pose;

  layer.ColorTexture[ovrEye_Right] = swap_chain_;
  layer.Viewport[ovrEye_Right] = right_viewport_;
  layer.Fov[ovrEye_Right] = right_fov_;
  layer.RenderPose[ovrEye_Right] = head_pose;
}

void OculusVRCompositor::OnSwapFrame() {
  ovrLayerHeader* layers = &layer.Header;
  if (OVR_FAILURE(ovr_SubmitFrame(session_, 0, nullptr, &layers, 1))) {
    LOG(ERROR) << "Failed to submit frame to Oculus HMD";
    fail_count_++;
    return;
  }
}

void OculusVRCompositor::TextureBounds(GLuint eye, GLfloat x, GLfloat y,
    GLfloat width, GLfloat height) {
  if (eye == 0) { // Left Eye
    left_viewport_.Pos.x = x * width_;
    left_viewport_.Pos.y = y * height_;
    left_viewport_.Size.w = width * width_;
    left_viewport_.Size.h = height * height_;
  } else if (eye == 1) { // Right eye
    right_viewport_.Pos.x = x * width_;
    right_viewport_.Pos.y = y * height_;
    right_viewport_.Size.w = width * width_;
    right_viewport_.Size.h = height * height_;
  }
}

void OculusVRCompositor::ResetPose() {
  ovr_RecenterTrackingOrigin(session_);
}

void OculusVRCompositor::CreateProgram() {
  program_ = glCreateProgram();
  GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  CompileShader(vertex_shader, vertex_shader_source);
  glAttachShader(program_, vertex_shader);

  GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  CompileShader(fragment_shader, fragment_shader_source);
  glAttachShader(program_, fragment_shader);

  glBindAttribLocation(program_, 0, "a_position");
  glLinkProgram(program_);

  GLint linked;
  glGetProgramiv(program_, GL_LINK_STATUS, &linked);
  if (!linked)
    LOG(ERROR) << "OculusVRCompositor::CreateProgram: program link failure.";

  sampler_location_ = glGetUniformLocation(program_, "u_sampler");
}

void OculusVRCompositorLog(uintptr_t userData, int level, const char* message) {
  LOG(INFO) << "Oculus GPU Log, Lvl " << level << ": " << message;
}

void OculusVRCompositor::Initialize() {
  if (!initialized_) {
    ovrInitParams params;
    params.ConnectionTimeoutMS = 0;

#if OCULUS_INIT_DEBUG
    params.Flags = ovrInit_Debug;
    params.LogCallback = OculusVRCompositorLog;
#else
    params.Flags = 0;
    params.LogCallback = nullptr;
#endif

    if (OVR_FAILURE(ovr_Initialize(&params))) {
      LOG(ERROR) << "Failed to initialize Oculus SDK";
      return;
    }

    ovrGraphicsLuid luid;
    if (OVR_FAILURE(ovr_Create(&session_, &luid))) {
      LOG(ERROR) << "Failed to create Oculus HMD";
      return;
    }

    ovrHmdDesc hmd_desc = ovr_GetHmdDesc(session_);

    left_fov_ = hmd_desc.DefaultEyeFov[ovrEye_Left];
    right_fov_ = hmd_desc.DefaultEyeFov[ovrEye_Right];

    ovrSizei left_size = ovr_GetFovTextureSize(session_, ovrEye_Left, left_fov_, 1);
    ovrSizei right_size = ovr_GetFovTextureSize(session_, ovrEye_Right, right_fov_, 1);

    width_ = left_size.w + right_size.w;
    height_ = left_size.h;

    left_viewport_.Pos.x = 0;
    left_viewport_.Pos.y = 0;
    left_viewport_.Size = left_size;

    right_viewport_.Pos.x = left_size.w;
    right_viewport_.Pos.y = 0;
    right_viewport_.Size = right_size;

    ovrTextureSwapChainDesc desc = {};
    desc.Type = ovrTexture_2D;
    desc.ArraySize = 1;
    desc.Width = width_;
    desc.Height = height_;
    desc.MipLevels = 1;
    desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    desc.SampleCount = 1;
    desc.StaticImage = ovrFalse;
    ovrResult result = ovr_CreateTextureSwapChainGL(session_, &desc, &swap_chain_);

    ovr_GetTextureSwapChainLength(session_, swap_chain_, &swap_chain_length_);

    if(OVR_SUCCESS(result)) {
      for (int i = 0; i < swap_chain_length_; ++i) {
        GLuint chainTexId;
        ovr_GetTextureSwapChainBufferGL(session_, swap_chain_, i, &chainTexId);
        glBindTexture(GL_TEXTURE_2D, chainTexId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      }
    }

    glGenBuffersARB(1, &buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, buffer_);
    const GLfloat kQuadVertices[] = {-1.0f, -1.0f,
                                      1.0f, -1.0f,
                                      1.0f,  1.0f,
                                     -1.0f,  1.0f};
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVertices), kQuadVertices, GL_STATIC_DRAW);

    glGenFramebuffersEXT(1, &framebuffer_);

    CreateProgram();

    initialized_ = true;
  }
}

}  // namespace gpu
