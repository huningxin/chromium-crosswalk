// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CONTENT_BROWSER_SPEECH_INTEL_PCSDK_LOCAL_ENGINE_H_
#define CONTENT_BROWSER_SPEECH_INTEL_PCSDK_LOCAL_ENGINE_H_

#include <string>
#include <vector>

#include "base/basictypes.h"
#include "base/memory/ref_counted.h"
#include "base/memory/scoped_ptr.h"
#include "content/browser/speech/audio_encoder.h"
#include "content/browser/speech/chunked_byte_buffer.h"
#include "content/browser/speech/speech_recognition_engine.h"
#include "content/common/content_export.h"
#include "content/public/common/speech_recognition_error.h"
#include "third_party/libpxc/include/pxcsmartptr.h"
#include "third_party/libpxc/include/pxcvoice.h"

namespace content {

class AudioChunk;
struct SpeechRecognitionError;
struct SpeechRecognitionResult;

class CONTENT_EXPORT IntelPcsdkLocalEngine
    : public NON_EXPORTED_BASE(SpeechRecognitionEngine),
      public NON_EXPORTED_BASE(PXCVoiceRecognition::Recognition::Handler),
      public NON_EXPORTED_BASE(PXCVoiceRecognition::Alert::Handler) {
 public:
  // Duration of each audio packet.
  static const int kAudioPacketIntervalMs;

  IntelPcsdkLocalEngine();
  virtual ~IntelPcsdkLocalEngine();

  // SpeechRecognitionEngine methods.
  virtual void SetConfig(const SpeechRecognitionEngineConfig& config) OVERRIDE;
  virtual void StartRecognition() OVERRIDE;
  virtual void EndRecognition() OVERRIDE;
  virtual void TakeAudioChunk(const AudioChunk& data) OVERRIDE;
  virtual void AudioChunksEnded() OVERRIDE;
  virtual bool IsRecognitionPending() const OVERRIDE;
  virtual int GetDesiredAudioChunkDurationMs() const OVERRIDE;

  // PXCVoiceRecognition::Recognition::Handler method.
  virtual void PXCAPI OnRecognized(
      PXCVoiceRecognition::Recognition *data) OVERRIDE;

  // PXCVoiceRecognition::Alert::Handler method.
  virtual void PXCAPI OnAlert(PXCVoiceRecognition::Alert *alert) OVERRIDE;

 private:
  void InitPxcVoiceRecognition();
  void ConfigPxcVoiceRecognition();

  SpeechRecognitionEngineConfig config_;

  PXCVoiceRecognition::ProfileInfo pxc_profile_;
  PXCSmartPtr<PXCSession> pxc_session_;
  PXCSmartPtr<PXCVoiceRecognition> pxc_voice_recognition_;

  DISALLOW_COPY_AND_ASSIGN(IntelPcsdkLocalEngine);
};

}  // namespace content

#endif  // CONTENT_BROWSER_SPEECH_INTEL_PCSDK_LOCAL_ENGINE_H_
