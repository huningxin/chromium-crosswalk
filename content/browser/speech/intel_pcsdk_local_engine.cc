// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/browser/speech/intel_pcsdk_local_engine.h"

#include "base/logging.h"

namespace content {

const int IntelPcsdkLocalEngine::kAudioPacketIntervalMs = 100;

IntelPcsdkLocalEngine::IntelPcsdkLocalEngine() {
  InitPxcVoiceRecognition();
}

IntelPcsdkLocalEngine::~IntelPcsdkLocalEngine() {
}

void IntelPcsdkLocalEngine::SetConfig(
    const SpeechRecognitionEngineConfig& config) {
  config_ = config;

  ConfigPxcVoiceRecognition();
}

void IntelPcsdkLocalEngine::InitPxcVoiceRecognition() {
  pxcStatus status = PXCSession_Create(&pxc_session_);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to create the PXCSession\n";
    return;
  }

  // Create PXCVoiceRecognition instance
  status = pxc_session_->CreateImpl(PXCVoiceRecognition::CUID,
      reinterpret_cast<void **>(&pxc_voice_recognition_));
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to create PXCVoiceRecognition " << status;
    pxc_voice_recognition_ = 0;
    return;
  }
}

void IntelPcsdkLocalEngine::ConfigPxcVoiceRecognition() {
  // Query PXCVoiceRecognition profile
  pxcStatus status = pxc_voice_recognition_->QueryProfile(0, &pxc_profile_);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to create QueryProfile " << status;
    pxc_voice_recognition_ = 0;
    return;
  }

  // Configure the inputs of speech recognition profile.
  memset(&pxc_profile_.inputs.info, 0, sizeof(pxc_profile_.inputs.info));
  pxc_profile_.inputs.info.format     = PXCAudio::AUDIO_FORMAT_PCM;
  pxc_profile_.inputs.info.nchannels  = 1;
  pxc_profile_.inputs.info.channelMask = PXCAudio::CHANNEL_MASK_FRONT_CENTER;
  pxc_profile_.inputs.info.sampleRate = config_.audio_sample_rate;
  pxc_profile_.inputs.info.bufferSize =
      config_.audio_sample_rate / 1000 * kAudioPacketIntervalMs;

  status = pxc_voice_recognition_->SetProfile(&pxc_profile_);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to set the PXCVoiceRecognition configuration "
                << status;
    pxc_voice_recognition_ = 0;
    return;
  }

  // TODO(nhu): investigate how to map the grammer in Web to pxc voice
  // recognition.
  pxcUID grammar = 0;
  pxc_voice_recognition_->SetGrammar(grammar);
}

void IntelPcsdkLocalEngine::StartRecognition() {
  if (!pxc_voice_recognition_.IsValid()) {
    delegate()->OnSpeechRecognitionEngineError(
        SpeechRecognitionError(SPEECH_RECOGNITION_ERROR_NONE));
    return;
  }

  // TODO(gaochun): Add multi-language support for pxc speech recognition.
  if (config_.language.size() >= 2) {
    if (config_.language.substr(0, 2) != std::string("en")) {
      delegate()->OnSpeechRecognitionEngineError(
        SpeechRecognitionError(SPEECH_RECOGNITION_ERROR_NONE));
      return;
    }
  }

  // Subscribe to events.
  // TODO(nhu): investigate what's meaning of the threshold.
  pxc_voice_recognition_->SubscribeRecognition(0, this);
  pxc_voice_recognition_->SubscribeAlert(this);
}

void IntelPcsdkLocalEngine::EndRecognition() {
  if (!pxc_voice_recognition_.IsValid()) {
    delegate()->OnSpeechRecognitionEngineError(
        SpeechRecognitionError(SPEECH_RECOGNITION_ERROR_NONE));
    return;
  }

  // Wait until the remaining processing is completed.
  pxc_voice_recognition_->ProcessAudioAsync(0, 0);

  // Unsubscribe to events.
  pxc_voice_recognition_->SubscribeRecognition(0, 0);
  pxc_voice_recognition_->SubscribeAlert(0);
}

void IntelPcsdkLocalEngine::TakeAudioChunk(const AudioChunk& data) {
  if (!pxc_voice_recognition_.IsValid()) {
    delegate()->OnSpeechRecognitionEngineError(
        SpeechRecognitionError(SPEECH_RECOGNITION_ERROR_NONE));
    return;
  }

  PXCSmartPtr<PXCAudio> pxc_audio;
  PXCSmartPtr<PXCAccelerator> pxc_accelerator;
  pxc_session_->CreateAccelerator(PXCAccelerator::ACCEL_TYPE_CPU,
                                  &pxc_accelerator);

  size_t buffer_size = data.bytes_per_sample() * data.NumSamples();
  scoped_ptr<uint8[]> audio_buffer(new uint8[buffer_size]);
  memcpy(audio_buffer.get(), data.AsString().c_str(), buffer_size);

  PXCAudio::AudioInfo audio_info;
  memset(&audio_info, 0, sizeof(audio_info));
  audio_info.format     = PXCAudio::AUDIO_FORMAT_PCM;
  audio_info.nchannels  = 1;
  audio_info.channelMask = PXCAudio::CHANNEL_MASK_FRONT_CENTER;
  audio_info.sampleRate = config_.audio_sample_rate;
  audio_info.bufferSize = data.NumSamples();

  PXCAudio::AudioData audio_data;
  memset(&audio_data, 0, sizeof(audio_data));
  audio_data.format   = PXCAudio::AUDIO_FORMAT_PCM;
  audio_data.dataPtr  = audio_buffer.get();
  audio_data.type     = PXCAudio::SURFACE_TYPE_SYSTEM_MEMORY;
  audio_data.dataSize = data.NumSamples();

  pxcStatus status = pxc_accelerator->CreateAudio(
      &audio_info, 0, &audio_data, &pxc_audio);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to create the audio " << status;
    return;
  }

  PXCSmartSPArray sps(1);

  status = pxc_voice_recognition_->ProcessAudioAsync(pxc_audio,
                                                     &sps[0]);
  if (status < PXC_STATUS_NO_ERROR) {
    DLOG(ERROR) << "Failed to ProcessAudioAsync " << status;
    return;
  }
  sps.SynchronizeEx();
}

void IntelPcsdkLocalEngine::AudioChunksEnded() {
  if (pxc_voice_recognition_.IsValid()) {
    // Wait until the remaining processing is completed.
    pxc_voice_recognition_->ProcessAudioAsync(0, 0);
  }

  // Dispatch the empty final result to ensure recognizer FSM running.
  SpeechRecognitionResults results;
  delegate()->OnSpeechRecognitionEngineResults(results);
}

bool IntelPcsdkLocalEngine::IsRecognitionPending() const {
  // TODO(nhu): how to check PXC voice recognition is pending?
  return false;
}

int IntelPcsdkLocalEngine::GetDesiredAudioChunkDurationMs() const {
  return kAudioPacketIntervalMs;
}

void PXCAPI IntelPcsdkLocalEngine::OnRecognized(
    PXCVoiceRecognition::Recognition *data) {
  DLOG(INFO) << "IntelPcsdkLocalEngine::OnRecognized " << data->dictation;
  SpeechRecognitionHypothesis hypothesis;
  hypothesis.confidence = data->confidence;
  hypothesis.utterance = string16(data->dictation);

  SpeechRecognitionResult result;
  result.hypotheses.push_back(hypothesis);
  result.is_provisional = false;

  SpeechRecognitionResults results;
  results.push_back(result);

  delegate()->OnSpeechRecognitionEngineResults(results);
}

void PXCAPI IntelPcsdkLocalEngine::OnAlert(PXCVoiceRecognition::Alert *alert) {
  DLOG(WARNING) << "IntelPcsdkLocalEngine::OnAlert " << alert->label;

  // TODO(nhu): investigate the alerts from PXCVoiceRecognition.
#if defined(SUPPORT_PXC_ALERTS)
  if (alert->label == PXCVoiceRecognition::Alert::LABEL_SPEECH_UNRECOGNIZABLE) {
    // In test, the PXCVoiceRecognition always triggers UNRECOGNIZABLE alert in
    // each session. It is confusing. So disable it temprorily.
    delegate()->OnSpeechRecognitionEngineError(
        SpeechRecognitionError(SPEECH_RECOGNITION_ERROR_NO_MATCH));
  }
#endif
}

}  // namespace content
