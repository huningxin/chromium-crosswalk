/*
 * Copyright (C) 2011, Google Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "modules/gamepad/Gamepad.h"
#include "modules/gamepad/GamepadDispatcher.h"

// Maximum number of entries in a vibration pattern.
const unsigned kVibrationPatternLengthMax = 99;

// Maximum duration of a vibration is 10 seconds.
const unsigned kVibrationDurationMsMax = 10000;

blink::Gamepad::VibrationPattern sanitizeGamepadVibrationPatternInternal(const blink::Gamepad::VibrationPattern& pattern)
{
    blink::Gamepad::VibrationPattern sanitized = pattern;
    size_t length = sanitized.size();

    // If the pattern is too long then truncate it.
    if (length > kVibrationPatternLengthMax) {
        sanitized.shrink(kVibrationPatternLengthMax);
        length = kVibrationPatternLengthMax;
    }

    // If any pattern entry is too long then truncate it.
    for (size_t i = 0; i < length; ++i) {
        if (sanitized[i] > kVibrationDurationMsMax)
            sanitized[i] = kVibrationDurationMsMax;
    }

    // If the last item in the pattern is a pause then discard it.
    if (length && !(length % 2))
        sanitized.removeLast();

    return sanitized;
}

namespace blink {

Gamepad::Gamepad()
    : m_index(0)
    , m_timestamp(0)
    , m_timerStart(this, &Gamepad::timerStartFired)
    , m_timerStop(this, &Gamepad::timerStopFired)
    , m_isVibrating(false)
{
}

Gamepad::~Gamepad()
{
}

void Gamepad::setAxes(unsigned count, const double* data)
{
    m_axes.resize(count);
    if (count)
        std::copy(data, data + count, m_axes.begin());
}

void Gamepad::setButtons(unsigned count, const WebGamepadButton* data)
{
    if (m_buttons.size() != count) {
        m_buttons.resize(count);
        for (unsigned i = 0; i < count; ++i)
            m_buttons[i] = GamepadButton::create();
    }
    for (unsigned i = 0; i < count; ++i) {
        m_buttons[i]->setValue(data[i].value);
        m_buttons[i]->setPressed(data[i].pressed);
        m_buttons[i]->setTouched(data[i].touched);
    }
}

void Gamepad::setPose(const WebGamepadPose& pose) {
    if (pose.isNull) {
        if (m_pose)
            m_pose = nullptr;
        return;
    }

    if (!m_pose)
        m_pose = VRPose::create();

    m_pose->setPose(pose);
}

bool Gamepad::vibrate(unsigned time)
{
    VibrationPattern pattern;
    pattern.append(time);
    return vibrate(pattern);
}

bool Gamepad::vibrate(const VibrationPattern& pattern)
{
    // Cancelling clears the stored pattern so do it before setting the new one.
    if (m_isVibrating)
        cancelVibration();

    m_pattern = sanitizeGamepadVibrationPatternInternal(pattern);

    if (m_timerStart.isActive())
        m_timerStart.stop();

    if (!m_pattern.size())
        return true;

    if (m_pattern.size() == 1 && !m_pattern[0]) {
        m_pattern.clear();
        return true;
    }

    m_timerStart.startOneShot(0, BLINK_FROM_HERE);
    m_isVibrating = true;
    return true;
}

void Gamepad::cancelVibration()
{
    m_pattern.clear();
    if (m_isVibrating) {
        //Platform::current()->cancelVibration();
        GamepadDispatcher::instance().cancelVibration(m_index);
        m_isVibrating = false;
        m_timerStop.stop();
    }
}

void Gamepad::timerStartFired(Timer<Gamepad>* timer)
{
    ASSERT_UNUSED(timer, timer == &m_timerStart);

    if (m_pattern.size()) {
        m_isVibrating = true;
        //Platform::current()->vibrate(m_pattern[0]);
        GamepadDispatcher::instance().vibrate(m_index, m_pattern[0]);
        m_timerStop.startOneShot(m_pattern[0] / 1000.0, BLINK_FROM_HERE);
        m_pattern.remove(0);
    }
}

void Gamepad::timerStopFired(Timer<Gamepad>* timer)
{
    ASSERT_UNUSED(timer, timer == &m_timerStop);

    if (m_pattern.isEmpty())
        m_isVibrating = false;

    if (m_pattern.size()) {
        m_timerStart.startOneShot(m_pattern[0] / 1000.0, BLINK_FROM_HERE);
        m_pattern.remove(0);
    }
}

DEFINE_TRACE(Gamepad)
{
    visitor->trace(m_buttons);
    visitor->trace(m_pose);
}

} // namespace blink
