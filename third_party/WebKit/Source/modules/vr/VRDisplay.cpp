// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "modules/vr/VRDisplay.h"

#include "core/dom/DOMException.h"
#include "core/dom/Fullscreen.h"
#include "core/inspector/ConsoleMessage.h"
#include "gpu/command_buffer/client/gles2_interface.h"
#include "modules/vr/NavigatorVR.h"
#include "modules/vr/VRController.h"
#include "modules/vr/VRDisplayCapabilities.h"
#include "modules/vr/VREyeParameters.h"
#include "modules/vr/VRLayer.h"
#include "modules/vr/VRPose.h"
#include "modules/vr/VRStageParameters.h"
#include "modules/webgl/WebGLRenderingContextBase.h"
#include "public/platform/Platform.h"

namespace blink {

namespace {

VREye stringToVREye(const String& whichEye)
{
    if (whichEye == "left")
        return VREyeLeft;
    if (whichEye == "right")
        return VREyeRight;
    return VREyeNone;
}

} // namespace

VRDisplay::VRDisplay(NavigatorVR* navigatorVR)
    : m_navigatorVR(navigatorVR)
    , m_displayId(0)
    , m_isConnected(false)
    , m_isPresenting(false)
    , m_canUpdateFramePose(true)
    , m_capabilities(new VRDisplayCapabilities())
    , m_eyeParametersLeft(new VREyeParameters())
    , m_eyeParametersRight(new VREyeParameters())
{
}

VRDisplay::~VRDisplay()
{
    gpu::gles2::GLES2Interface* sharedContext = getCompositingContext();
    if (sharedContext && m_compositorHandle) {
        sharedContext->DeleteVRCompositorCHROMIUM(m_compositorHandle);
    }
}

VRController* VRDisplay::controller()
{
    return m_navigatorVR->controller();
}

void VRDisplay::update(const WebVRDisplay& display)
{
    m_displayId = display.index;
    m_displayName = display.displayName;
    m_compositorType = display.compositorType;
    m_isConnected = true;

    m_capabilities->setHasOrientation(display.capabilities.hasOrientation);
    m_capabilities->setHasPosition(display.capabilities.hasPosition);
    m_capabilities->setHasExternalDisplay(display.capabilities.hasExternalDisplay);
    m_capabilities->setCanPresent(display.capabilities.canPresent);
    m_capabilities->setMaxLayers(display.capabilities.canPresent ? 1 : 0);

    m_eyeParametersLeft->update(display.leftEye);
    m_eyeParametersRight->update(display.rightEye);

    if (display.hasStageParameters) {
        if (!m_stageParameters)
            m_stageParameters = new VRStageParameters();
        m_stageParameters->update(display.stageParameters);
    } else {
        m_stageParameters = nullptr;
    }
}

VRPose* VRDisplay::getPose()
{
    if (m_canUpdateFramePose) {
        m_framePose = getImmediatePose();
        Platform::current()->currentThread()->addTaskObserver(this);
        m_canUpdateFramePose = false;
    }

    return m_framePose;
}

VRPose* VRDisplay::getImmediatePose()
{
    WebVRPose webPose;
    controller()->getPose(m_displayId, webPose);

    VRPose* pose = VRPose::create();
    pose->setPose(webPose);
    return pose;
}

void VRDisplay::resetPose()
{
    controller()->resetPose(m_displayId);

    if (m_compositorHandle) {
        gpu::gles2::GLES2Interface* sharedContext = getCompositingContext();
        if (sharedContext)
            sharedContext->ResetVRCompositorPoseCHROMIUM(m_compositorHandle);
    }
}

VREyeParameters* VRDisplay::getEyeParameters(const String& whichEye)
{
    switch (stringToVREye(whichEye)) {
    case VREyeLeft:
        return m_eyeParametersLeft;
    case VREyeRight:
        return m_eyeParametersRight;
    default:
        return nullptr;
    }
}

int VRDisplay::requestAnimationFrame(FrameRequestCallback* callback)
{
    // TODO: Use HMD-specific rAF when an external display is present.
    callback->m_useLegacyTimeBase = false;
    if (Document* doc = m_navigatorVR->document())
        return doc->requestAnimationFrame(callback);
    return 0;
}

void VRDisplay::cancelAnimationFrame(int id)
{
    if (Document* document = m_navigatorVR->document())
        document->cancelAnimationFrame(id);
}

gpu::gles2::GLES2Interface* VRDisplay::getCompositingContext() {
    if (!m_contextProvider)
        m_contextProvider = adoptPtr(Platform::current()->createSharedOffscreenGraphicsContext3DProvider());

    gpu::gles2::GLES2Interface* sharedContext = nullptr;
    if (m_contextProvider) {
        sharedContext = m_contextProvider->contextGL();

        if (!sharedContext)
            return nullptr;
    }

    return sharedContext;
}

ScriptPromise VRDisplay::requestPresent(ScriptState* scriptState, const VRLayer& layer)
{
    static bool didPrintDeprecationWarning = false;
    if (!didPrintDeprecationWarning) {
        Document* document = m_navigatorVR->document();
        if (document) {
            document->addConsoleMessage(ConsoleMessage::create(RenderingMessageSource, WarningMessageLevel, "Using a deprecated form of requestPresent. Should pass in an array of VRLayers."));
            didPrintDeprecationWarning = true;
        }
    }

    HeapVector<VRLayer> layers;
    layers.append(layer);
    return requestPresent(scriptState, layers);
}

ScriptPromise VRDisplay::requestPresent(ScriptState* scriptState, const HeapVector<VRLayer>& layers)
{
    ScriptPromiseResolver* resolver = ScriptPromiseResolver::create(scriptState);
    ScriptPromise promise = resolver->promise();

    m_context = nullptr;
    m_isPresenting = false;

    if (!m_capabilities->canPresent()) {
        DOMException* exception = DOMException::create(InvalidStateError, "VRDisplay cannot present");
        resolver->reject(exception);
        return promise;
    }

    if (layers.size() == 0 || layers.size() > m_capabilities->maxLayers()) {
        DOMException* exception = DOMException::create(InvalidStateError, "Invalid number of layers.");
        if (m_isPresenting) {
            exitPresent(scriptState);
        }

        resolver->reject(exception);
        return promise;
    }

    m_layer = layers[0];

    if (m_layer.source() != nullptr) {
        // On devices without an external VR display just make the canvas
        // fullscreen for now. It's a major hack, but it works for a POC.
        if (!m_capabilities->hasExternalDisplay()) {
            Fullscreen::from(m_layer.source()->document()).requestFullscreen(*(m_layer.source()), Fullscreen::UnprefixedRequest);
            m_isPresenting = true;
            // TODO: Listen for fullscreen exit by the UA.
            resolver->resolve();

            m_navigatorVR->fireVRDisplayPresentChange();
        } else {
            CanvasRenderingContext* renderingContext = m_layer.source()->renderingContext();

            if (!renderingContext || !renderingContext->is3d()) {
                DOMException* exception = DOMException::create(InvalidStateError, "Layer source must have a WebGLRenderingContext");
                resolver->reject(exception);
            } else {
                m_context = toWebGLRenderingContextBase(renderingContext);

                gpu::gles2::GLES2Interface* sharedContext = getCompositingContext();
                if (!sharedContext) {
                    DOMException* exception = DOMException::create(InvalidStateError, "Unable to acquire shared context");
                    if (m_isPresenting) { exitPresent(scriptState); }
                    resolver->reject(exception);
                } else {
                    m_compositorHandle = sharedContext->CreateVRCompositorCHROMIUM(static_cast<GLenum>(m_compositorType));

                    if (!m_compositorHandle) {
                        DOMException* exception = DOMException::create(InvalidStateError, "Unable to create VR compositor");
                        if (m_isPresenting) { exitPresent(scriptState); }
                        resolver->reject(exception);
                    } else {

                        if (m_layer.hasLeftBounds()) {
                            sharedContext->VRCompositorTextureBoundsCHROMIUM(m_compositorHandle, 0, // Left Eye
                                m_layer.leftBounds()[0], m_layer.leftBounds()[1], m_layer.leftBounds()[2], m_layer.leftBounds()[3]);
                        }

                        if (m_layer.hasRightBounds()) {
                            sharedContext->VRCompositorTextureBoundsCHROMIUM(m_compositorHandle, 1, // Right Eye
                                m_layer.rightBounds()[0], m_layer.rightBounds()[1], m_layer.rightBounds()[2], m_layer.rightBounds()[3]);
                        }

                        m_isPresenting = true;
                        // TODO: Resolve when presentation is confirmed
                        resolver->resolve();

                        m_navigatorVR->fireVRDisplayPresentChange();

                        if (m_compositorType == VR_COMPOSITOR_OCULUS) {
                            // FIXME: This is terrible. :(
                            resetPose();
                        }
                    }
                }
            }
        }
    } else {
        // TODO: Resolve when presentation is confirmed
        resolver->resolve();
    }

    // TODO: Begin presenting!

    return promise;
}

ScriptPromise VRDisplay::exitPresent(ScriptState* scriptState)
{
    ScriptPromiseResolver* resolver = ScriptPromiseResolver::create(scriptState);
    ScriptPromise promise = resolver->promise();

    if (!m_isPresenting) {
        // Can't stop presenting if we're not presenting.
        DOMException* exception = DOMException::create(InvalidStateError, "VRDisplay is not presenting");
        resolver->reject(exception);
        return promise;
    }

    if (!m_capabilities->hasExternalDisplay()) {
        Fullscreen::fullyExitFullscreen(m_layer.source()->document());
    } else {
        gpu::gles2::GLES2Interface* sharedContext = getCompositingContext();
        if (sharedContext && m_compositorHandle) {
            sharedContext->DeleteVRCompositorCHROMIUM(m_compositorHandle);
            sharedContext->Finish();
        }
    }

    m_context = nullptr;
    m_contextProvider = nullptr;
    m_compositorHandle = 0;
    m_isPresenting = false;

    // TODO: Resolve when exit is confirmed
    resolver->resolve();

    m_navigatorVR->fireVRDisplayPresentChange();

    return promise;
}

HeapVector<VRLayer> VRDisplay::getLayers()
{
    HeapVector<VRLayer> layers;

    if (m_isPresenting) {
        layers.append(m_layer);
    }

    return layers;
}

void VRDisplay::submitFrame(VRPose* pose)
{
    if (!pose) {
        pose = m_framePose;
    }

    if (m_context && m_compositorHandle) {
        gpu::gles2::GLES2Interface* sharedContext = getCompositingContext();
        if (!sharedContext)
            return;

        // TODO: Should be able to more directly submit from here if we can
        // figure out how to do so without blocking the compositor.
        DrawingBuffer* drawingBuffer = m_context->drawingBuffer();

        WebExternalTextureMailbox mailbox;
        drawingBuffer->prepareMailbox(&mailbox, nullptr);
        drawingBuffer->markContentsChanged();

        if (mailbox.validSyncToken)
            sharedContext->WaitSyncTokenCHROMIUM(mailbox.syncToken);
        GLuint vrSourceTexture = sharedContext->CreateAndConsumeTextureCHROMIUM(mailbox.textureTarget, mailbox.name);

        // Get last orientation
        float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
        if (!pose || pose->orientation()) {
            x = pose->orientation()->data()[0];
            y = pose->orientation()->data()[1];
            z = pose->orientation()->data()[2];
            w = pose->orientation()->data()[3];
        }

        sharedContext->SubmitVRCompositorFrameCHROMIUM(
            m_compositorHandle, vrSourceTexture, x, y, z, w);

        sharedContext->Flush();

        drawingBuffer->mailboxReleased(mailbox, false);
    }
}

void VRDisplay::didProcessTask()
{
    // Pose should be stable until control is returned to the user agent.
    if (!m_canUpdateFramePose) {
        Platform::current()->currentThread()->removeTaskObserver(this);
        m_canUpdateFramePose = true;
    }
}

DEFINE_TRACE(VRDisplay)
{
    visitor->trace(m_navigatorVR);
    visitor->trace(m_capabilities);
    visitor->trace(m_stageParameters);
    visitor->trace(m_eyeParametersLeft);
    visitor->trace(m_eyeParametersRight);
    visitor->trace(m_framePose);
    visitor->trace(m_layer);
    visitor->trace(m_context);
}

} // namespace blink
