// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef DOMArrayBufferBase_h
#define DOMArrayBufferBase_h

#include "bindings/core/v8/ScriptWrappable.h"
#include "core/CoreExport.h"
#include "wtf/ArrayBuffer.h"
#include "wtf/RefCounted.h"

namespace blink {

class CORE_EXPORT DOMArrayBufferBase : public RefCounted<DOMArrayBufferBase>, public ScriptWrappable {
public:
    virtual ~DOMArrayBufferBase() { }

    const WTF::ArrayBuffer* buffer() const { return m_buffer.get(); }
    WTF::ArrayBuffer* buffer() { return m_buffer.get(); }

    const void* data() const {
      if (m_isExternal)
          return m_source;
      else
          return buffer()->data();
    }
    void* data() {
        if (m_isExternal)
            return m_source;
        else
            return buffer()->data();
    }
    unsigned byteLength() const {
        if (m_isExternal)
            return m_byteLength;
        else
            return buffer()->byteLength();
    }
    bool transfer(WTF::ArrayBufferContents& result) { return buffer()->transfer(result); }
    bool shareContentsWith(WTF::ArrayBufferContents& result) { return buffer()->shareContentsWith(result); }
    bool isNeutered() const { return buffer()->isNeutered(); }
    bool isShared() const { return buffer()->isShared(); }

    v8::Local<v8::Object> wrap(v8::Isolate*, v8::Local<v8::Object> creationContext) override
    {
        ASSERT_NOT_REACHED();
        return v8::Local<v8::Object>();
    }

protected:
    explicit DOMArrayBufferBase(PassRefPtr<WTF::ArrayBuffer> buffer)
        : m_buffer(buffer),
          m_isExternal(false),
          m_source(nullptr),
          m_byteLength(0)
    {
        ASSERT(m_buffer);
    }

    DOMArrayBufferBase(void* source, unsigned byteLength)
        : m_buffer(WTF::ArrayBuffer::create(source, byteLength)),
          m_isExternal(true),
          m_source(source),
          m_byteLength(byteLength)
    {
    }

    RefPtr<WTF::ArrayBuffer> m_buffer;
    bool m_isExternal;
    void* m_source;
    unsigned m_byteLength;
 
};

} // namespace blink

#endif // DOMArrayBuffer_h
