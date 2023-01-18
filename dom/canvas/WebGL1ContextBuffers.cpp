/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "WebGL1Context.h"
#include "WebGLBuffer.h"
#include "GLContext.h"

using namespace mozilla;
using namespace mozilla::dom;

// -------------------------------------------------------------------------
// Buffer objects

/** Target validation for BindBuffer, etc */
bool
WebGL1Context::ValidateBufferTarget(GLenum target, const char* info)
{
    switch (target) {
    case LOCAL_GL_ARRAY_BUFFER:
    case LOCAL_GL_ELEMENT_ARRAY_BUFFER:
        return true;

    default:
        ErrorInvalidEnumInfo(info, target);
        return false;
    }
}

bool
WebGL1Context::ValidateBufferIndexedTarget(GLenum target, const char* info)
{
    ErrorInvalidEnumInfo(info, target);
    return false;
}

/** Buffer and Target validation for BindBuffer */
bool
WebGL1Context::ValidateBufferForTarget(GLenum target, WebGLBuffer* buffer,
                                       const char* info)
{
    if (!buffer)
        return true;

    if (buffer->HasEverBeenBound() && target != buffer->Target()) {
        ErrorInvalidOperation("%s: buffer already bound to a different target", info);
        return false;
    }

    return true;
}
