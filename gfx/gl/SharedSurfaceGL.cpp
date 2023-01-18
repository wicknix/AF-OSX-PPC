/* -*- Mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; tab-width: 40; -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "SharedSurfaceGL.h"

#include "GLBlitHelper.h"
#include "GLContext.h"
#include "GLReadTexImageHelper.h"
#include "mozilla/gfx/2D.h"
#include "ScopedGLHelpers.h"

namespace mozilla {
namespace gl {

using gfx::IntSize;
using gfx::SurfaceFormat;

/*static*/ UniquePtr<SharedSurface_Basic>
SharedSurface_Basic::Create(GLContext* gl,
                            const GLFormats& formats,
                            const IntSize& size,
                            bool hasAlpha)
{
    UniquePtr<SharedSurface_Basic> ret;
    gl->MakeCurrent();

    GLContext::LocalErrorScope localError(*gl);
    GLuint tex = CreateTexture(gl, formats.color_texInternalFormat,
                               formats.color_texFormat,
                               formats.color_texType,
                               size);

    GLenum err = localError.GetError();
    MOZ_ASSERT_IF(err != LOCAL_GL_NO_ERROR, err == LOCAL_GL_OUT_OF_MEMORY);
    if (err) {
        gl->fDeleteTextures(1, &tex);
        return Move(ret);
    }

    SurfaceFormat format = SurfaceFormat::B8G8R8X8;
    switch (formats.color_texInternalFormat) {
    case LOCAL_GL_RGB:
    case LOCAL_GL_RGB8:
        if (formats.color_texType == LOCAL_GL_UNSIGNED_SHORT_5_6_5)
            format = SurfaceFormat::R5G6B5;
        else
            format = SurfaceFormat::B8G8R8X8;
        break;
    case LOCAL_GL_RGBA:
    case LOCAL_GL_RGBA8:
    case LOCAL_GL_BGRA:
    case LOCAL_GL_BGRA8_EXT:
        format = SurfaceFormat::B8G8R8A8;
        break;
    default:
        MOZ_CRASH("Unhandled Tex format.");
    }

    ret.reset( new SharedSurface_Basic(gl, size, hasAlpha, format, tex) );
    return Move(ret);
}

SharedSurface_Basic::SharedSurface_Basic(GLContext* gl,
                                         const IntSize& size,
                                         bool hasAlpha,
                                         SurfaceFormat format,
                                         GLuint tex)
    : SharedSurface(SharedSurfaceType::Basic,
                    AttachmentType::GLTexture,
                    gl,
                    size,
                    hasAlpha)
    , mTex(tex)
    , mFB(0)
{
    mGL->MakeCurrent();
    mGL->fGenFramebuffers(1, &mFB);

    ScopedBindFramebuffer autoFB(mGL, mFB);
    mGL->fFramebufferTexture2D(LOCAL_GL_FRAMEBUFFER,
                              LOCAL_GL_COLOR_ATTACHMENT0,
                              LOCAL_GL_TEXTURE_2D,
                              mTex,
                              0);

    DebugOnly<GLenum> status = mGL->fCheckFramebufferStatus(LOCAL_GL_FRAMEBUFFER);
    MOZ_ASSERT(status == LOCAL_GL_FRAMEBUFFER_COMPLETE);
}

SharedSurface_Basic::~SharedSurface_Basic()
{
    if (!mGL->MakeCurrent())
        return;

    if (mFB)
        mGL->fDeleteFramebuffers(1, &mFB);

    mGL->fDeleteTextures(1, &mTex);
}

////////////////////////////////////////////////////////////////////////
// SharedSurface_GLTexture

/*static*/ UniquePtr<SharedSurface_GLTexture>
SharedSurface_GLTexture::Create(GLContext* prodGL,
                                GLContext* consGL,
                                const GLFormats& formats,
                                const IntSize& size,
                                bool hasAlpha,
                                GLuint texture)
{
    MOZ_ASSERT(prodGL);
    MOZ_ASSERT(!consGL || prodGL->SharesWith(consGL));

    prodGL->MakeCurrent();

    GLuint tex = texture;

    bool ownsTex = false;

    UniquePtr<SharedSurface_GLTexture> ret;

    if (!tex) {
        GLContext::LocalErrorScope localError(*prodGL);

        tex = CreateTextureForOffscreen(prodGL, formats, size);

        GLenum err = localError.GetError();
        MOZ_ASSERT_IF(err != LOCAL_GL_NO_ERROR, err == LOCAL_GL_OUT_OF_MEMORY);
        if (err) {
            prodGL->fDeleteTextures(1, &tex);
            return Move(ret);
        }

        ownsTex = true;
    }

    ret.reset( new SharedSurface_GLTexture(prodGL, consGL, size,
                                           hasAlpha, tex, ownsTex) );
    return Move(ret);
}

SharedSurface_GLTexture::~SharedSurface_GLTexture()
{
    if (!mGL->MakeCurrent())
        return;

    if (mOwnsTex) {
        mGL->fDeleteTextures(1, &mTex);
    }

    if (mSync) {
        mGL->fDeleteSync(mSync);
    }
}

void
SharedSurface_GLTexture::Fence()
{
    MutexAutoLock lock(mMutex);
    mGL->MakeCurrent();

    if (mConsGL && mGL->IsExtensionSupported(GLContext::ARB_sync)) {
        if (mSync) {
            mGL->fDeleteSync(mSync);
            mSync = 0;
        }

        mSync = mGL->fFenceSync(LOCAL_GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        if (mSync) {
            mGL->fFlush();
            return;
        }
    }
    MOZ_ASSERT(!mSync);

    mGL->fFinish();
}

bool
SharedSurface_GLTexture::WaitSync()
{
    MutexAutoLock lock(mMutex);
    if (!mSync) {
        // We either used glFinish, or we passed this fence already.
        // (PollSync/WaitSync returned true previously)
        return true;
    }

    mConsGL->MakeCurrent();
    MOZ_ASSERT(mConsGL->IsExtensionSupported(GLContext::ARB_sync));

    mConsGL->fWaitSync(mSync,
                       0,
                       LOCAL_GL_TIMEOUT_IGNORED);
    mConsGL->fDeleteSync(mSync);
    mSync = 0;

    return true;
}

bool
SharedSurface_GLTexture::PollSync()
{
    MutexAutoLock lock(mMutex);
    if (!mSync) {
        // We either used glFinish, or we passed this fence already.
        // (PollSync/WaitSync returned true previously)
        return true;
    }

    mConsGL->MakeCurrent();
    MOZ_ASSERT(mConsGL->IsExtensionSupported(GLContext::ARB_sync));

    GLint status = 0;
    mConsGL->fGetSynciv(mSync,
                        LOCAL_GL_SYNC_STATUS,
                        1,
                        nullptr,
                        &status);
    if (status != LOCAL_GL_SIGNALED)
        return false;

    mConsGL->fDeleteSync(mSync);
    mSync = 0;

    return true;
}

GLuint
SharedSurface_GLTexture::ConsTexture(GLContext* consGL)
{
    MutexAutoLock lock(mMutex);
    MOZ_ASSERT(consGL);
    MOZ_ASSERT(mGL->SharesWith(consGL));
    MOZ_ASSERT_IF(mConsGL, consGL == mConsGL);

    mConsGL = consGL;

    return mTex;
}

} /* namespace gfx */
} /* namespace mozilla */
