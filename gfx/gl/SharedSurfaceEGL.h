/* -*- Mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; tab-width: 40; -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef SHARED_SURFACE_EGL_H_
#define SHARED_SURFACE_EGL_H_

#include "mozilla/Attributes.h"
#include "mozilla/Mutex.h"
#include "nsAutoPtr.h"
#include "SharedSurface.h"

namespace mozilla {
namespace gl {

class GLContext;
class GLLibraryEGL;
class TextureGarbageBin;

class SharedSurface_EGLImage
    : public SharedSurface
{
public:
    static UniquePtr<SharedSurface_EGLImage> Create(GLContext* prodGL,
                                                    const GLFormats& formats,
                                                    const gfx::IntSize& size,
                                                    bool hasAlpha,
                                                    EGLContext context);

    static SharedSurface_EGLImage* Cast(SharedSurface* surf) {
        MOZ_ASSERT(surf->mType == SharedSurfaceType::EGLImageShare);

        return (SharedSurface_EGLImage*)surf;
    }

    static bool HasExtensions(GLLibraryEGL* egl, GLContext* gl);

protected:
    mutable Mutex mMutex;
    GLLibraryEGL* const mEGL;
    const GLFormats mFormats;
    GLuint mProdTex;
public:
    const EGLImage mImage;
protected:
    GLContext* mCurConsGL;
    GLuint mConsTex;
    nsRefPtr<TextureGarbageBin> mGarbageBin;
    EGLSync mSync;

    SharedSurface_EGLImage(GLContext* gl,
                           GLLibraryEGL* egl,
                           const gfx::IntSize& size,
                           bool hasAlpha,
                           const GLFormats& formats,
                           GLuint prodTex,
                           EGLImage image);

    EGLDisplay Display() const;
    void UpdateProdTexture(const MutexAutoLock& curAutoLock);

public:
    virtual ~SharedSurface_EGLImage();

    virtual void LockProdImpl() MOZ_OVERRIDE {}
    virtual void UnlockProdImpl() MOZ_OVERRIDE {}

    virtual void Fence() MOZ_OVERRIDE;
    virtual bool WaitSync() MOZ_OVERRIDE;
    virtual bool PollSync() MOZ_OVERRIDE;

    virtual GLuint ProdTexture() MOZ_OVERRIDE {
      return mProdTex;
    }

    // Implementation-specific functions below:
    // Returns texture and target
    void AcquireConsumerTexture(GLContext* consGL, GLuint* out_texture, GLuint* out_target);
};



class SurfaceFactory_EGLImage
    : public SurfaceFactory
{
public:
    // Fallible:
    static UniquePtr<SurfaceFactory_EGLImage> Create(GLContext* prodGL,
                                                     const SurfaceCaps& caps);

protected:
    const EGLContext mContext;

    SurfaceFactory_EGLImage(GLContext* prodGL,
                            EGLContext context,
                            const SurfaceCaps& caps)
        : SurfaceFactory(prodGL, SharedSurfaceType::EGLImageShare, caps)
        , mContext(context)
    {}

public:
    virtual UniquePtr<SharedSurface> CreateShared(const gfx::IntSize& size) MOZ_OVERRIDE {
        bool hasAlpha = mReadCaps.alpha;
        return SharedSurface_EGLImage::Create(mGL, mFormats, size, hasAlpha, mContext);
    }
};

} /* namespace gfx */
} /* namespace mozilla */

#endif /* SHARED_SURFACE_EGL_H_ */
