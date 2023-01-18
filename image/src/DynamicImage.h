/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MOZILLA_IMAGELIB_DYNAMICIMAGE_H_
#define MOZILLA_IMAGELIB_DYNAMICIMAGE_H_

#include "mozilla/MemoryReporting.h"
#include "gfxDrawable.h"
#include "Image.h"

namespace mozilla {
namespace image {

/**
 * An Image that is dynamically created. The content of the image is provided by
 * a gfxDrawable. It's anticipated that most uses of DynamicImage will be
 * ephemeral.
 */
class DynamicImage : public Image
{
public:
  NS_DECL_ISUPPORTS
  NS_DECL_IMGICONTAINER

  explicit DynamicImage(gfxDrawable* aDrawable)
    : mDrawable(aDrawable)
  {
    MOZ_ASSERT(aDrawable, "Must have a gfxDrawable to wrap");
  }

  // Inherited methods from Image.
  virtual nsresult Init(const char* aMimeType, uint32_t aFlags) override;

  virtual already_AddRefed<ProgressTracker> GetProgressTracker() override;
  virtual size_t SizeOfSourceWithComputedFallback(
                                 MallocSizeOf aMallocSizeOf) const override;
  virtual size_t SizeOfDecoded(gfxMemoryLocation aLocation,
                               MallocSizeOf aMallocSizeOf) const override;

  virtual void IncrementAnimationConsumers() override;
  virtual void DecrementAnimationConsumers() override;
#ifdef DEBUG
  virtual uint32_t GetAnimationConsumers() override;
#endif

  virtual nsresult OnImageDataAvailable(nsIRequest* aRequest,
                                        nsISupports* aContext,
                                        nsIInputStream* aInStr,
                                        uint64_t aSourceOffset,
                                        uint32_t aCount) override;
  virtual nsresult OnImageDataComplete(nsIRequest* aRequest,
                                       nsISupports* aContext,
                                       nsresult aStatus,
                                       bool aLastPart) override;

  virtual void OnSurfaceDiscarded() override;

  virtual void SetInnerWindowID(uint64_t aInnerWindowId) override;
  virtual uint64_t InnerWindowID() const override;

  virtual bool HasError() override;
  virtual void SetHasError() override;

  virtual ImageURL* GetURI() override;

private:
  virtual ~DynamicImage() { }

  nsRefPtr<gfxDrawable> mDrawable;
};

} // namespace image
} // namespace mozilla

#endif // MOZILLA_IMAGELIB_DYNAMICIMAGE_H_
