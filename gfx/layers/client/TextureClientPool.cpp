/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "TextureClientPool.h"
#include "CompositableClient.h"
#include "mozilla/layers/ISurfaceAllocator.h"

#include "gfxPrefs.h"

#include "nsComponentManagerUtils.h"

namespace mozilla {
namespace layers {

static void
ShrinkCallback(nsITimer *aTimer, void *aClosure)
{
  static_cast<TextureClientPool*>(aClosure)->ShrinkToMinimumSize();
}

TextureClientPool::TextureClientPool(gfx::SurfaceFormat aFormat,
                                     gfx::IntSize aSize,
                                     uint32_t aMaxTextureClients,
                                     uint32_t aShrinkTimeoutMsec,
                                     ISurfaceAllocator *aAllocator)
  : mFormat(aFormat)
  , mSize(aSize)
  , mMaxTextureClients(aMaxTextureClients)
  , mShrinkTimeoutMsec(aShrinkTimeoutMsec)
  , mOutstandingClients(0)
  , mSurfaceAllocator(aAllocator)
{
  mTimer = do_CreateInstance("@mozilla.org/timer;1");
  if (aFormat == gfx::SurfaceFormat::UNKNOWN) {
    gfxWarning() << "Creating texture pool for SurfaceFormat::UNKNOWN format";
  }
}

TextureClientPool::~TextureClientPool()
{
  mTimer->Cancel();
}

#ifdef GFX_DEBUG_TRACK_CLIENTS_IN_POOL
static bool TestClientPool(const char* what,
                           TextureClient* aClient,
                           TextureClientPool* aPool)
{
  if (!aClient || !aPool) {
    return false;
  }

  TextureClientPool* actual = aClient->mPoolTracker;
  bool ok = (actual == aPool);
  if (ok) {
    ok = (aClient->GetFormat() == aPool->GetFormat());
  }

  if (!ok) {
    if (actual) {
      gfxCriticalError() << "Pool error(" << what << "): "
                   << aPool << "-" << aPool->GetFormat() << ", "
                   << actual << "-" << actual->GetFormat() << ", "
                   << aClient->GetFormat();
      MOZ_CRASH("Crashing with actual");
    } else {
      gfxCriticalError() << "Pool error(" << what << "): "
                   << aPool << "-" << aPool->GetFormat() << ", nullptr, "
                   << aClient->GetFormat();
      MOZ_CRASH("Crashing without actual");
    }
  }
  return ok;
}
#endif

TemporaryRef<TextureClient>
TextureClientPool::GetTextureClient()
{
  // Try to fetch a client from the pool
  RefPtr<TextureClient> textureClient;
  if (mTextureClients.size()) {
    mOutstandingClients++;
    textureClient = mTextureClients.top();
    mTextureClients.pop();
#ifdef GFX_DEBUG_TRACK_CLIENTS_IN_POOL
    DebugOnly<bool> ok = TestClientPool("fetch", textureClient, this);
    MOZ_ASSERT(ok);
#endif
    return textureClient;
  }

  // We're increasing the number of outstanding TextureClients without reusing a
  // client, we may need to free a deferred-return TextureClient.
  ShrinkToMaximumSize();

  // No unused clients in the pool, create one
  if (gfxPrefs::ForceShmemTiles()) {
    // gfx::BackendType::NONE means use the content backend
    textureClient = TextureClient::CreateForRawBufferAccess(mSurfaceAllocator,
      mFormat, mSize, gfx::BackendType::NONE,
      TextureFlags::IMMEDIATE_UPLOAD, ALLOC_DEFAULT);
  } else {
    textureClient = TextureClient::CreateForDrawing(mSurfaceAllocator,
      mFormat, mSize, gfx::BackendType::NONE, TextureFlags::IMMEDIATE_UPLOAD);
  }

  mOutstandingClients++;
#ifdef GFX_DEBUG_TRACK_CLIENTS_IN_POOL
  if (textureClient) {
    textureClient->mPoolTracker = this;
  }
#endif
  return textureClient;
}

void
TextureClientPool::ReturnTextureClient(TextureClient *aClient)
{
  if (!aClient) {
    return;
  }
#ifdef GFX_DEBUG_TRACK_CLIENTS_IN_POOL
  DebugOnly<bool> ok = TestClientPool("return", aClient, this);
  MOZ_ASSERT(ok);
#endif
  // Add the client to the pool:
  MOZ_ASSERT(mOutstandingClients > mTextureClientsDeferred.size());
  mOutstandingClients--;
  mTextureClients.push(aClient);

  // Shrink down if we're beyond our maximum size
  ShrinkToMaximumSize();

  // Kick off the pool shrinking timer if there are still more unused texture
  // clients than our desired minimum cache size.
  if (mTextureClients.size() > sMinCacheSize) {
    mTimer->InitWithFuncCallback(ShrinkCallback, this, mShrinkTimeoutMsec,
                                 nsITimer::TYPE_ONE_SHOT);
  }
}

void
TextureClientPool::ReturnTextureClientDeferred(TextureClient *aClient)
{
  if (!aClient) {
    return;
  }
#ifdef GFX_DEBUG_TRACK_CLIENTS_IN_POOL
  DebugOnly<bool> ok = TestClientPool("defer", aClient, this);
  MOZ_ASSERT(ok);
#endif
  mTextureClientsDeferred.push(aClient);
  ShrinkToMaximumSize();
}

void
TextureClientPool::ShrinkToMaximumSize()
{
  uint32_t totalClientsOutstanding = mTextureClients.size() + mOutstandingClients;

  // We're over our desired maximum size, immediately shrink down to the
  // maximum, or zero if we have too many outstanding texture clients.
  // We cull from the deferred TextureClients first, as we can't reuse those
  // until they get returned.
  while (totalClientsOutstanding > mMaxTextureClients) {
    if (mTextureClientsDeferred.size()) {
      MOZ_ASSERT(mOutstandingClients > 0);
      mOutstandingClients--;
      mTextureClientsDeferred.pop();
    } else {
      if (!mTextureClients.size()) {
        // Getting here means we're over our desired number of TextureClients
        // with none in the pool. This can happen for pathological cases, or
        // it could mean that mMaxTextureClients needs adjusting for whatever
        // device we're running on.
        break;
      }
      mTextureClients.pop();
    }
    totalClientsOutstanding--;
  }
}

void
TextureClientPool::ShrinkToMinimumSize()
{
  while (mTextureClients.size() > sMinCacheSize) {
    mTextureClients.pop();
  }
}

void
TextureClientPool::ReturnDeferredClients()
{
  while (!mTextureClientsDeferred.empty()) {
    mTextureClients.push(mTextureClientsDeferred.top());
    mTextureClientsDeferred.pop();

    MOZ_ASSERT(mOutstandingClients > 0);
    mOutstandingClients--;
  }
  ShrinkToMaximumSize();

  // Kick off the pool shrinking timer if there are still more unused texture
  // clients than our desired minimum cache size.
  if (mTextureClients.size() > sMinCacheSize) {
    mTimer->InitWithFuncCallback(ShrinkCallback, this, mShrinkTimeoutMsec,
                                 nsITimer::TYPE_ONE_SHOT);
  }
}

void
TextureClientPool::Clear()
{
  while (!mTextureClients.empty()) {
    mTextureClients.pop();
  }
  while (!mTextureClientsDeferred.empty()) {
    MOZ_ASSERT(mOutstandingClients > 0);
    mOutstandingClients--;
    mTextureClientsDeferred.pop();
  }
}

}
}
