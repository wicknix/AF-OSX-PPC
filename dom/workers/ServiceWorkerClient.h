/* -*- Mode: IDL; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef mozilla_dom_workers_serviceworkerclient_h
#define mozilla_dom_workers_serviceworkerclient_h

#include "nsCOMPtr.h"
#include "nsWrapperCache.h"
#include "mozilla/ErrorResult.h"
#include "mozilla/dom/BindingDeclarations.h"

namespace mozilla {
namespace dom {
namespace workers {

class ServiceWorkerClient final : public nsISupports,
                                      public nsWrapperCache
{
public:
  NS_DECL_CYCLE_COLLECTING_ISUPPORTS
  NS_DECL_CYCLE_COLLECTION_SCRIPT_HOLDER_CLASS(ServiceWorkerClient)

  ServiceWorkerClient(nsISupports* aOwner, uint64_t aId)
    : mOwner(aOwner),
      mId(aId)
  {
  }

  uint32_t Id() const
  {
    return mId;
  }

  nsISupports* GetParentObject() const
  {
    return mOwner;
  }

  void PostMessage(JSContext* aCx, JS::Handle<JS::Value> aMessage,
                   const Optional<Sequence<JS::Value>>& aTransferable,
                   ErrorResult& aRv);

  JSObject* WrapObject(JSContext* aCx) override;

private:
  ~ServiceWorkerClient()
  {
  }

  nsCOMPtr<nsISupports> mOwner;
  uint64_t mId;
};

} // namespace workers
} // namespace dom
} // namespace mozilla

#endif // mozilla_dom_workers_serviceworkerclient_h
