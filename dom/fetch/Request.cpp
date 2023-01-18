/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Request.h"

#include "nsIURI.h"
#include "nsPIDOMWindow.h"

#include "mozilla/ErrorResult.h"
#include "mozilla/dom/Headers.h"
#include "mozilla/dom/Fetch.h"
#include "mozilla/dom/Promise.h"
#include "mozilla/dom/URL.h"
#include "mozilla/dom/workers/bindings/URL.h"

#include "WorkerPrivate.h"

namespace mozilla {
namespace dom {

NS_IMPL_CYCLE_COLLECTING_ADDREF(Request)
NS_IMPL_CYCLE_COLLECTING_RELEASE(Request)
NS_IMPL_CYCLE_COLLECTION_WRAPPERCACHE(Request, mOwner, mHeaders)

NS_INTERFACE_MAP_BEGIN_CYCLE_COLLECTION(Request)
  NS_WRAPPERCACHE_INTERFACE_MAP_ENTRY
  NS_INTERFACE_MAP_ENTRY(nsISupports)
NS_INTERFACE_MAP_END

Request::Request(nsIGlobalObject* aOwner, InternalRequest* aRequest)
  : FetchBody<Request>()
  , mOwner(aOwner)
  , mRequest(aRequest)
  , mContext(RequestContext::Fetch)
{
}

Request::~Request()
{
}

already_AddRefed<InternalRequest>
Request::GetInternalRequest()
{
  nsRefPtr<InternalRequest> r = mRequest;
  return r.forget();
}

/*static*/ already_AddRefed<Request>
Request::Constructor(const GlobalObject& aGlobal,
                     const RequestOrUSVString& aInput,
                     const RequestInit& aInit, ErrorResult& aRv)
{
  nsRefPtr<InternalRequest> request;

  nsCOMPtr<nsIGlobalObject> global = do_QueryInterface(aGlobal.GetAsSupports());

  if (aInput.IsRequest()) {
    nsRefPtr<Request> inputReq = &aInput.GetAsRequest();
    nsCOMPtr<nsIInputStream> body;
    inputReq->GetBody(getter_AddRefs(body));
    if (body) {
      if (inputReq->BodyUsed()) {
        aRv.ThrowTypeError(MSG_FETCH_BODY_CONSUMED_ERROR);
        return nullptr;
      } else {
        inputReq->SetBodyUsed();
      }
    }

    request = inputReq->GetInternalRequest();
  } else {
    request = new InternalRequest();
  }

  request = request->GetRequestConstructorCopy(global, aRv);
  if (NS_WARN_IF(aRv.Failed())) {
    return nullptr;
  }

  RequestMode fallbackMode = RequestMode::EndGuard_;
  RequestCredentials fallbackCredentials = RequestCredentials::EndGuard_;
  if (aInput.IsUSVString()) {
    nsString input;
    input.Assign(aInput.GetAsUSVString());

    nsString requestURL;
    if (NS_IsMainThread()) {
      nsCOMPtr<nsPIDOMWindow> window = do_QueryInterface(global);
      MOZ_ASSERT(window);
      nsCOMPtr<nsIURI> docURI = window->GetDocumentURI();
      nsCString spec;
      aRv = docURI->GetSpec(spec);
      if (NS_WARN_IF(aRv.Failed())) {
        return nullptr;
      }

      nsRefPtr<mozilla::dom::URL> url =
        dom::URL::Constructor(aGlobal, input, NS_ConvertUTF8toUTF16(spec), aRv);
      if (aRv.Failed()) {
        return nullptr;
      }

      url->Stringify(requestURL, aRv);
      if (aRv.Failed()) {
        return nullptr;
      }
    } else {
      workers::WorkerPrivate* worker = workers::GetCurrentThreadWorkerPrivate();
      MOZ_ASSERT(worker);
      worker->AssertIsOnWorkerThread();

      nsString baseURL = NS_ConvertUTF8toUTF16(worker->GetLocationInfo().mHref);
      nsRefPtr<workers::URL> url =
        workers::URL::Constructor(aGlobal, input, baseURL, aRv);
      if (aRv.Failed()) {
        return nullptr;
      }

      url->Stringify(requestURL, aRv);
      if (aRv.Failed()) {
        return nullptr;
      }
    }
    request->SetURL(NS_ConvertUTF16toUTF8(requestURL));
    fallbackMode = RequestMode::Cors;
    fallbackCredentials = RequestCredentials::Omit;
  }

  // CORS-with-forced-preflight is not publicly exposed and should not be
  // considered a valid value.
  if (aInit.mMode.WasPassed() &&
      aInit.mMode.Value() == RequestMode::Cors_with_forced_preflight) {
    NS_NAMED_LITERAL_STRING(sourceDescription, "'mode' member of RequestInit");
    NS_NAMED_LITERAL_STRING(value, "cors-with-forced-preflight");
    NS_NAMED_LITERAL_STRING(type, "RequestMode");
    aRv.ThrowTypeError(MSG_INVALID_ENUM_VALUE, &sourceDescription, &value, &type);
    return nullptr;
  }
  RequestMode mode = aInit.mMode.WasPassed() ? aInit.mMode.Value() : fallbackMode;
  RequestCredentials credentials =
    aInit.mCredentials.WasPassed() ? aInit.mCredentials.Value()
                                   : fallbackCredentials;

  if (mode != RequestMode::EndGuard_) {
    request->SetMode(mode);
  }

  if (credentials != RequestCredentials::EndGuard_) {
    request->SetCredentialsMode(credentials);
  }

  // Request constructor step 14.
  if (aInit.mMethod.WasPassed()) {
    nsAutoCString method(aInit.mMethod.Value());
    nsAutoCString upperCaseMethod = method;
    ToUpperCase(upperCaseMethod);

    // Step 14.1. Disallow forbidden methods, and anything that is not a HTTP
    // token, since HTTP states that Method may be any of the defined values or
    // a token (extension method).
    if (upperCaseMethod.EqualsLiteral("CONNECT") ||
        upperCaseMethod.EqualsLiteral("TRACE") ||
        upperCaseMethod.EqualsLiteral("TRACK") ||
        !NS_IsValidHTTPToken(method)) {
      NS_ConvertUTF8toUTF16 label(method);
      aRv.ThrowTypeError(MSG_INVALID_REQUEST_METHOD, &label);
      return nullptr;
    }

    // Step 14.2
    if (upperCaseMethod.EqualsLiteral("DELETE") ||
        upperCaseMethod.EqualsLiteral("GET") ||
        upperCaseMethod.EqualsLiteral("HEAD") ||
        upperCaseMethod.EqualsLiteral("POST") ||
        upperCaseMethod.EqualsLiteral("PUT") ||
        upperCaseMethod.EqualsLiteral("OPTIONS")) {
      request->SetMethod(upperCaseMethod);
    } else {
      request->SetMethod(method);
    }
  }

  nsRefPtr<InternalHeaders> requestHeaders = request->Headers();

  nsRefPtr<InternalHeaders> headers;
  if (aInit.mHeaders.WasPassed()) {
    nsRefPtr<Headers> h = Headers::Constructor(aGlobal, aInit.mHeaders.Value(), aRv);
    if (aRv.Failed()) {
      return nullptr;
    }
    headers = h->GetInternalHeaders();
  } else {
    headers = new InternalHeaders(*requestHeaders);
  }

  requestHeaders->Clear();

  if (request->Mode() == RequestMode::No_cors) {
    if (!request->HasSimpleMethod()) {
      nsAutoCString method;
      request->GetMethod(method);
      NS_ConvertUTF8toUTF16 label(method);
      aRv.ThrowTypeError(MSG_INVALID_REQUEST_METHOD, &label);
      return nullptr;
    }

    requestHeaders->SetGuard(HeadersGuardEnum::Request_no_cors, aRv);
    if (aRv.Failed()) {
      return nullptr;
    }
  }

  requestHeaders->Fill(*headers, aRv);
  if (aRv.Failed()) {
    return nullptr;
  }

  if (aInit.mBody.WasPassed()) {
    // HEAD and GET are not allowed to have a body.
    nsAutoCString method;
    request->GetMethod(method);
    // method is guaranteed to be uppercase due to step 14.2 above.
    if (method.EqualsLiteral("HEAD") || method.EqualsLiteral("GET")) {
      aRv.ThrowTypeError(MSG_NO_BODY_ALLOWED_FOR_GET_AND_HEAD);
      return nullptr;
    }

    const OwningArrayBufferOrArrayBufferViewOrBlobOrUSVStringOrURLSearchParams& bodyInit = aInit.mBody.Value();
    nsCOMPtr<nsIInputStream> stream;
    nsCString contentType;
    aRv = ExtractByteStreamFromBody(bodyInit,
                                    getter_AddRefs(stream), contentType);
    if (NS_WARN_IF(aRv.Failed())) {
      return nullptr;
    }
    request->SetBody(stream);

    if (!contentType.IsVoid() &&
        !requestHeaders->Has(NS_LITERAL_CSTRING("Content-Type"), aRv)) {
      requestHeaders->Append(NS_LITERAL_CSTRING("Content-Type"),
                             contentType, aRv);
    }

    if (aRv.Failed()) {
      return nullptr;
    }
  }

  nsRefPtr<Request> domRequest = new Request(global, request);
  domRequest->SetMimeType(aRv);
  return domRequest.forget();
}

already_AddRefed<Request>
Request::Clone(ErrorResult& aRv) const
{
  if (BodyUsed()) {
    aRv.ThrowTypeError(MSG_FETCH_BODY_CONSUMED_ERROR);
    return nullptr;
  }

  nsRefPtr<InternalRequest> ir = mRequest->Clone();
  if (!ir) {
    aRv.Throw(NS_ERROR_FAILURE);
    return nullptr;
  }

  nsRefPtr<Request> request = new Request(mOwner, ir);
  return request.forget();
}

Headers*
Request::Headers_()
{
  if (!mHeaders) {
    mHeaders = new Headers(mOwner, mRequest->Headers());
  }

  return mHeaders;
}

} // namespace dom
} // namespace mozilla
