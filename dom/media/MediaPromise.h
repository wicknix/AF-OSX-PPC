/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(MediaPromise_h_)
#define MediaPromise_h_

#include "prlog.h"

#include "nsTArray.h"
#include "nsThreadUtils.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/Maybe.h"
#include "mozilla/Mutex.h"
#include "mozilla/Monitor.h"

/* Polyfill __func__ on MSVC for consumers to pass to the MediaPromise API. */
#ifdef _MSC_VER
#define __func__ __FUNCTION__
#endif

class nsIEventTarget;
namespace mozilla {

extern PRLogModuleInfo* gMediaPromiseLog;

#define PROMISE_LOG(x, ...) \
  MOZ_ASSERT(gMediaPromiseLog); \
  PR_LOG(gMediaPromiseLog, PR_LOG_DEBUG, (x, ##__VA_ARGS__))

class MediaTaskQueue;
namespace detail {

nsresult DispatchMediaPromiseRunnable(MediaTaskQueue* aQueue, nsIRunnable* aRunnable);
nsresult DispatchMediaPromiseRunnable(nsIEventTarget* aTarget, nsIRunnable* aRunnable);

#ifdef DEBUG
void AssertOnThread(MediaTaskQueue* aQueue);
void AssertOnThread(nsIEventTarget* aTarget);
#endif

} // namespace detail

/*
 * A promise manages an asynchronous request that may or may not be able to be
 * fulfilled immediately. When an API returns a promise, the consumer may attach
 * callbacks to be invoked (asynchronously, on a specified thread) when the
 * request is either completed (resolved) or cannot be completed (rejected).
 *
 * When IsExclusive is true, the MediaPromise does a release-mode assertion that
 * there is at most one call to either Then(...) or ChainTo(...).
 */
template<typename T> class MediaPromiseHolder;
template<typename ResolveValueT, typename RejectValueT, bool IsExclusive>
class MediaPromise
{
public:
  typedef ResolveValueT ResolveValueType;
  typedef RejectValueT RejectValueType;

  NS_INLINE_DECL_THREADSAFE_REFCOUNTING(MediaPromise)

protected:
  // MediaPromise is the public type, and never constructed directly. Construct
  // a MediaPromise::Private, defined below.
  explicit MediaPromise(const char* aCreationSite)
    : mCreationSite(aCreationSite)
    , mMutex("MediaPromise Mutex")
    , mHaveConsumer(false)
  {
    PROMISE_LOG("%s creating MediaPromise (%p)", mCreationSite, this);
  }

public:
  // MediaPromise::Private allows us to separate the public interface (upon which
  // consumers of the promise may invoke methods like Then()) from the private
  // interface (upon which the creator of the promise may invoke Resolve() or
  // Reject()). APIs should create and store a MediaPromise::Private (usually
  // via a MediaPromiseHolder), and return a MediaPromise to consumers.
  //
  // NB: We can include the definition of this class inline once B2G ICS is gone.
  class Private;

  static nsRefPtr<MediaPromise>
  CreateAndResolve(ResolveValueType aResolveValue, const char* aResolveSite)
  {
    nsRefPtr<typename MediaPromise::Private> p = new MediaPromise::Private(aResolveSite);
    p->Resolve(aResolveValue, aResolveSite);
    return Move(p);
  }

  static nsRefPtr<MediaPromise>
  CreateAndReject(RejectValueType aRejectValue, const char* aRejectSite)
  {
    nsRefPtr<typename MediaPromise::Private> p = new MediaPromise::Private(aRejectSite);
    p->Reject(aRejectValue, aRejectSite);
    return Move(p);
  }

  class Consumer
  {
  public:
    NS_INLINE_DECL_THREADSAFE_REFCOUNTING(Consumer)

    void Disconnect()
    {
      AssertOnDispatchThread();
      MOZ_DIAGNOSTIC_ASSERT(!mComplete);
      mDisconnected = true;
    }

#ifdef DEBUG
    virtual void AssertOnDispatchThread() = 0;
#else
    void AssertOnDispatchThread() {}
#endif

  protected:
    Consumer() : mComplete(false), mDisconnected(false) {}
    virtual ~Consumer() {}

    bool mComplete;
    bool mDisconnected;
  };

protected:

  /*
   * A ThenValue tracks a single consumer waiting on the promise. When a consumer
   * invokes promise->Then(...), a ThenValue is created. Once the Promise is
   * resolved or rejected, a {Resolve,Reject}Runnable is dispatched, which
   * invokes the resolve/reject method and then deletes the ThenValue.
   */
  class ThenValueBase : public Consumer
  {
  public:
    class ResolveRunnable : public nsRunnable
    {
    public:
      ResolveRunnable(ThenValueBase* aThenValue, ResolveValueType aResolveValue)
        : mThenValue(aThenValue)
        , mResolveValue(aResolveValue) {}

      ~ResolveRunnable()
      {
        MOZ_ASSERT(!mThenValue);
      }

      NS_IMETHODIMP Run()
      {
        PROMISE_LOG("ResolveRunnable::Run() [this=%p]", this);
        mThenValue->DoResolve(mResolveValue);
        mThenValue = nullptr;
        return NS_OK;
      }

    private:
      nsRefPtr<ThenValueBase> mThenValue;
      ResolveValueType mResolveValue;
    };

    class RejectRunnable : public nsRunnable
    {
    public:
      RejectRunnable(ThenValueBase* aThenValue, RejectValueType aRejectValue)
        : mThenValue(aThenValue)
        , mRejectValue(aRejectValue) {}

      ~RejectRunnable()
      {
        MOZ_ASSERT(!mThenValue);
      }

      NS_IMETHODIMP Run()
      {
        PROMISE_LOG("RejectRunnable::Run() [this=%p]", this);
        mThenValue->DoReject(mRejectValue);
        mThenValue = nullptr;
        return NS_OK;
      }

    private:
      nsRefPtr<ThenValueBase> mThenValue;
      RejectValueType mRejectValue;
    };

    explicit ThenValueBase(const char* aCallSite) : mCallSite(aCallSite) {}

    virtual void Dispatch(MediaPromise *aPromise) = 0;

  protected:
    virtual void DoResolve(ResolveValueType aResolveValue) = 0;
    virtual void DoReject(RejectValueType aRejectValue) = 0;

    const char* mCallSite;
  };

  /*
   * We create two overloads for invoking Resolve/Reject Methods so as to
   * make the resolve/reject value argument "optional".
   */

  // Avoid confusing the compiler when the callback accepts T* but the ValueType
  // is nsRefPtr<T>. See bug 1109954 comment 6.
  template <typename T>
  struct NonDeduced
  {
    typedef T type;
  };

  template<typename ThisType, typename ValueType>
  static void InvokeCallbackMethod(ThisType* aThisVal, void(ThisType::*aMethod)(ValueType),
                                   typename NonDeduced<ValueType>::type aValue)
  {
      ((*aThisVal).*aMethod)(aValue);
  }

  template<typename ThisType, typename ValueType>
  static void InvokeCallbackMethod(ThisType* aThisVal, void(ThisType::*aMethod)(), ValueType aValue)
  {
      ((*aThisVal).*aMethod)();
  }

  template<typename TargetType, typename ThisType,
           typename ResolveMethodType, typename RejectMethodType>
  class ThenValue : public ThenValueBase
  {
  public:
    ThenValue(TargetType* aResponseTarget, ThisType* aThisVal,
              ResolveMethodType aResolveMethod, RejectMethodType aRejectMethod,
              const char* aCallSite)
      : ThenValueBase(aCallSite)
      , mResponseTarget(aResponseTarget)
      , mThisVal(aThisVal)
      , mResolveMethod(aResolveMethod)
      , mRejectMethod(aRejectMethod) {}

    void Dispatch(MediaPromise *aPromise) override
    {
      aPromise->mMutex.AssertCurrentThreadOwns();
      MOZ_ASSERT(!aPromise->IsPending());
      bool resolved = aPromise->mResolveValue.isSome();
      nsRefPtr<nsRunnable> runnable =
        resolved ? static_cast<nsRunnable*>(new (typename ThenValueBase::ResolveRunnable)(this, aPromise->mResolveValue.ref()))
                 : static_cast<nsRunnable*>(new (typename ThenValueBase::RejectRunnable)(this, aPromise->mRejectValue.ref()));
      PROMISE_LOG("%s Then() call made from %s [Runnable=%p, Promise=%p, ThenValue=%p]",
                  resolved ? "Resolving" : "Rejecting", ThenValueBase::mCallSite,
                  runnable.get(), aPromise, this);
      DebugOnly<nsresult> rv = detail::DispatchMediaPromiseRunnable(mResponseTarget, runnable);
      MOZ_ASSERT(NS_SUCCEEDED(rv));
    }

#ifdef DEBUG
  virtual void AssertOnDispatchThread() override
  {
    detail::AssertOnThread(mResponseTarget);
  }
#endif

  protected:
    virtual void DoResolve(ResolveValueType aResolveValue) override
    {
      Consumer::mComplete = true;
      if (Consumer::mDisconnected) {
        PROMISE_LOG("ThenValue::DoResolve disconnected - bailing out [this=%p]", this);
        // Null these out for the same reasons described below.
        mResponseTarget = nullptr;
        mThisVal = nullptr;
        return;
      }
      InvokeCallbackMethod(mThisVal.get(), mResolveMethod, aResolveValue);

      // Null these out after invoking the callback so that any references are
      // released predictably on the target thread. Otherwise, they would be
      // released on whatever thread last drops its reference to the ThenValue,
      // which may or may not be ok.
      mResponseTarget = nullptr;
      mThisVal = nullptr;
    }

    virtual void DoReject(RejectValueType aRejectValue) override
    {
      Consumer::mComplete = true;
      if (Consumer::mDisconnected) {
        PROMISE_LOG("ThenValue::DoReject disconnected - bailing out [this=%p]", this);
        // Null these out for the same reasons described below.
        mResponseTarget = nullptr;
        mThisVal = nullptr;
        return;
      }
      InvokeCallbackMethod(mThisVal.get(), mRejectMethod, aRejectValue);

      // Null these out after invoking the callback so that any references are
      // released predictably on the target thread. Otherwise, they would be
      // released on whatever thread last drops its reference to the ThenValue,
      // which may or may not be ok.
      mResponseTarget = nullptr;
      mThisVal = nullptr;
    }

  private:
    nsRefPtr<TargetType> mResponseTarget;
    nsRefPtr<ThisType> mThisVal;
    ResolveMethodType mResolveMethod;
    RejectMethodType mRejectMethod;
  };
public:

  template<typename TargetType, typename ThisType,
           typename ResolveMethodType, typename RejectMethodType>
  already_AddRefed<Consumer> RefableThen(TargetType* aResponseTarget, const char* aCallSite, ThisType* aThisVal,
                                         ResolveMethodType aResolveMethod, RejectMethodType aRejectMethod)
  {
    MutexAutoLock lock(mMutex);
    MOZ_DIAGNOSTIC_ASSERT(!IsExclusive || !mHaveConsumer);
    mHaveConsumer = true;
    nsRefPtr<ThenValueBase> thenValue = new ThenValue<TargetType, ThisType, ResolveMethodType,
                                                      RejectMethodType>(aResponseTarget, aThisVal,
                                                                        aResolveMethod, aRejectMethod,
                                                                        aCallSite);
    PROMISE_LOG("%s invoking Then() [this=%p, thenValue=%p, aThisVal=%p, isPending=%d]",
                aCallSite, this, thenValue.get(), aThisVal, (int) IsPending());
    if (!IsPending()) {
      thenValue->Dispatch(this);
    } else {
      mThenValues.AppendElement(thenValue);
    }

    return thenValue.forget();
  }

  template<typename TargetType, typename ThisType,
           typename ResolveMethodType, typename RejectMethodType>
  void Then(TargetType* aResponseTarget, const char* aCallSite, ThisType* aThisVal,
            ResolveMethodType aResolveMethod, RejectMethodType aRejectMethod)
  {
    nsRefPtr<Consumer> c =
      RefableThen(aResponseTarget, aCallSite, aThisVal, aResolveMethod, aRejectMethod);
    return;
  }

  void ChainTo(already_AddRefed<Private> aChainedPromise, const char* aCallSite)
  {
    MutexAutoLock lock(mMutex);
    MOZ_DIAGNOSTIC_ASSERT(!IsExclusive || !mHaveConsumer);
    mHaveConsumer = true;
    nsRefPtr<Private> chainedPromise = aChainedPromise;
    PROMISE_LOG("%s invoking Chain() [this=%p, chainedPromise=%p, isPending=%d]",
                aCallSite, this, chainedPromise.get(), (int) IsPending());
    if (!IsPending()) {
      ForwardTo(chainedPromise);
    } else {
      mChainedPromises.AppendElement(chainedPromise);
    }
  }

protected:
  bool IsPending() { return mResolveValue.isNothing() && mRejectValue.isNothing(); }
  void DispatchAll()
  {
    mMutex.AssertCurrentThreadOwns();
    for (size_t i = 0; i < mThenValues.Length(); ++i) {
      mThenValues[i]->Dispatch(this);
    }
    mThenValues.Clear();

    for (size_t i = 0; i < mChainedPromises.Length(); ++i) {
      ForwardTo(mChainedPromises[i]);
    }
    mChainedPromises.Clear();
  }

  void ForwardTo(Private* aOther)
  {
    MOZ_ASSERT(!IsPending());
    if (mResolveValue.isSome()) {
      aOther->Resolve(mResolveValue.ref(), "<chained promise>");
    } else {
      aOther->Reject(mRejectValue.ref(), "<chained promise>");
    }
  }

  ~MediaPromise()
  {
    PROMISE_LOG("MediaPromise::~MediaPromise [this=%p]", this);
    MOZ_ASSERT(!IsPending());
    MOZ_ASSERT(mThenValues.IsEmpty());
    MOZ_ASSERT(mChainedPromises.IsEmpty());
  };

  const char* mCreationSite; // For logging
  Mutex mMutex;
  Maybe<ResolveValueType> mResolveValue;
  Maybe<RejectValueType> mRejectValue;
  nsTArray<nsRefPtr<ThenValueBase>> mThenValues;
  nsTArray<nsRefPtr<Private>> mChainedPromises;
  bool mHaveConsumer;
};

template<typename ResolveValueT, typename RejectValueT, bool IsExclusive>
class MediaPromise<ResolveValueT, RejectValueT, IsExclusive>::Private
  : public MediaPromise<ResolveValueT, RejectValueT, IsExclusive>
{
public:
  explicit Private(const char* aCreationSite) : MediaPromise(aCreationSite) {}

  void Resolve(ResolveValueT aResolveValue, const char* aResolveSite)
  {
    MutexAutoLock lock(mMutex);
    MOZ_ASSERT(IsPending());
    PROMISE_LOG("%s resolving MediaPromise (%p created at %s)", aResolveSite, this, mCreationSite);
    mResolveValue.emplace(aResolveValue);
    DispatchAll();
  }

  void Reject(RejectValueT aRejectValue, const char* aRejectSite)
  {
    MutexAutoLock lock(mMutex);
    MOZ_ASSERT(IsPending());
    PROMISE_LOG("%s rejecting MediaPromise (%p created at %s)", aRejectSite, this, mCreationSite);
    mRejectValue.emplace(aRejectValue);
    DispatchAll();
  }
};

/*
 * Class to encapsulate a promise for a particular role. Use this as the member
 * variable for a class whose method returns a promise.
 */
template<typename PromiseType>
class MediaPromiseHolder
{
public:
  MediaPromiseHolder()
    : mMonitor(nullptr) {}

  ~MediaPromiseHolder() { MOZ_ASSERT(!mPromise); }

  already_AddRefed<PromiseType> Ensure(const char* aMethodName) {
    if (mMonitor) {
      mMonitor->AssertCurrentThreadOwns();
    }
    if (!mPromise) {
      mPromise = new (typename PromiseType::Private)(aMethodName);
    }
    nsRefPtr<PromiseType> p = mPromise.get();
    return p.forget();
  }

  // Provide a Monitor that should always be held when accessing this instance.
  void SetMonitor(Monitor* aMonitor) { mMonitor = aMonitor; }

  bool IsEmpty()
  {
    if (mMonitor) {
      mMonitor->AssertCurrentThreadOwns();
    }
    return !mPromise;
  }

  already_AddRefed<typename PromiseType::Private> Steal()
  {
    if (mMonitor) {
      mMonitor->AssertCurrentThreadOwns();
    }

    nsRefPtr<typename PromiseType::Private> p = mPromise;
    mPromise = nullptr;
    return p.forget();
  }

  void Resolve(typename PromiseType::ResolveValueType aResolveValue,
               const char* aMethodName)
  {
    if (mMonitor) {
      mMonitor->AssertCurrentThreadOwns();
    }
    MOZ_ASSERT(mPromise);
    mPromise->Resolve(aResolveValue, aMethodName);
    mPromise = nullptr;
  }

  void ResolveIfExists(typename PromiseType::ResolveValueType aResolveValue,
                       const char* aMethodName)
  {
    if (!IsEmpty()) {
      Resolve(aResolveValue, aMethodName);
    }
  }

  void Reject(typename PromiseType::RejectValueType aRejectValue,
              const char* aMethodName)
  {
    if (mMonitor) {
      mMonitor->AssertCurrentThreadOwns();
    }
    MOZ_ASSERT(mPromise);
    mPromise->Reject(aRejectValue, aMethodName);
    mPromise = nullptr;
  }

  void RejectIfExists(typename PromiseType::RejectValueType aRejectValue,
                      const char* aMethodName)
  {
    if (!IsEmpty()) {
      Reject(aRejectValue, aMethodName);
    }
  }

private:
  Monitor* mMonitor;
  nsRefPtr<typename PromiseType::Private> mPromise;
};

/*
 * Class to encapsulate a MediaPromise::Consumer reference. Use this as the member
 * variable for a class waiting on a media promise.
 */
template<typename PromiseType>
class MediaPromiseConsumerHolder
{
public:
  MediaPromiseConsumerHolder() {}
  ~MediaPromiseConsumerHolder() { MOZ_ASSERT(!mConsumer); }

  void Begin(already_AddRefed<typename PromiseType::Consumer> aConsumer)
  {
    MOZ_DIAGNOSTIC_ASSERT(!Exists());
    mConsumer = aConsumer;
  }

  void Complete()
  {
    MOZ_DIAGNOSTIC_ASSERT(Exists());
    mConsumer = nullptr;
  }

  // Disconnects and forgets an outstanding promise. The resolve/reject methods
  // will never be called.
  void Disconnect() {
    MOZ_ASSERT(Exists());
    mConsumer->Disconnect();
    mConsumer = nullptr;
  }

  void DisconnectIfExists() {
    if (Exists()) {
      Disconnect();
    }
  }

  bool Exists() { return !!mConsumer; }

private:
  nsRefPtr<typename PromiseType::Consumer> mConsumer;
};

// Proxy Media Calls.
//
// This machinery allows callers to schedule a promise-returning method to be
// invoked asynchronously on a given thread, while at the same time receiving
// a promise upon which to invoke Then() immediately. ProxyMediaCall dispatches
// a task to invoke the method on the proper thread and also chain the resulting
// promise to the one that the caller received, so that resolve/reject values
// are forwarded through.

namespace detail {

template<typename PromiseType>
class MethodCallBase
{
public:
  MethodCallBase() { MOZ_COUNT_CTOR(MethodCallBase); }
  virtual nsRefPtr<PromiseType> Invoke() = 0;
  virtual ~MethodCallBase() { MOZ_COUNT_DTOR(MethodCallBase); };
};

template<typename PromiseType, typename ThisType>
class MethodCallWithNoArgs : public MethodCallBase<PromiseType>
{
public:
  typedef nsRefPtr<PromiseType>(ThisType::*Type)();
  MethodCallWithNoArgs(ThisType* aThisVal, Type aMethod)
    : mThisVal(aThisVal), mMethod(aMethod) {}
  nsRefPtr<PromiseType> Invoke() override { return ((*mThisVal).*mMethod)(); }
protected:
  nsRefPtr<ThisType> mThisVal;
  Type mMethod;
};

// NB: MethodCallWithOneArg definition should go here, if/when it is needed.

template<typename PromiseType, typename ThisType, typename Arg1Type, typename Arg2Type>
class MethodCallWithTwoArgs : public MethodCallBase<PromiseType>
{
public:
  typedef nsRefPtr<PromiseType>(ThisType::*Type)(Arg1Type, Arg2Type);
  MethodCallWithTwoArgs(ThisType* aThisVal, Type aMethod, Arg1Type aArg1, Arg2Type aArg2)
    : mThisVal(aThisVal), mMethod(aMethod), mArg1(aArg1), mArg2(aArg2) {}
  nsRefPtr<PromiseType> Invoke() override { return ((*mThisVal).*mMethod)(mArg1, mArg2); }
protected:
  nsRefPtr<ThisType> mThisVal;
  Type mMethod;
  Arg1Type mArg1;
  Arg2Type mArg2;
};

template<typename PromiseType>
class ProxyRunnable : public nsRunnable
{
public:
  ProxyRunnable(typename PromiseType::Private* aProxyPromise, MethodCallBase<PromiseType>* aMethodCall)
    : mProxyPromise(aProxyPromise), mMethodCall(aMethodCall) {}

  NS_IMETHODIMP Run()
  {
    nsRefPtr<PromiseType> p = mMethodCall->Invoke();
    mMethodCall = nullptr;
    p->ChainTo(mProxyPromise.forget(), "<Proxy Promise>");
    return NS_OK;
  }

private:
  nsRefPtr<typename PromiseType::Private> mProxyPromise;
  nsAutoPtr<MethodCallBase<PromiseType>> mMethodCall;
};

template<typename PromiseType, typename TargetType>
static nsRefPtr<PromiseType>
ProxyInternal(TargetType* aTarget, MethodCallBase<PromiseType>* aMethodCall, const char* aCallerName)
{
  nsRefPtr<typename PromiseType::Private> p = new (typename PromiseType::Private)(aCallerName);
  nsRefPtr<ProxyRunnable<PromiseType>> r = new ProxyRunnable<PromiseType>(p, aMethodCall);
  nsresult rv = detail::DispatchMediaPromiseRunnable(aTarget, r);
  MOZ_DIAGNOSTIC_ASSERT(NS_SUCCEEDED(rv));
  (void) rv; // Avoid compilation failures in builds with MOZ_DIAGNOSTIC_ASSERT disabled.
  return Move(p);
}

} // namespace detail

template<typename PromiseType, typename TargetType, typename ThisType>
static nsRefPtr<PromiseType>
ProxyMediaCall(TargetType* aTarget, ThisType* aThisVal, const char* aCallerName,
               nsRefPtr<PromiseType>(ThisType::*aMethod)())
{
  typedef detail::MethodCallWithNoArgs<PromiseType, ThisType> MethodCallType;
  MethodCallType* methodCall = new MethodCallType(aThisVal, aMethod);
  return detail::ProxyInternal(aTarget, methodCall, aCallerName);
}

// NB: One-arg overload should go here, if/when it is needed.

template<typename PromiseType, typename TargetType, typename ThisType,
         typename Arg1Type, typename Arg2Type>
static nsRefPtr<PromiseType>
ProxyMediaCall(TargetType* aTarget, ThisType* aThisVal, const char* aCallerName,
               nsRefPtr<PromiseType>(ThisType::*aMethod)(Arg1Type, Arg2Type), Arg1Type aArg1, Arg2Type aArg2)
{
  typedef detail::MethodCallWithTwoArgs<PromiseType, ThisType, Arg1Type, Arg2Type> MethodCallType;
  MethodCallType* methodCall = new MethodCallType(aThisVal, aMethod, aArg1, aArg2);
  return detail::ProxyInternal(aTarget, methodCall, aCallerName);
}

#undef PROMISE_LOG

} // namespace mozilla

#endif
