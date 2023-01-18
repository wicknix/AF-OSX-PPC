/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef GMPService_h_
#define GMPService_h_

#include "nsString.h"
#include "mozIGeckoMediaPluginService.h"
#include "nsIObserver.h"
#include "nsTArray.h"
#include "mozilla/Attributes.h"
#include "mozilla/Monitor.h"
#include "nsString.h"
#include "nsCOMPtr.h"
#include "nsIThread.h"
#include "nsThreadUtils.h"
#include "nsITimer.h"
#include "nsClassHashtable.h"
#include "nsDataHashtable.h"
#include "mozilla/Atomics.h"

template <class> struct already_AddRefed;

namespace mozilla {
namespace gmp {

class GMPParent;

#define GMP_DEFAULT_ASYNC_SHUTDONW_TIMEOUT 3000

class GeckoMediaPluginService final : public mozIGeckoMediaPluginService
                                        , public nsIObserver
{
public:
  static already_AddRefed<GeckoMediaPluginService> GetGeckoMediaPluginService();

  GeckoMediaPluginService();
  nsresult Init();

  NS_DECL_THREADSAFE_ISUPPORTS
  NS_DECL_MOZIGECKOMEDIAPLUGINSERVICE
  NS_DECL_NSIOBSERVER

  void AsyncShutdownNeeded(GMPParent* aParent);
  void AsyncShutdownComplete(GMPParent* aParent);
  void AbortAsyncShutdown();

  int32_t AsyncShutdownTimeoutMs();

  class PluginCrashCallback
  {
  public:
    NS_INLINE_DECL_REFCOUNTING(PluginCrashCallback)

    PluginCrashCallback(const nsACString& aPluginId)
      : mPluginId(aPluginId)
    {
      MOZ_ASSERT(NS_IsMainThread());
    }
    const nsACString& PluginId() const { return mPluginId; }
    virtual void Run(const nsACString& aPluginName, const nsAString& aPluginDumpId) = 0;
    virtual bool IsStillValid() = 0; // False if callback has become useless.
  protected:
    virtual ~PluginCrashCallback()
    {
      MOZ_ASSERT(NS_IsMainThread());
    }
  private:
    const nsCString mPluginId;
  };
  void RemoveObsoletePluginCrashCallbacks(); // Called from add/remove/run.
  void AddPluginCrashCallback(nsRefPtr<PluginCrashCallback> aPluginCrashCallback);
  void RemovePluginCrashCallbacks(const nsACString& aPluginId);
  void RunPluginCrashCallbacks(const nsACString& aPluginId,
                               const nsACString& aPluginName,
                               const nsAString& aPluginDumpId);

private:
  ~GeckoMediaPluginService();

  nsresult GMPDispatch(nsIRunnable* event, uint32_t flags = NS_DISPATCH_NORMAL);

  void ClearStorage();

  GMPParent* SelectPluginForAPI(const nsACString& aNodeId,
                                const nsCString& aAPI,
                                const nsTArray<nsCString>& aTags);
  GMPParent* FindPluginForAPIFrom(size_t aSearchStartIndex,
                                  const nsCString& aAPI,
                                  const nsTArray<nsCString>& aTags,
                                  size_t* aOutPluginIndex);

  void UnloadPlugins();
  void CrashPlugins();
  void SetAsyncShutdownComplete();

  void LoadFromEnvironment();
  void ProcessPossiblePlugin(nsIFile* aDir);

  void AddOnGMPThread(const nsAString& aDirectory);
  void RemoveOnGMPThread(const nsAString& aDirectory,
                         const bool aDeleteFromDisk,
                         const bool aCanDefer);

  nsresult SetAsyncShutdownTimeout();

  struct DirectoryFilter {
    virtual bool operator()(nsIFile* aPath) = 0;
    ~DirectoryFilter() {}
  };
  void ClearNodeIdAndPlugin(DirectoryFilter& aFilter);

  void ForgetThisSiteOnGMPThread(const nsACString& aOrigin);
  void ClearRecentHistoryOnGMPThread(PRTime aSince);

protected:
  friend class GMPParent;
  void ReAddOnGMPThread(const nsRefPtr<GMPParent>& aOld);
  void PluginTerminated(const nsRefPtr<GMPParent>& aOld);
private:
  GMPParent* ClonePlugin(const GMPParent* aOriginal);
  nsresult EnsurePluginsOnDiskScanned();

  class PathRunnable : public nsRunnable
  {
  public:
    enum EOperation {
      ADD,
      REMOVE,
      REMOVE_AND_DELETE_FROM_DISK,
    };

    PathRunnable(GeckoMediaPluginService* aService, const nsAString& aPath,
                 EOperation aOperation, bool aDefer = false)
      : mService(aService)
      , mPath(aPath)
      , mOperation(aOperation)
      , mDefer(aDefer)
    { }

    NS_DECL_NSIRUNNABLE

  private:
    nsRefPtr<GeckoMediaPluginService> mService;
    nsString mPath;
    EOperation mOperation;
    bool mDefer;
  };

  Mutex mMutex; // Protects mGMPThread and mShuttingDown and mPlugins
  nsTArray<nsRefPtr<GMPParent>> mPlugins;
  nsCOMPtr<nsIThread> mGMPThread;
  bool mShuttingDown;
  bool mShuttingDownOnGMPThread;

  nsTArray<nsRefPtr<PluginCrashCallback>> mPluginCrashCallbacks;

  // True if we've inspected MOZ_GMP_PATH on the GMP thread and loaded any
  // plugins found there into mPlugins.
  Atomic<bool> mScannedPluginOnDisk;

  template<typename T>
  class MainThreadOnly {
  public:
    MOZ_IMPLICIT MainThreadOnly(T aValue)
      : mValue(aValue)
    {}
    operator T&() {
      MOZ_ASSERT(NS_IsMainThread());
      return mValue;
    }

  private:
    T mValue;
  };

  MainThreadOnly<bool> mWaitingForPluginsAsyncShutdown;

  nsTArray<nsRefPtr<GMPParent>> mAsyncShutdownPlugins; // GMP Thread only.

  nsTArray<nsString> mPluginsWaitingForDeletion;

#ifndef MOZ_WIDGET_GONK
  nsCOMPtr<nsIFile> mStorageBaseDir;
#endif

  // Hashes of (origin,topLevelOrigin) to the node id for
  // non-persistent sessions.
  nsClassHashtable<nsUint32HashKey, nsCString> mTempNodeIds;

  // Hashes node id to whether that node id is allowed to store data
  // persistently on disk.
  nsDataHashtable<nsCStringHashKey, bool> mPersistentStorageAllowed;
};

nsresult ReadSalt(nsIFile* aPath, nsACString& aOutData);
bool MatchOrigin(nsIFile* aPath, const nsACString& aSite);

} // namespace gmp
} // namespace mozilla

#endif // GMPService_h_
