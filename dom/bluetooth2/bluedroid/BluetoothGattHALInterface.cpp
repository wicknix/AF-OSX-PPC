/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "BluetoothGattHALInterface.h"
#include "BluetoothHALHelpers.h"

BEGIN_BLUETOOTH_NAMESPACE

typedef
  BluetoothHALInterfaceRunnable0<BluetoothGattClientResultHandler, void>
  BluetoothGattClientHALResultRunnable;

typedef
  BluetoothHALInterfaceRunnable1<BluetoothGattClientResultHandler, void,
                                 BluetoothStatus, BluetoothStatus>
  BluetoothGattClientHALErrorRunnable;

typedef
  BluetoothHALInterfaceRunnable0<BluetoothGattResultHandler, void>
  BluetoothGattHALResultRunnable;

typedef
  BluetoothHALInterfaceRunnable1<BluetoothGattResultHandler, void,
                                 BluetoothStatus, BluetoothStatus>
  BluetoothGattHALErrorRunnable;

static nsresult
DispatchBluetoothGattClientHALResult(
  BluetoothGattClientResultHandler* aRes,
  void (BluetoothGattClientResultHandler::*aMethod)(),
  BluetoothStatus aStatus)
{
  MOZ_ASSERT(aRes);

  nsRunnable* runnable;

  if (aStatus == STATUS_SUCCESS) {
    runnable = new BluetoothGattClientHALResultRunnable(aRes, aMethod);
  } else {
    runnable = new BluetoothGattClientHALErrorRunnable(aRes,
      &BluetoothGattClientResultHandler::OnError, aStatus);
  }
  nsresult rv = NS_DispatchToMainThread(runnable);
  if (NS_FAILED(rv)) {
    BT_WARNING("NS_DispatchToMainThread failed: %X", rv);
  }
  return rv;
}

static nsresult
DispatchBluetoothGattHALResult(
  BluetoothGattResultHandler* aRes,
  void (BluetoothGattResultHandler::*aMethod)(),
  BluetoothStatus aStatus)
{
  MOZ_ASSERT(aRes);

  nsRunnable* runnable;

  if (aStatus == STATUS_SUCCESS) {
    runnable = new BluetoothGattHALResultRunnable(aRes, aMethod);
  } else {
    runnable = new BluetoothGattHALErrorRunnable(aRes,
      &BluetoothGattResultHandler::OnError, aStatus);
  }
  nsresult rv = NS_DispatchToMainThread(runnable);
  if (NS_FAILED(rv)) {
    BT_WARNING("NS_DispatchToMainThread failed: %X", rv);
  }
  return rv;
}

// Notification Handling
//

static BluetoothGattNotificationHandler* sGattNotificationHandler;

struct BluetoothGattClientCallback
{
  class GattClientNotificationHandlerWrapper
  {
  public:
    typedef BluetoothGattClientNotificationHandler ObjectType;

    static ObjectType* GetInstance()
    {
      MOZ_ASSERT(NS_IsMainThread());

      return sGattNotificationHandler;
    }
  };

  // Notifications

  // GATT Client Notification
  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothUuid,
    int, int, const BluetoothUuid&>
    RegisterClientNotification;

  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    nsString, int, BluetoothGattAdvData,
    const nsAString&, int, const BluetoothGattAdvData&>
    ScanResultNotification;

  typedef BluetoothNotificationHALRunnable4<
    GattClientNotificationHandlerWrapper, void,
    int, int, int, nsString,
    int, int, int, const nsAString&>
    ConnectNotification;

  typedef BluetoothNotificationHALRunnable4<
    GattClientNotificationHandlerWrapper, void,
    int, int, int, nsString,
    int, int, int, const nsAString&>
    DisconnectNotification;

  typedef BluetoothNotificationHALRunnable2<
    GattClientNotificationHandlerWrapper, void,
    int, int>
    SearchCompleteNotification;

  typedef BluetoothNotificationHALRunnable2<
    GattClientNotificationHandlerWrapper, void,
    int, BluetoothGattServiceId,
    int, const BluetoothGattServiceId&>
    SearchResultNotification;

  typedef BluetoothNotificationHALRunnable5<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattServiceId, BluetoothGattId, int,
    int, int, const BluetoothGattServiceId&, const BluetoothGattId&>
    GetCharacteristicNotification;

  typedef BluetoothNotificationHALRunnable5<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattServiceId,
    BluetoothGattId, BluetoothGattId,
    int, int, const BluetoothGattServiceId&,
    const BluetoothGattId&, const BluetoothGattId&>
    GetDescriptorNotification;

  typedef BluetoothNotificationHALRunnable4<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattServiceId, BluetoothGattServiceId,
    int, int, const BluetoothGattServiceId&, const BluetoothGattServiceId&>
    GetIncludedServiceNotification;

  typedef BluetoothNotificationHALRunnable5<
    GattClientNotificationHandlerWrapper, void,
    int, int, int,
    BluetoothGattServiceId, BluetoothGattId,
    int, int, int,
    const BluetoothGattServiceId&, const BluetoothGattId&>
    RegisterNotificationNotification;

  typedef BluetoothNotificationHALRunnable2<
    GattClientNotificationHandlerWrapper, void,
    int, BluetoothGattNotifyParam,
    int, const BluetoothGattNotifyParam&>
    NotifyNotification;

  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattReadParam,
    int, int, const BluetoothGattReadParam&>
    ReadCharacteristicNotification;

  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattWriteParam,
    int, int, const BluetoothGattWriteParam&>
    WriteCharacteristicNotification;

  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattReadParam,
    int, int, const BluetoothGattReadParam&>
    ReadDescriptorNotification;

  typedef BluetoothNotificationHALRunnable3<
    GattClientNotificationHandlerWrapper, void,
    int, int, BluetoothGattWriteParam,
    int, int, const BluetoothGattWriteParam&>
    WriteDescriptorNotification;

  typedef BluetoothNotificationHALRunnable2<
    GattClientNotificationHandlerWrapper, void,
    int, int>
    ExecuteWriteNotification;

  typedef BluetoothNotificationHALRunnable4<
    GattClientNotificationHandlerWrapper, void,
    int, nsString, int, int,
    int, const nsAString&, int, int>
    ReadRemoteRssiNotification;

  typedef BluetoothNotificationHALRunnable2<
    GattClientNotificationHandlerWrapper, void,
    int, int>
    ListenNotification;

  // Bluedroid GATT client callbacks
#if ANDROID_VERSION >= 19
  static void
  RegisterClient(int aStatus, int aClientIf, bt_uuid_t* aAppUuid)
  {
    RegisterClientNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::RegisterClientNotification,
      aStatus, aClientIf, *aAppUuid);
  }

  static void
  ScanResult(bt_bdaddr_t* aBdAddr, int aRssi, uint8_t* aAdvData)
  {
    ScanResultNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ScanResultNotification,
      aBdAddr, aRssi, aAdvData);
  }

  static void
  Connect(int aConnId, int aStatus, int aClientIf, bt_bdaddr_t* aBdAddr)
  {
    ConnectNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ConnectNotification,
      aConnId, aStatus, aClientIf, aBdAddr);
  }

  static void
  Disconnect(int aConnId, int aStatus, int aClientIf, bt_bdaddr_t* aBdAddr)
  {
    DisconnectNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::DisconnectNotification,
      aConnId, aStatus, aClientIf, aBdAddr);
  }

  static void
  SearchComplete(int aConnId, int aStatus)
  {
    SearchCompleteNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::SearchCompleteNotification,
      aConnId, aStatus);
  }

  static void
  SearchResult(int aConnId, btgatt_srvc_id_t* aServiceId)
  {
    SearchResultNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::SearchResultNotification,
      aConnId, *aServiceId);
  }

  static void
  GetCharacteristic(int aConnId, int aStatus,
                    btgatt_srvc_id_t* aServiceId,
                    btgatt_gatt_id_t* aCharId,
                    int aCharProperty)
  {
    GetCharacteristicNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::GetCharacteristicNotification,
      aConnId, aStatus, *aServiceId, *aCharId, aCharProperty);
  }

  static void
  GetDescriptor(int aConnId, int aStatus,
                btgatt_srvc_id_t* aServiceId,
                btgatt_gatt_id_t* aCharId,
                btgatt_gatt_id_t* aDescriptorId)
  {
    GetDescriptorNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::GetDescriptorNotification,
      aConnId, aStatus, *aServiceId, *aCharId, *aDescriptorId);
  }

  static void
  GetIncludedService(int aConnId, int aStatus,
                     btgatt_srvc_id_t* aServiceId,
                     btgatt_srvc_id_t* aIncludedServiceId)
  {
    GetIncludedServiceNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::GetIncludedServiceNotification,
      aConnId, aStatus, *aServiceId, *aIncludedServiceId);
  }

  static void
  RegisterNotification(int aConnId, int aIsRegister, int aStatus,
                       btgatt_srvc_id_t* aServiceId,
                       btgatt_gatt_id_t* aCharId)
  {
    RegisterNotificationNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::RegisterNotificationNotification,
      aConnId, aIsRegister, aStatus, *aServiceId, *aCharId);
  }

  static void
  Notify(int aConnId, btgatt_notify_params_t* aParam)
  {
    NotifyNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::NotifyNotification,
      aConnId, *aParam);
  }

  static void
  ReadCharacteristic(int aConnId, int aStatus, btgatt_read_params_t* aParam)
  {
    ReadCharacteristicNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ReadCharacteristicNotification,
      aConnId, aStatus, *aParam);
  }

  static void
  WriteCharacteristic(int aConnId, int aStatus, btgatt_write_params_t* aParam)
  {
    WriteCharacteristicNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::WriteCharacteristicNotification,
      aConnId, aStatus, *aParam);
  }

  static void
  ReadDescriptor(int aConnId, int aStatus, btgatt_read_params_t* aParam)
  {
    ReadDescriptorNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ReadDescriptorNotification,
      aConnId, aStatus, *aParam);
  }

  static void
  WriteDescriptor(int aConnId, int aStatus, btgatt_write_params_t* aParam)
  {
    WriteDescriptorNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::WriteDescriptorNotification,
      aConnId, aStatus, *aParam);
  }

  static void
  ExecuteWrite(int aConnId, int aStatus)
  {
    ExecuteWriteNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ExecuteWriteNotification,
      aConnId, aStatus);
  }

  static void
  ReadRemoteRssi(int aClientIf, bt_bdaddr_t* aBdAddr, int aRssi, int aStatus)
  {
    ReadRemoteRssiNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ReadRemoteRssiNotification,
      aClientIf, aBdAddr, aRssi, aStatus);
  }

  static void
  Listen(int aStatus, int aServerIf)
  {
    ListenNotification::Dispatch(
      &BluetoothGattClientNotificationHandler::ListenNotification,
      aStatus, aServerIf);
  }
#endif // ANDROID_VERSION >= 19
};

struct BluetoothGattServerCallback
{
  class GattServerNotificationHandlerWrapper
  {
  public:
    typedef BluetoothGattServerNotificationHandler ObjectType;

    static ObjectType* GetInstance()
    {
      MOZ_ASSERT(NS_IsMainThread());

      return sGattNotificationHandler;
    }
  };

  // Notifications
  // TODO: Add Server Notifications

  // GATT Server callbacks
  // TODO: Implement server callbacks

#if ANDROID_VERSION >= 19
  static void
  RegisterServer(int aStatus, int aServerIf, bt_uuid_t* aAppUuid)
  { }

  static void
  Connection(int aConnId, int aServerIf, int aIsConnected,
             bt_bdaddr_t* aBdAddr)
  { }

  static void
  ServiceAdded(int aStatus, int aServerIf, btgatt_srvc_id_t* aServiceId,
               int aServiceHandle)
  { }

  static void
  IncludedServiceAdded(int aStatus, int aServerIf, int aServiceHandle,
                       int aIncludedServiceHandle)
  { }

  static void
  CharacteristicAdded(int aStatus, int aServerIf, bt_uuid_t* aUuid,
                      int aServiceHandle, int aCharHandle)
  { }

  static void
  DescriptorAdded(int aStatus, int aServerIf, bt_uuid_t* aUuid,
                  int aServiceHandle, int aDescriptorHandle)
  { }

  static void
  ServiceStarted(int aStatus, int aServerIf, int aServiceHandle)
  { }

  static void
  ServiceStopped(int aStatus, int aServerIf, int aServiceHandle)
  { }

  static void
  ServiceDeleted(int aStatus, int aServerIf, int aServiceHandle)
  { }

  static void
  RequestRead(int aConnId, int aTransId, bt_bdaddr_t* aBdAddr,
              int aAttrHandle, int aOffset, bool aIsLong)
  { }

  static void
  RequestWrite(int aConnId, int aTransId, bt_bdaddr_t* aBdAddr,
               int aAttrHandle, int aOffset, int aLength,
               bool aNeedRsp, bool aIsPrep, uint8_t* aValue)
  { }

  static void
  RequestExecWrite(int aConnId, int aTransId, bt_bdaddr_t* aBdAddr,
                   int aExecWrite)
  { }

  static void
  ResponseConfirmation(int aStatus, int aHandle)
  { }
#endif // ANDROID_VERSION >= 19
};

// GATT Client Interface

BluetoothGattClientHALInterface::BluetoothGattClientHALInterface(
#if ANDROID_VERSION >= 19
  const btgatt_client_interface_t* aInterface
#endif
  )
#if ANDROID_VERSION >= 19
  :mInterface(aInterface)
#endif
{
#if ANDROID_VERSION >= 19
  MOZ_ASSERT(mInterface);
#endif
}

BluetoothGattClientHALInterface::~BluetoothGattClientHALInterface()
{ }

void
BluetoothGattClientHALInterface::RegisterClient(
  const BluetoothUuid& aUuid, BluetoothGattClientResultHandler* aRes)
{
  int status;

#if ANDROID_VERSION >= 19
  bt_uuid_t uuid;
  if (NS_SUCCEEDED(Convert(aUuid, uuid))) {
    status = mInterface->register_client(&uuid);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif
  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::RegisterClient,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::UnregisterClient(
  int aClientIf, BluetoothGattClientResultHandler* aRes)
{
#if ANDROID_VERSION >= 19
  int status = mInterface->unregister_client(aClientIf);
#else
  int status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::UnregisterClient,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::Scan(
  int aClientIf, bool aStart, BluetoothGattClientResultHandler* aRes)
{
#if ANDROID_VERSION >= 19
  int status = mInterface->scan(aClientIf, aStart);
#else
  int status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::Scan,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::Connect(
  int aClientIf, const nsAString& aBdAddr,
  bool aIsDirect, BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr))) {
    status = mInterface->connect(aClientIf, &bdAddr, aIsDirect);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::Connect,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::Disconnect(
  int aClientIf, const nsAString& aBdAddr,
  int aConnId, BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr))) {
    status = mInterface->disconnect(aClientIf, &bdAddr, aConnId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::Disconnect,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::Listen(
  int aClientIf, bool aIsStart, BluetoothGattClientResultHandler* aRes)
{
#if ANDROID_VERSION >= 19
  bt_status_t status = mInterface->listen(aClientIf, aIsStart);
#else
  bt_status_t status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::Listen,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::Refresh(
  int aClientIf, const nsAString& aBdAddr,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr))) {
    status = mInterface->refresh(aClientIf, &bdAddr);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::Refresh,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::SearchService(
  int aConnId, const BluetoothUuid& aUuid,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_uuid_t uuid;

  if (NS_SUCCEEDED(Convert(aUuid, uuid))) {
    status = mInterface->search_service(aConnId, &uuid);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::SearchService,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::GetIncludedService(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattServiceId& aStartServiceId,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_srvc_id_t startServiceId;

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aStartServiceId, startServiceId))) {
    status = mInterface->get_included_service(aConnId, &serviceId,
                                              &startServiceId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::GetIncludedService,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::GetCharacteristic(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aStartCharId,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t startCharId;

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aStartCharId, startCharId))) {
    status = mInterface->get_characteristic(aConnId, &serviceId, &startCharId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::GetCharacteristic,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::GetDescriptor(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId,
  const BluetoothGattId& aDescriptorId,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;
  btgatt_gatt_id_t descriptorId;

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId)) &&
      NS_SUCCEEDED(Convert(aDescriptorId, descriptorId))) {
    status = mInterface->get_descriptor(aConnId, &serviceId, &charId,
                                        &descriptorId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::GetDescriptor,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::ReadCharacteristic(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId, int aAuthReq,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId))) {
    status = mInterface->read_characteristic(aConnId, &serviceId, &charId,
                                             aAuthReq);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::ReadCharacteristic,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::WriteCharacteristic(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId, int aWriteType, int aLen,
  int aAuthReq, const ArrayBuffer& aValue,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;
  char value[aLen + 1];

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId)) &&
      NS_SUCCEEDED(Convert(aValue, value))) {
    status = mInterface->write_characteristic(aConnId, &serviceId, &charId,
                                              aWriteType, aLen, aAuthReq,
                                              value);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::WriteCharacteristic,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::ReadDescriptor(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId,
  const BluetoothGattId& aDescriptorId,
  int aAuthReq, BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;
  btgatt_gatt_id_t descriptorId;

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId)) &&
      NS_SUCCEEDED(Convert(aDescriptorId, descriptorId))) {
    status = mInterface->read_descriptor(aConnId, &serviceId, &charId,
                                         &descriptorId, aAuthReq);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::ReadDescriptor,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::WriteDescriptor(
  int aConnId, const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId,
  const BluetoothGattId& aDescriptorId,
  int aWriteType, int aLen, int aAuthReq,
  const ArrayBuffer& aValue,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;
  btgatt_gatt_id_t descriptorId;
  char value[aLen + 1];

  if (NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId)) &&
      NS_SUCCEEDED(Convert(aDescriptorId, descriptorId)) &&
      NS_SUCCEEDED(Convert(aValue, value))) {
    status = mInterface->write_descriptor(aConnId, &serviceId, &charId,
                                          &descriptorId, aWriteType, aLen,
                                          aAuthReq, value);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::WriteDescriptor,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::ExecuteWrite(
  int aConnId, int aIsExecute, BluetoothGattClientResultHandler* aRes)
{
#if ANDROID_VERSION >= 19
  int status = mInterface->execute_write(aConnId, aIsExecute);
#else
  int status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::ExecuteWrite,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::RegisterNotification(
  int aClientIf, const nsAString& aBdAddr,
  const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr)) &&
      NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId))) {
    status = mInterface->register_for_notification(aClientIf, &bdAddr,
                                                   &serviceId, &charId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::RegisterNotification,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::DeregisterNotification(
  int aClientIf, const nsAString& aBdAddr,
  const BluetoothGattServiceId& aServiceId,
  const BluetoothGattId& aCharId,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;
  btgatt_srvc_id_t serviceId;
  btgatt_gatt_id_t charId;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr)) &&
      NS_SUCCEEDED(Convert(aServiceId, serviceId)) &&
      NS_SUCCEEDED(Convert(aCharId, charId))) {
    status = mInterface->deregister_for_notification(aClientIf, &bdAddr,
                                                     &serviceId, &charId);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::DeregisterNotification,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::ReadRemoteRssi(
  int aClientIf, const nsAString& aBdAddr,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr))) {
    status = mInterface->read_remote_rssi(aClientIf, &bdAddr);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::ReadRemoteRssi,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::GetDeviceType(
  const nsAString& aBdAddr, BluetoothGattClientResultHandler* aRes)
{
  int status;
#if ANDROID_VERSION >= 19
  bt_bdaddr_t bdAddr;

  if (NS_SUCCEEDED(Convert(aBdAddr, bdAddr))) {
    status = mInterface->get_device_type(&bdAddr);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::GetDeviceType,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattClientHALInterface::SetAdvData(
  int aServerIf, bool aIsScanRsp, bool aIsNameIncluded,
  bool aIsTxPowerIncluded, int aMinInterval, int aMaxInterval, int aApperance,
  uint8_t aManufacturerLen, const ArrayBuffer& aManufacturerData,
  BluetoothGattClientResultHandler* aRes)
{
  bt_status_t status;
#if ANDROID_VERSION >= 19
  char value[aManufacturerLen + 1];

  if (NS_SUCCEEDED(Convert(aManufacturerData, value))) {
    status = mInterface->set_adv_data(
      aServerIf, aIsScanRsp, aIsNameIncluded, aIsTxPowerIncluded, aMinInterval,
      aMaxInterval, aApperance, aManufacturerLen, value);
  } else {
    status = BT_STATUS_PARM_INVALID;
  }
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattClientHALResult(
      aRes, &BluetoothGattClientResultHandler::SetAdvData,
      ConvertDefault(status, STATUS_FAIL));
  }
}

// TODO: Add GATT Server Interface

// GATT Interface

BluetoothGattHALInterface::BluetoothGattHALInterface(
#if ANDROID_VERSION >= 19
  const btgatt_interface_t* aInterface
#endif
  )
#if ANDROID_VERSION >= 19
  :mInterface(aInterface)
#endif
{
#if ANDROID_VERSION >= 19
  MOZ_ASSERT(mInterface);
#endif
}

BluetoothGattHALInterface::~BluetoothGattHALInterface()
{ }

void
BluetoothGattHALInterface::Init(
  BluetoothGattNotificationHandler* aNotificationHandler,
  BluetoothGattResultHandler* aRes)
{
#if ANDROID_VERSION >= 19
  static const btgatt_client_callbacks_t sGattClientCallbacks = {
    BluetoothGattClientCallback::RegisterClient,
    BluetoothGattClientCallback::ScanResult,
    BluetoothGattClientCallback::Connect,
    BluetoothGattClientCallback::Disconnect,
    BluetoothGattClientCallback::SearchComplete,
    BluetoothGattClientCallback::SearchResult,
    BluetoothGattClientCallback::GetCharacteristic,
    BluetoothGattClientCallback::GetDescriptor,
    BluetoothGattClientCallback::GetIncludedService,
    BluetoothGattClientCallback::RegisterNotification,
    BluetoothGattClientCallback::Notify,
    BluetoothGattClientCallback::ReadCharacteristic,
    BluetoothGattClientCallback::WriteCharacteristic,
    BluetoothGattClientCallback::ReadDescriptor,
    BluetoothGattClientCallback::WriteDescriptor,
    BluetoothGattClientCallback::ExecuteWrite,
    BluetoothGattClientCallback::ReadRemoteRssi,
    BluetoothGattClientCallback::Listen
  };

  static const btgatt_server_callbacks_t sGattServerCallbacks = {
    BluetoothGattServerCallback::RegisterServer,
    BluetoothGattServerCallback::Connection,
    BluetoothGattServerCallback::ServiceAdded,
    BluetoothGattServerCallback::IncludedServiceAdded,
    BluetoothGattServerCallback::CharacteristicAdded,
    BluetoothGattServerCallback::DescriptorAdded,
    BluetoothGattServerCallback::ServiceStarted,
    BluetoothGattServerCallback::ServiceStopped,
    BluetoothGattServerCallback::ServiceDeleted,
    BluetoothGattServerCallback::RequestRead,
    BluetoothGattServerCallback::RequestWrite,
    BluetoothGattServerCallback::RequestExecWrite,
    BluetoothGattServerCallback::ResponseConfirmation
  };

  static const btgatt_callbacks_t sCallbacks = {
    sizeof(sCallbacks),
    &sGattClientCallbacks,
    &sGattServerCallbacks
  };
#endif // ANDROID_VERSION >= 19

  sGattNotificationHandler = aNotificationHandler;

#if ANDROID_VERSION >= 19
  bt_status_t status = mInterface->init(&sCallbacks);
#else
  bt_status_t status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattHALResult(
      aRes, &BluetoothGattResultHandler::Init,
      ConvertDefault(status, STATUS_FAIL));
  }
}

void
BluetoothGattHALInterface::Cleanup(BluetoothGattResultHandler* aRes)
{
  bt_status_t status;

#if ANDROID_VERSION >= 19
  mInterface->cleanup();
  status = BT_STATUS_SUCCESS;
#else
  status = BT_STATUS_UNSUPPORTED;
#endif

  if (aRes) {
    DispatchBluetoothGattHALResult(
      aRes, &BluetoothGattResultHandler::Cleanup,
      ConvertDefault(status, STATUS_FAIL));
  }
}

BluetoothGattClientInterface*
BluetoothGattHALInterface::GetBluetoothGattClientInterface()
{
  static BluetoothGattClientHALInterface* sBluetoothGattClientHALInterface;

  if (sBluetoothGattClientHALInterface) {
    return sBluetoothGattClientHALInterface;
  }

#if ANDROID_VERSION >= 19
  MOZ_ASSERT(mInterface->client);
  sBluetoothGattClientHALInterface =
    new BluetoothGattClientHALInterface(mInterface->client);
#else
  sBluetoothGattClientHALInterface =
    new BluetoothGattClientHALInterface();
#endif

  return sBluetoothGattClientHALInterface;
}

END_BLUETOOTH_NAMESPACE
