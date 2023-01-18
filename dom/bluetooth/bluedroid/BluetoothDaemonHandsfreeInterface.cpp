/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "BluetoothDaemonHandsfreeInterface.h"
#include "BluetoothDaemonSetupInterface.h"
#include "mozilla/unused.h"

BEGIN_BLUETOOTH_NAMESPACE

//
// Handsfree module
//

BluetoothHandsfreeNotificationHandler*
  BluetoothDaemonHandsfreeModule::sNotificationHandler;

#if ANDROID_VERSION < 21
nsString BluetoothDaemonHandsfreeModule::sConnectedDeviceAddress(
  NS_ConvertUTF8toUTF16(BLUETOOTH_ADDRESS_NONE));
#endif

void
BluetoothDaemonHandsfreeModule::SetNotificationHandler(
  BluetoothHandsfreeNotificationHandler* aNotificationHandler)
{
  sNotificationHandler = aNotificationHandler;
}

nsresult
BluetoothDaemonHandsfreeModule::Send(BluetoothDaemonPDU* aPDU,
                                     BluetoothHandsfreeResultHandler* aRes)
{
  aRes->AddRef(); // Keep reference for response
  return Send(aPDU, static_cast<void*>(aRes));
}

void
BluetoothDaemonHandsfreeModule::HandleSvc(const BluetoothDaemonPDUHeader& aHeader,
                                          BluetoothDaemonPDU& aPDU, void* aUserData)
{
  static void (BluetoothDaemonHandsfreeModule::* const HandleOp[])(
    const BluetoothDaemonPDUHeader&, BluetoothDaemonPDU&, void*) = {
    INIT_ARRAY_AT(0, &BluetoothDaemonHandsfreeModule::HandleRsp),
    INIT_ARRAY_AT(1, &BluetoothDaemonHandsfreeModule::HandleNtf),
  };

  MOZ_ASSERT(!NS_IsMainThread());

  // Negate twice to map bit to 0/1
  unsigned long isNtf = !!(aHeader.mOpcode & 0x80);

  (this->*(HandleOp[isNtf]))(aHeader, aPDU, aUserData);
}

// Commands
//

nsresult
BluetoothDaemonHandsfreeModule::ConnectCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_CONNECT,
                           6)); // Address

  nsresult rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::DisconnectCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_DISCONNECT,
                           6)); // Address

  nsresult rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::ConnectAudioCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_CONNECT_AUDIO,
                           6)); // Address

  nsresult rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::DisconnectAudioCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_DISCONNECT_AUDIO,
                           6)); // Address

  nsresult rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::StartVoiceRecognitionCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_START_VOICE_RECOGNITION,
                           6)); // Address (BlueZ 5.25)

  nsresult rv;
#if ANDROID_VERSION >= 21
  rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
#endif
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::StopVoiceRecognitionCmd(
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_STOP_VOICE_RECOGNITION,
                           6)); // Address (BlueZ 5.25)

  nsresult rv;
#if ANDROID_VERSION >= 21
  rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
#endif
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::VolumeControlCmd(
  BluetoothHandsfreeVolumeType aType, int aVolume,
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_VOLUME_CONTROL,
                           1 + // Volume type
                           1 + // Volume
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    aType, PackConversion<int, uint8_t>(aVolume),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(aType, PackConversion<int, uint8_t>(aVolume), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::DeviceStatusNotificationCmd(
  BluetoothHandsfreeNetworkState aNtkState,
  BluetoothHandsfreeServiceType aSvcType, int aSignal, int aBattChg,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_DEVICE_STATUS_NOTIFICATION,
                           1 + // Network state
                           1 + // Service type
                           1 + // Signal strength
                           1)); // Battery level

  nsresult rv = PackPDU(aNtkState, aSvcType,
                        PackConversion<int, uint8_t>(aSignal),
                        PackConversion<int, uint8_t>(aBattChg), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::CopsResponseCmd(
  const char* aCops, const nsAString& aRemoteAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_COPS_RESPONSE,
                           0 + // Dynamically allocated
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    PackCString0(nsDependentCString(aCops)),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(PackCString0(nsDependentCString(aCops)), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::CindResponseCmd(
  int aSvc, int aNumActive, int aNumHeld,
  BluetoothHandsfreeCallState aCallSetupState,
  int aSignal, int aRoam, int aBattChg,
  const nsAString& aRemoteAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_CIND_RESPONSE,
                           1 + // Service
                           1 + // # Active
                           1 + // # Held
                           1 + // Call state
                           1 + // Signal strength
                           1 + // Roaming
                           1 + // Battery level
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    PackConversion<int, uint8_t>(aSvc),
    PackConversion<int, uint8_t>(aNumActive),
    PackConversion<int, uint8_t>(aNumHeld),
    aCallSetupState,
    PackConversion<int, uint8_t>(aSignal),
    PackConversion<int, uint8_t>(aRoam),
    PackConversion<int, uint8_t>(aBattChg),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(PackConversion<int, uint8_t>(aSvc),
                        PackConversion<int, uint8_t>(aNumActive),
                        PackConversion<int, uint8_t>(aNumHeld),
                        aCallSetupState,
                        PackConversion<int, uint8_t>(aSignal),
                        PackConversion<int, uint8_t>(aRoam),
                        PackConversion<int, uint8_t>(aBattChg), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::FormattedAtResponseCmd(
  const char* aRsp, const nsAString& aRemoteAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_FORMATTED_AT_RESPONSE,
                           0 + // Dynamically allocated
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    PackCString0(nsDependentCString(aRsp)),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(PackCString0(nsDependentCString(aRsp)), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::AtResponseCmd(
  BluetoothHandsfreeAtResponse aResponseCode, int aErrorCode,
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_AT_RESPONSE,
                           1 + // AT Response code
                           1 + // Error code
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    aResponseCode, PackConversion<int, uint8_t>(aErrorCode),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(aResponseCode,
                        PackConversion<int, uint8_t>(aErrorCode), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::ClccResponseCmd(
  int aIndex,
  BluetoothHandsfreeCallDirection aDir, BluetoothHandsfreeCallState aState,
  BluetoothHandsfreeCallMode aMode, BluetoothHandsfreeCallMptyType aMpty,
  const nsAString& aNumber, BluetoothHandsfreeCallAddressType aType,
  const nsAString& aRemoteAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  NS_ConvertUTF16toUTF8 number(aNumber);

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_CLCC_RESPONSE,
                           1 + // Call index
                           1 + // Call direction
                           1 + // Call state
                           1 + // Call mode
                           1 + // Call MPTY
                           1 + // Address type
                           number.Length() + 1 + // Number string + \0
                           6)); // Address (BlueZ 5.25)

#if ANDROID_VERSION >= 21
  nsresult rv = PackPDU(
    PackConversion<int, uint8_t>(aIndex),
    aDir, aState, aMode, aMpty, aType,
    PackCString0(number),
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), *pdu);
#else
  nsresult rv = PackPDU(PackConversion<int, uint8_t>(aIndex),
                        aDir, aState, aMode, aMpty, aType,
                        PackCString0(number), *pdu);
#endif
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::PhoneStateChangeCmd(
  int aNumActive, int aNumHeld, BluetoothHandsfreeCallState aCallSetupState,
  const nsAString& aNumber, BluetoothHandsfreeCallAddressType aType,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  NS_ConvertUTF16toUTF8 number(aNumber);

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_PHONE_STATE_CHANGE,
                           1 + // # Active
                           1 + // # Held
                           1 + // Call state
                           1 + // Address type
                           number.Length() + 1)); // Number string + \0

  nsresult rv = PackPDU(PackConversion<int, uint8_t>(aNumActive),
                        PackConversion<int, uint8_t>(aNumHeld),
                        aCallSetupState, aType,
                        PackCString0(NS_ConvertUTF16toUTF8(aNumber)), *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

nsresult
BluetoothDaemonHandsfreeModule::ConfigureWbsCmd(
  const nsAString& aRemoteAddr,
  BluetoothHandsfreeWbsConfig aConfig,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(NS_IsMainThread());

  nsAutoPtr<BluetoothDaemonPDU> pdu(
    new BluetoothDaemonPDU(SERVICE_ID, OPCODE_CONFIGURE_WBS,
                           6 + // Address
                           1)); // Config

  nsresult rv = PackPDU(
    PackConversion<nsAString, BluetoothAddress>(aRemoteAddr), aConfig, *pdu);
  if (NS_FAILED(rv)) {
    return rv;
  }
  rv = Send(pdu, aRes);
  if (NS_FAILED(rv)) {
    return rv;
  }
  unused << pdu.forget();
  return NS_OK;
}

// Responses
//

void
BluetoothDaemonHandsfreeModule::ErrorRsp(
  const BluetoothDaemonPDUHeader& aHeader,
  BluetoothDaemonPDU& aPDU, BluetoothHandsfreeResultHandler* aRes)
{
  ErrorRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::OnError, UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::ConnectRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::Connect, UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::DisconnectRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::Disconnect, UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::ConnectAudioRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::ConnectAudio,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::DisconnectAudioRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::DisconnectAudio,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::StartVoiceRecognitionRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::StartVoiceRecognition,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::StopVoiceRecognitionRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::StopVoiceRecognition,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::VolumeControlRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::VolumeControl,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::DeviceStatusNotificationRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::DeviceStatusNotification,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::CopsResponseRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::CopsResponse,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::CindResponseRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::CindResponse,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::FormattedAtResponseRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::FormattedAtResponse,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::AtResponseRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::AtResponse,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::ClccResponseRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::ClccResponse,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::PhoneStateChangeRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::PhoneStateChange,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::ConfigureWbsRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  BluetoothHandsfreeResultHandler* aRes)
{
  ResultRunnable::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::ConfigureWbs,
    UnpackPDUInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::HandleRsp(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  void* aUserData)
{
  static void (BluetoothDaemonHandsfreeModule::* const HandleRsp[])(
    const BluetoothDaemonPDUHeader&,
    BluetoothDaemonPDU&,
    BluetoothHandsfreeResultHandler*) = {
    INIT_ARRAY_AT(OPCODE_ERROR,
      &BluetoothDaemonHandsfreeModule::ErrorRsp),
    INIT_ARRAY_AT(OPCODE_CONNECT,
      &BluetoothDaemonHandsfreeModule::ConnectRsp),
    INIT_ARRAY_AT(OPCODE_DISCONNECT,
      &BluetoothDaemonHandsfreeModule::DisconnectRsp),
    INIT_ARRAY_AT(OPCODE_CONNECT_AUDIO,
      &BluetoothDaemonHandsfreeModule::ConnectAudioRsp),
    INIT_ARRAY_AT(OPCODE_DISCONNECT_AUDIO,
      &BluetoothDaemonHandsfreeModule::DisconnectAudioRsp),
    INIT_ARRAY_AT(OPCODE_START_VOICE_RECOGNITION,
      &BluetoothDaemonHandsfreeModule::StartVoiceRecognitionRsp),
    INIT_ARRAY_AT(OPCODE_STOP_VOICE_RECOGNITION,
      &BluetoothDaemonHandsfreeModule::StopVoiceRecognitionRsp),
    INIT_ARRAY_AT(OPCODE_VOLUME_CONTROL,
      &BluetoothDaemonHandsfreeModule::VolumeControlRsp),
    INIT_ARRAY_AT(OPCODE_DEVICE_STATUS_NOTIFICATION,
      &BluetoothDaemonHandsfreeModule::DeviceStatusNotificationRsp),
    INIT_ARRAY_AT(OPCODE_COPS_RESPONSE,
      &BluetoothDaemonHandsfreeModule::CopsResponseRsp),
    INIT_ARRAY_AT(OPCODE_CIND_RESPONSE,
      &BluetoothDaemonHandsfreeModule::CindResponseRsp),
    INIT_ARRAY_AT(OPCODE_FORMATTED_AT_RESPONSE,
      &BluetoothDaemonHandsfreeModule::FormattedAtResponseRsp),
    INIT_ARRAY_AT(OPCODE_AT_RESPONSE,
      &BluetoothDaemonHandsfreeModule::AtResponseRsp),
    INIT_ARRAY_AT(OPCODE_CLCC_RESPONSE,
      &BluetoothDaemonHandsfreeModule::ClccResponseRsp),
    INIT_ARRAY_AT(OPCODE_PHONE_STATE_CHANGE,
      &BluetoothDaemonHandsfreeModule::PhoneStateChangeRsp),
    INIT_ARRAY_AT(OPCODE_CONFIGURE_WBS,
      &BluetoothDaemonHandsfreeModule::ConfigureWbsRsp)
  };

  MOZ_ASSERT(!NS_IsMainThread()); // I/O thread

  if (NS_WARN_IF(!(aHeader.mOpcode < MOZ_ARRAY_LENGTH(HandleRsp))) ||
      NS_WARN_IF(!HandleRsp[aHeader.mOpcode])) {
    return;
  }

  nsRefPtr<BluetoothHandsfreeResultHandler> res =
    already_AddRefed<BluetoothHandsfreeResultHandler>(
      static_cast<BluetoothHandsfreeResultHandler*>(aUserData));

  if (!res) {
    return; // Return early if no result handler has been set for response
  }

  (this->*(HandleRsp[aHeader.mOpcode]))(aHeader, aPDU, res);
}

// Notifications
//

// Returns the current notification handler to a notification runnable
class BluetoothDaemonHandsfreeModule::NotificationHandlerWrapper final
{
public:
  typedef BluetoothHandsfreeNotificationHandler ObjectType;

  static ObjectType* GetInstance()
  {
    MOZ_ASSERT(NS_IsMainThread());

    return sNotificationHandler;
  }
};

// Init operator class for ConnectionStateNotification
class BluetoothDaemonHandsfreeModule::ConnectionStateInitOp final
  : private PDUInitOp
{
public:
  ConnectionStateInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeConnectionState& aArg1,
               nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read state */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }

#if ANDROID_VERSION < 21
    if (aArg1 == HFP_CONNECTION_STATE_CONNECTED) {
      sConnectedDeviceAddress = aArg2;
    } else if (aArg1 == HFP_CONNECTION_STATE_DISCONNECTED) {
      sConnectedDeviceAddress.AssignLiteral(BLUETOOTH_ADDRESS_NONE);
    }
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::ConnectionStateNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  ConnectionStateNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::ConnectionStateNotification,
    ConnectionStateInitOp(aPDU));
}

// Init operator class for AudioStateNotification
class BluetoothDaemonHandsfreeModule::AudioStateInitOp final
  : private PDUInitOp
{
public:
  AudioStateInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeAudioState& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read state */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::AudioStateNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  AudioStateNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::AudioStateNotification,
    AudioStateInitOp(aPDU));
}

// Init operator class for VoiceRecognitionNotification
class BluetoothDaemonHandsfreeModule::VoiceRecognitionInitOp final
  : private PDUInitOp
{
public:
  VoiceRecognitionInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeVoiceRecognitionState& aArg1,
               nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read state */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::VoiceRecognitionNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  VoiceRecognitionNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::VoiceRecognitionNotification,
    VoiceRecognitionInitOp(aPDU));
}

// Init operator class for AnswerCallNotification
class BluetoothDaemonHandsfreeModule::AnswerCallInitOp final
  : private PDUInitOp
{
public:
  AnswerCallInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::AnswerCallNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  AnswerCallNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::AnswerCallNotification,
    AnswerCallInitOp(aPDU));
}

// Init operator class for HangupCallNotification
class BluetoothDaemonHandsfreeModule::HangupCallInitOp final
  : private PDUInitOp
{
public:
  HangupCallInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::HangupCallNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  HangupCallNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::HangupCallNotification,
    HangupCallInitOp(aPDU));
}

// Init operator class for VolumeNotification
class BluetoothDaemonHandsfreeModule::VolumeInitOp final
  : private PDUInitOp
{
public:
  VolumeInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeVolumeType& aArg1, int& aArg2,
               nsString& aArg3) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read volume type */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read volume */
    rv = UnpackPDU(pdu, UnpackConversion<uint8_t, int>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg3));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg3 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::VolumeNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  VolumeNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::VolumeNotification,
    VolumeInitOp(aPDU));
}

// Init operator class for DialCallNotification
class BluetoothDaemonHandsfreeModule::DialCallInitOp final
  : private PDUInitOp
{
public:
  DialCallInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    nsresult rv;
    /* Read address
     * It's a little weird to parse aArg2(aBdAddr) before parsing
     * aArg1(aNumber), but this order is defined in BlueZ 5.25 anyway.
     */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif

    /* Read number */
    rv = UnpackPDU(pdu, UnpackString0(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::DialCallNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  DialCallNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::DialCallNotification,
    DialCallInitOp(aPDU));
}

// Init operator class for DtmfNotification
class BluetoothDaemonHandsfreeModule::DtmfInitOp final
  : private PDUInitOp
{
public:
  DtmfInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (char& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read tone */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::DtmfNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  DtmfNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::DtmfNotification,
    DtmfInitOp(aPDU));
}

// Init operator class for NRECNotification
class BluetoothDaemonHandsfreeModule::NRECInitOp final
  : private PDUInitOp
{
public:
  NRECInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeNRECState& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read state */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::NRECNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  NRECNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::NRECNotification,
    NRECInitOp(aPDU));
}

// Init operator class for CallHoldNotification
class BluetoothDaemonHandsfreeModule::CallHoldInitOp final
  : private PDUInitOp
{
public:
  CallHoldInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (BluetoothHandsfreeCallHoldType& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    /* Read type */
    nsresult rv = UnpackPDU(pdu, aArg1);
    if (NS_FAILED(rv)) {
      return rv;
    }

    /* Read address */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::CallHoldNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  CallHoldNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::CallHoldNotification,
    CallHoldInitOp(aPDU));
}

// Init operator class for CnumNotification
class BluetoothDaemonHandsfreeModule::CnumInitOp final
  : private PDUInitOp
{
public:
  CnumInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::CnumNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  CnumNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::CnumNotification,
    CnumInitOp(aPDU));
}

// Init operator class for CindNotification
class BluetoothDaemonHandsfreeModule::CindInitOp final
  : private PDUInitOp
{
public:
  CindInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::CindNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  CindNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::CindNotification,
    CindInitOp(aPDU));
}

// Init operator class for CopsNotification
class BluetoothDaemonHandsfreeModule::CopsInitOp final
  : private PDUInitOp
{
public:
  CopsInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::CopsNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  CopsNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::CopsNotification,
    CopsInitOp(aPDU));
}

// Init operator class for ClccNotification
class BluetoothDaemonHandsfreeModule::ClccInitOp final
  : private PDUInitOp
{
public:
  ClccInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::ClccNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  ClccNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::ClccNotification,
    ClccInitOp(aPDU));
}

// Init operator class for UnknownAtNotification
class BluetoothDaemonHandsfreeModule::UnknownAtInitOp final
  : private PDUInitOp
{
public:
  UnknownAtInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsCString& aArg1, nsString& aArg2) const
  {
    BluetoothDaemonPDU& pdu = GetPDU();

    nsresult rv;
    /* Read address
     * It's a little weird to parse aArg2(aBdAddr) before parsing
     * aArg1(aAtString), but this order is defined in BlueZ 5.25 anyway.
     */
#if ANDROID_VERSION >= 21
    rv = UnpackPDU(
      pdu, UnpackConversion<BluetoothAddress, nsAString>(aArg2));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg2 = sConnectedDeviceAddress;
#endif

    /* Read string */
    rv = UnpackPDU(pdu, UnpackCString0(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::UnknownAtNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  UnknownAtNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::UnknownAtNotification,
    UnknownAtInitOp(aPDU));
}

// Init operator class for KeyPressedNotification
class BluetoothDaemonHandsfreeModule::KeyPressedInitOp final
  : private PDUInitOp
{
public:
  KeyPressedInitOp(BluetoothDaemonPDU& aPDU)
    : PDUInitOp(aPDU)
  { }

  nsresult
  operator () (nsString& aArg1) const
  {
    /* Read address */
#if ANDROID_VERSION >= 21
    nsresult rv = UnpackPDU(
      GetPDU(), UnpackConversion<BluetoothAddress, nsAString>(aArg1));
    if (NS_FAILED(rv)) {
      return rv;
    }
#else
    aArg1 = sConnectedDeviceAddress;
#endif
    WarnAboutTrailingData();
    return NS_OK;
  }
};

void
BluetoothDaemonHandsfreeModule::KeyPressedNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU)
{
  KeyPressedNotification::Dispatch(
    &BluetoothHandsfreeNotificationHandler::KeyPressedNotification,
    KeyPressedInitOp(aPDU));
}

void
BluetoothDaemonHandsfreeModule::HandleNtf(
  const BluetoothDaemonPDUHeader& aHeader, BluetoothDaemonPDU& aPDU,
  void* aUserData)
{
  static void (BluetoothDaemonHandsfreeModule::* const HandleNtf[])(
    const BluetoothDaemonPDUHeader&, BluetoothDaemonPDU&) = {
    INIT_ARRAY_AT(0, &BluetoothDaemonHandsfreeModule::ConnectionStateNtf),
    INIT_ARRAY_AT(1, &BluetoothDaemonHandsfreeModule::AudioStateNtf),
    INIT_ARRAY_AT(2, &BluetoothDaemonHandsfreeModule::VoiceRecognitionNtf),
    INIT_ARRAY_AT(3, &BluetoothDaemonHandsfreeModule::AnswerCallNtf),
    INIT_ARRAY_AT(4, &BluetoothDaemonHandsfreeModule::HangupCallNtf),
    INIT_ARRAY_AT(5, &BluetoothDaemonHandsfreeModule::VolumeNtf),
    INIT_ARRAY_AT(6, &BluetoothDaemonHandsfreeModule::DialCallNtf),
    INIT_ARRAY_AT(7, &BluetoothDaemonHandsfreeModule::DtmfNtf),
    INIT_ARRAY_AT(8, &BluetoothDaemonHandsfreeModule::NRECNtf),
    INIT_ARRAY_AT(9, &BluetoothDaemonHandsfreeModule::CallHoldNtf),
    INIT_ARRAY_AT(10, &BluetoothDaemonHandsfreeModule::CnumNtf),
    INIT_ARRAY_AT(11, &BluetoothDaemonHandsfreeModule::CindNtf),
    INIT_ARRAY_AT(12, &BluetoothDaemonHandsfreeModule::CopsNtf),
    INIT_ARRAY_AT(13, &BluetoothDaemonHandsfreeModule::ClccNtf),
    INIT_ARRAY_AT(14, &BluetoothDaemonHandsfreeModule::UnknownAtNtf),
    INIT_ARRAY_AT(15, &BluetoothDaemonHandsfreeModule::KeyPressedNtf)
  };

  MOZ_ASSERT(!NS_IsMainThread());

  uint8_t index = aHeader.mOpcode - 0x81;

  if (NS_WARN_IF(!(index < MOZ_ARRAY_LENGTH(HandleNtf))) ||
      NS_WARN_IF(!HandleNtf[index])) {
    return;
  }

  (this->*(HandleNtf[index]))(aHeader, aPDU);
}

//
// Handsfree interface
//

BluetoothDaemonHandsfreeInterface::BluetoothDaemonHandsfreeInterface(
  BluetoothDaemonHandsfreeModule* aModule)
  : mModule(aModule)
{ }

BluetoothDaemonHandsfreeInterface::~BluetoothDaemonHandsfreeInterface()
{ }

class BluetoothDaemonHandsfreeInterface::InitResultHandler final
  : public BluetoothSetupResultHandler
{
public:
  InitResultHandler(BluetoothHandsfreeResultHandler* aRes)
    : mRes(aRes)
  {
    MOZ_ASSERT(mRes);
  }

  void OnError(BluetoothStatus aStatus) override
  {
    MOZ_ASSERT(NS_IsMainThread());

    mRes->OnError(aStatus);
  }

  void RegisterModule() override
  {
    MOZ_ASSERT(NS_IsMainThread());

    mRes->Init();
  }

private:
  nsRefPtr<BluetoothHandsfreeResultHandler> mRes;
};

void
BluetoothDaemonHandsfreeInterface::Init(
  BluetoothHandsfreeNotificationHandler* aNotificationHandler,
  int aMaxNumClients, BluetoothHandsfreeResultHandler* aRes)
{
  // Set notification handler _before_ registering the module. It could
  // happen that we receive notifications, before the result handler runs.
  mModule->SetNotificationHandler(aNotificationHandler);

  InitResultHandler* res;

  if (aRes) {
    res = new InitResultHandler(aRes);
  } else {
    // We don't need a result handler if the caller is not interested.
    res = nullptr;
  }

  nsresult rv = mModule->RegisterModule(
    BluetoothDaemonHandsfreeModule::SERVICE_ID, MODE_NARROWBAND_SPEECH,
    aMaxNumClients, res);

  if (NS_FAILED(rv) && aRes) {
    DispatchError(aRes, STATUS_FAIL);
  }
}

class BluetoothDaemonHandsfreeInterface::CleanupResultHandler final
  : public BluetoothSetupResultHandler
{
public:
  CleanupResultHandler(BluetoothDaemonHandsfreeModule* aModule,
                       BluetoothHandsfreeResultHandler* aRes)
    : mModule(aModule)
    , mRes(aRes)
  {
    MOZ_ASSERT(mModule);
  }

  void OnError(BluetoothStatus aStatus) override
  {
    MOZ_ASSERT(NS_IsMainThread());

    if (mRes) {
      mRes->OnError(aStatus);
    }
  }

  void UnregisterModule() override
  {
    MOZ_ASSERT(NS_IsMainThread());

    // Clear notification handler _after_ module has been
    // unregistered. While unregistering the module, we might
    // still receive notifications.
    mModule->SetNotificationHandler(nullptr);

    if (mRes) {
      mRes->Cleanup();
    }
  }

private:
  BluetoothDaemonHandsfreeModule* mModule;
  nsRefPtr<BluetoothHandsfreeResultHandler> mRes;
};

void
BluetoothDaemonHandsfreeInterface::Cleanup(
  BluetoothHandsfreeResultHandler* aRes)
{
  mModule->UnregisterModule(BluetoothDaemonHandsfreeModule::SERVICE_ID,
                            new CleanupResultHandler(mModule, aRes));
}

/* Connect / Disconnect */

void
BluetoothDaemonHandsfreeInterface::Connect(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->ConnectCmd(aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::Disconnect(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->DisconnectCmd(aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::ConnectAudio(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->ConnectAudioCmd(aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::DisconnectAudio(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->DisconnectAudioCmd(aBdAddr, aRes);
}

/* Voice Recognition */

void
BluetoothDaemonHandsfreeInterface::StartVoiceRecognition(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->StartVoiceRecognitionCmd(aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::StopVoiceRecognition(
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->StopVoiceRecognitionCmd(aBdAddr, aRes);
}

/* Volume */

void
BluetoothDaemonHandsfreeInterface::VolumeControl(
  BluetoothHandsfreeVolumeType aType, int aVolume, const nsAString& aBdAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->VolumeControlCmd(aType, aVolume, aBdAddr, aRes);
}

/* Device status */

void
BluetoothDaemonHandsfreeInterface::DeviceStatusNotification(
  BluetoothHandsfreeNetworkState aNtkState,
  BluetoothHandsfreeServiceType aSvcType, int aSignal, int aBattChg,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->DeviceStatusNotificationCmd(aNtkState, aSvcType, aSignal,
                                       aBattChg, aRes);
}

/* Responses */

void
BluetoothDaemonHandsfreeInterface::CopsResponse(
  const char* aCops, const nsAString& aBdAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->CopsResponseCmd(aCops, aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::CindResponse(
  int aSvc, int aNumActive, int aNumHeld,
  BluetoothHandsfreeCallState aCallSetupState,
  int aSignal, int aRoam, int aBattChg,
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->CindResponseCmd(aSvc, aNumActive, aNumHeld, aCallSetupState,
                           aSignal, aRoam, aBattChg, aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::FormattedAtResponse(
  const char* aRsp, const nsAString& aBdAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->FormattedAtResponseCmd(aRsp, aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::AtResponse(
  BluetoothHandsfreeAtResponse aResponseCode, int aErrorCode,
  const nsAString& aBdAddr, BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->AtResponseCmd(aResponseCode, aErrorCode, aBdAddr, aRes);
}

void
BluetoothDaemonHandsfreeInterface::ClccResponse(
  int aIndex, BluetoothHandsfreeCallDirection aDir,
  BluetoothHandsfreeCallState aState,
  BluetoothHandsfreeCallMode aMode,
  BluetoothHandsfreeCallMptyType aMpty,
  const nsAString& aNumber,
  BluetoothHandsfreeCallAddressType aType,
  const nsAString& aBdAddr,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->ClccResponseCmd(aIndex, aDir, aState, aMode, aMpty, aNumber,
                           aType, aBdAddr, aRes);
}

/* Phone State */

void
BluetoothDaemonHandsfreeInterface::PhoneStateChange(
  int aNumActive, int aNumHeld,
  BluetoothHandsfreeCallState aCallSetupState,
  const nsAString& aNumber,
  BluetoothHandsfreeCallAddressType aType,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->PhoneStateChangeCmd(aNumActive, aNumHeld, aCallSetupState, aNumber,
                               aType, aRes);
}

/* Wide Band Speech */

void
BluetoothDaemonHandsfreeInterface::ConfigureWbs(
  const nsAString& aBdAddr, BluetoothHandsfreeWbsConfig aConfig,
  BluetoothHandsfreeResultHandler* aRes)
{
  MOZ_ASSERT(mModule);

  mModule->ConfigureWbsCmd(aBdAddr, aConfig, aRes);
}

void
BluetoothDaemonHandsfreeInterface::DispatchError(
  BluetoothHandsfreeResultHandler* aRes, BluetoothStatus aStatus)
{
  BluetoothResultRunnable1<BluetoothHandsfreeResultHandler, void,
                           BluetoothStatus, BluetoothStatus>::Dispatch(
    aRes, &BluetoothHandsfreeResultHandler::OnError,
    ConstantInitOp1<BluetoothStatus>(aStatus));
}

END_BLUETOOTH_NAMESPACE
