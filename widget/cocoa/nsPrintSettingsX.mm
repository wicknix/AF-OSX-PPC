/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "nsPrintSettingsX.h"
#include "nsObjCExceptions.h"

#include "plbase64.h"
#include "plstr.h"

#include "nsCocoaUtils.h"

#include "mozilla/Preferences.h"

using namespace mozilla;

#define MAC_OS_X_PAGE_SETUP_PREFNAME    "print.macosx.pagesetup-2"

NS_IMPL_ISUPPORTS_INHERITED(nsPrintSettingsX, nsPrintSettings, nsPrintSettingsX)

nsPrintSettingsX::nsPrintSettingsX()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK;

  mPrintInfo = [[NSPrintInfo sharedPrintInfo] copy];

  NS_OBJC_END_TRY_ABORT_BLOCK;
}

nsPrintSettingsX::nsPrintSettingsX(const nsPrintSettingsX& src)
{
  *this = src;
}

nsPrintSettingsX::~nsPrintSettingsX()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK;

  [mPrintInfo release];

  NS_OBJC_END_TRY_ABORT_BLOCK;
}

nsPrintSettingsX& nsPrintSettingsX::operator=(const nsPrintSettingsX& rhs)
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK_RETURN;

  if (this == &rhs) {
    return *this;
  }
  
  nsPrintSettings::operator=(rhs);

  [mPrintInfo release];
  mPrintInfo = [rhs.mPrintInfo copy];

  return *this;

  NS_OBJC_END_TRY_ABORT_BLOCK_RETURN(*this);
}

nsresult nsPrintSettingsX::Init()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK_NSRESULT;

  InitUnwriteableMargin();

  return NS_OK;

  NS_OBJC_END_TRY_ABORT_BLOCK_NSRESULT;
}

// Should be called whenever the page format changes.
NS_IMETHODIMP nsPrintSettingsX::InitUnwriteableMargin()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK_NSRESULT;

  PMPaper paper;
  PMPaperMargins paperMargin;
  PMPageFormat pageFormat = GetPMPageFormat();
  ::PMGetPageFormatPaper(pageFormat, &paper);
  ::PMPaperGetMargins(paper, &paperMargin);
  mUnwriteableMargin.top    = NS_POINTS_TO_INT_TWIPS(paperMargin.top);
  mUnwriteableMargin.left   = NS_POINTS_TO_INT_TWIPS(paperMargin.left);
  mUnwriteableMargin.bottom = NS_POINTS_TO_INT_TWIPS(paperMargin.bottom);
  mUnwriteableMargin.right  = NS_POINTS_TO_INT_TWIPS(paperMargin.right);

  return NS_OK;

  NS_OBJC_END_TRY_ABORT_BLOCK_NSRESULT;  
}

void
nsPrintSettingsX::SetCocoaPrintInfo(NSPrintInfo* aPrintInfo)
{
  mPrintInfo = aPrintInfo;
}

NS_IMETHODIMP nsPrintSettingsX::ReadPageFormatFromPrefs()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK_NSRESULT;

  nsAutoCString encodedData;
  nsresult rv =
    Preferences::GetCString(MAC_OS_X_PAGE_SETUP_PREFNAME, &encodedData);
  if (NS_FAILED(rv)) {
    return rv;
  }

  // decode the base64
  char* decodedData = PL_Base64Decode(encodedData.get(), encodedData.Length(), nullptr);
  NSData* data = [NSData dataWithBytes:decodedData length:strlen(decodedData)];
  if (!data)
    return NS_ERROR_FAILURE;

  PMPageFormat newPageFormat;
#ifdef NS_LEOPARD_AND_LATER
  OSStatus status = ::PMPageFormatCreateWithDataRepresentation((CFDataRef)data, &newPageFormat);
#else
  OSStatus status = ::PMUnflattenPageFormatWithCFData((CFDataRef)data, &newPageFormat);
#endif
  if (status == noErr) {
    SetPMPageFormat(newPageFormat);
  }
  InitUnwriteableMargin();

  return NS_OK;

  NS_OBJC_END_TRY_ABORT_BLOCK_NSRESULT;
}

NS_IMETHODIMP nsPrintSettingsX::WritePageFormatToPrefs()
{
  NS_OBJC_BEGIN_TRY_ABORT_BLOCK_NSRESULT;

  PMPageFormat pageFormat = GetPMPageFormat();
  if (pageFormat == kPMNoPageFormat)
    return NS_ERROR_NOT_INITIALIZED;

  NSData* data = nil;
#ifdef NS_LEOPARD_AND_LATER
  OSStatus err = ::PMPageFormatCreateDataRepresentation(pageFormat, (CFDataRef*)&data, kPMDataFormatXMLDefault);
#else
  OSStatus err = ::PMFlattenPageFormatToCFData(pageFormat, (CFDataRef*)&data);
#endif
  if (err != noErr)
    return NS_ERROR_FAILURE;

  nsAutoCString encodedData;
  encodedData.Adopt(PL_Base64Encode((char*)[data bytes], [data length], nullptr));
  if (!encodedData.get())
    return NS_ERROR_OUT_OF_MEMORY;

  return Preferences::SetCString(MAC_OS_X_PAGE_SETUP_PREFNAME, encodedData);

  NS_OBJC_END_TRY_ABORT_BLOCK_NSRESULT;
}

nsresult nsPrintSettingsX::_Clone(nsIPrintSettings **_retval)
{
  NS_ENSURE_ARG_POINTER(_retval);
  *_retval = nullptr;
  
  nsPrintSettingsX *newSettings = new nsPrintSettingsX(*this);
  if (!newSettings)
    return NS_ERROR_FAILURE;
  *_retval = newSettings;
  NS_ADDREF(*_retval);
  return NS_OK;
}

NS_IMETHODIMP nsPrintSettingsX::_Assign(nsIPrintSettings *aPS)
{
  nsPrintSettingsX *printSettingsX = static_cast<nsPrintSettingsX*>(aPS);
  if (!printSettingsX)
    return NS_ERROR_UNEXPECTED;
  *this = *printSettingsX;
  return NS_OK;
}

// The methods below provide wrappers for the different ways of accessing the
// Core Printing PM* objects from the NSPrintInfo object. On 10.4 we need to
// use secret methods which have been made public in 10.5 with slightly
// different names.

// Secret 10.4 methods (from Appkit class dump):
@interface NSPrintInfo (NSTemporaryCompatibility)
- (struct OpaquePMPrintSession *)_pmPrintSession;
- (void)setPMPageFormat:(struct OpaquePMPageFormat *)arg1;
- (struct OpaquePMPageFormat *)pmPageFormat;
- (void)setPMPrintSettings:(struct OpaquePMPrintSettings *)arg1;
- (struct OpaquePMPrintSettings *)pmPrintSettings;
@end

#if MAC_OS_X_VERSION_MIN_REQUIRED <= MAC_OS_X_VERSION_10_4
// Official 10.5+ methods:
@interface NSPrintInfo (OfficialPMAccessors)
- (void*)PMPageFormat;
- (void*)PMPrintSession; 
- (void*)PMPrintSettings; 
- (void)updateFromPMPageFormat; 
- (void)updateFromPMPrintSettings; 
@end
#endif


PMPrintSettings
nsPrintSettingsX::GetPMPrintSettings()
{
//  return static_cast<PMPrintSettings>([mPrintInfo PMPrintSettings]);
  if ([mPrintInfo respondsToSelector:@selector(PMPrintSettings)])
    return static_cast<PMPrintSettings>([mPrintInfo PMPrintSettings]); // 10.5+

  if ([mPrintInfo respondsToSelector:@selector(pmPrintSettings)])
    return static_cast<PMPrintSettings>([mPrintInfo pmPrintSettings]); // 10.4

  NS_ASSERTION(PR_FALSE, "no way of getting PMPrintSettings from NSPrintInfo");
  PMPrintSettings printSettings;
  PMCreatePrintSettings(&printSettings);
  return printSettings;
}

PMPrintSession
nsPrintSettingsX::GetPMPrintSession()
{
//  return static_cast<PMPrintSession>([mPrintInfo PMPrintSession]);
  if ([mPrintInfo respondsToSelector:@selector(PMPrintSession)])
    return static_cast<PMPrintSession>([mPrintInfo PMPrintSession]); // 10.5+

  if ([mPrintInfo respondsToSelector:@selector(_pmPrintSession)])
    return static_cast<PMPrintSession>([mPrintInfo _pmPrintSession]); // 10.4

  NS_ASSERTION(PR_FALSE, "no way of getting PMPrintSession from NSPrintInfo");
  PMPrintSession printSession;
  PMCreateSession(&printSession);
  return printSession;
}

PMPageFormat
nsPrintSettingsX::GetPMPageFormat()
{
//  return static_cast<PMPageFormat>([mPrintInfo PMPageFormat]);
  if ([mPrintInfo respondsToSelector:@selector(PMPageFormat)])
    return static_cast<PMPageFormat>([mPrintInfo PMPageFormat]); // 10.5+

  if ([mPrintInfo respondsToSelector:@selector(pmPageFormat)])
    return static_cast<PMPageFormat>([mPrintInfo pmPageFormat]); // 10.4

  NS_ASSERTION(PR_FALSE, "no way of getting PMPageFormat from NSPrintInfo");
  PMPageFormat pageFormat;
  PMCreatePageFormat(&pageFormat);
  return pageFormat;
}

void
nsPrintSettingsX::SetPMPageFormat(PMPageFormat aPageFormat)
{
  PMPageFormat oldPageFormat = GetPMPageFormat();
  ::PMCopyPageFormat(aPageFormat, oldPageFormat);
//  [mPrintInfo updateFromPMPageFormat];
  if ([mPrintInfo respondsToSelector:@selector(updateFromPMPageFormat)]) {
    [mPrintInfo updateFromPMPageFormat]; // 10.5+
  } else if ([mPrintInfo respondsToSelector:@selector(setPMPageFormat:)]) {
    [mPrintInfo setPMPageFormat:oldPageFormat]; // 10.4
  }
}

