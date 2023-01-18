/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef gfxMacPlatformFontList_H_
#define gfxMacPlatformFontList_H_

#include <CoreFoundation/CoreFoundation.h>
#include <Carbon/Carbon.h>

#include "mozilla/MemoryReporting.h"
#include "nsDataHashtable.h"
#include "nsRefPtrHashtable.h"

#include "gfxPlatformFontList.h"
#include "gfxPlatform.h"
#include "gfxPlatformMac.h"

#include "nsUnicharUtils.h"
#include "nsTArray.h"

class gfxMacPlatformFontList;

// a single member of a font family (i.e. a single face, such as Times Italic)
class MacOSFontEntry : public gfxFontEntry
{
public:
    friend class gfxMacPlatformFontList;

    MacOSFontEntry(const nsAString& aPostscriptName, int32_t aWeight,
                   bool aIsStandardFace = false);

    // for use with data fonts
#if(0)
    MacOSFontEntry(const nsAString& aPostscriptName, CGFontRef aFontRef,
#else
    MacOSFontEntry(const nsAString& aPostscriptName, ATSFontRef aFontRef,
#endif
                   uint16_t aWeight, uint16_t aStretch, uint32_t aItalicStyle,
                   ATSFontContainerRef aContainerRef, // 10.4Fx
                   bool aIsDataUserFont, bool aIsLocal);

    virtual ~MacOSFontEntry() {
	if (mFontRefInitialized)
        ::CGFontRelease(mFontRef);
	/* Per Apple, even synthesized CGFontRefs must be released. Also,
	   we do need to release our container ref, if any. */
	if (mContainerRef)
		::ATSFontDeactivate(mContainerRef, NULL,
			kATSOptionFlagsDefault);
    }

    // 10.4Fx
    ATSFontRef GetATSFontRef();

    virtual CGFontRef GetFontRef();

    // override gfxFontEntry table access function to bypass table cache,
    // use CGFontRef API to get direct access to system font data
    virtual hb_blob_t *GetFontTable(uint32_t aTag) override;

    virtual void AddSizeOfIncludingThis(mozilla::MallocSizeOf aMallocSizeOf,
                                        FontListSizes* aSizes) const override;

    nsresult ReadCMAP(FontInfoData *aFontInfoData = nullptr) override;

    bool RequiresAATLayout() const { return mRequiresAAT; }

    bool IsCFF();

protected:
    virtual gfxFont* CreateFontInstance(const gfxFontStyle *aFontStyle, bool aNeedsBold) override;

    virtual bool HasFontTable(uint32_t aTableTag) override;

    static void DestroyBlobFunc(void* aUserData);

    CGFontRef mFontRef; // owning reference to the CGFont, released on destruction

    // 10.4Fx class variables
    ATSFontRef mATSFontRef; // 10.4Fx (owning reference to our ATSFont)
    ATSFontContainerRef mContainerRef; // 10.4Fx (for MakePlatformFont)
    bool mATSFontRefInitialized; // 10.4Fx. mUserFontData is in gfxFont.h.
    AutoFallibleTArray<uint8_t,1024> mFontTableDir; // 10.4Fx
    ByteCount mFontTableDirSize; // 10.4Fx
    void TryGlobalFontTableCache();

    bool mFontRefInitialized;
    bool mRequiresAAT;
    bool mIsCFF;
    bool mIsCFFInitialized;
};

class gfxMacPlatformFontList : public gfxPlatformFontList {
public:
    static gfxMacPlatformFontList* PlatformFontList() {
        return static_cast<gfxMacPlatformFontList*>(sPlatformFontList);
    }

    static int32_t AppleWeightToCSSWeight(int32_t aAppleWeight);

    virtual gfxFontFamily* GetDefaultFont(const gfxFontStyle* aStyle);

    virtual bool GetStandardFamilyName(const nsAString& aFontName, nsAString& aFamilyName);

    virtual gfxFontEntry* LookupLocalFont(const nsAString& aFontName,
                                          uint16_t aWeight,
                                          int16_t aStretch,
                                          bool aItalic);
    
    virtual gfxFontEntry* MakePlatformFont(const nsAString& aFontName,
                                           uint16_t aWeight,
                                           int16_t aStretch,
                                           bool aItalic,
                                           const uint8_t* aFontData,
                                           uint32_t aLength);

    void SetFixedPitch(const nsAString& aFamilyName); // 10.4Fx

private:
    friend class gfxPlatformMac;

    gfxMacPlatformFontList();
    virtual ~gfxMacPlatformFontList();

    // initialize font lists
    virtual nsresult InitFontList();

    // special case font faces treated as font families (set via prefs)
    void InitSingleFaceList();

#if(0)
    static void RegisteredFontsChangedNotificationCallback(CFNotificationCenterRef center,
                                                           void *observer,
                                                           CFStringRef name,
                                                           const void *object,
                                                           CFDictionaryRef userInfo);
#else
    // eliminate faces which have the same ATS font reference
    // backout bug 663688
    void EliminateDuplicateFaces(const nsAString& aFamilyName);

	// backout bug 869762
	static void ATSNotification(ATSFontNotificationInfoRef aInfo, void* aUserArg);
	uint32_t mATSGeneration;
#endif

    // search fonts system-wide for a given character, null otherwise
    virtual gfxFontEntry* GlobalFontFallback(const uint32_t aCh,
                                             int32_t aRunScript,
                                             const gfxFontStyle* aMatchStyle,
                                             uint32_t& aCmapCount,
                                             gfxFontFamily** aMatchedFamily);

    virtual bool UsesSystemFallback() { return true; }

    virtual already_AddRefed<FontInfoData> CreateFontInfoData();

#ifdef MOZ_BUNDLED_FONTS
    void ActivateBundledFonts();
#endif

    enum {
        kATSGenerationInitial = -1
    };

    // default font for use with system-wide font fallback
#if(0)
    CTFontRef mDefaultFont;
#else
    ATSFontRef mDefaultFont;
#endif
};

#endif /* gfxMacPlatformFontList_H_ */
