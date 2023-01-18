/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef GFX_PLATFORM_MAC_H
#define GFX_PLATFORM_MAC_H

#include "nsTArrayForwardDeclare.h"
#include "gfxPlatform.h"
#include "nsDataHashtable.h"
#include "nsClassHashtable.h"

typedef size_t ByteCount;

namespace mozilla {
namespace gfx {
class DrawTarget;
class VsyncSource;
} // gfx
} // mozilla

// 10.4Fx
class FontDirWrapper {
public:
	uint8_t fontDir[1024];
	ByteCount sizer;
	FontDirWrapper::FontDirWrapper(ByteCount sized, uint8_t *dir) {
		if (sized < 0 || sized > 1024) return;
		sizer = sized;
		memcpy(fontDir, dir, sizer);
	}
	FontDirWrapper::~FontDirWrapper() { }
};

class gfxPlatformMac : public gfxPlatform {
public:
    gfxPlatformMac();
    virtual ~gfxPlatformMac();

    static gfxPlatformMac *GetPlatform() {
        return (gfxPlatformMac*) gfxPlatform::GetPlatform();
    }

    virtual already_AddRefed<gfxASurface>
      CreateOffscreenSurface(const IntSize& size,
                             gfxContentType contentType) override;

    mozilla::TemporaryRef<mozilla::gfx::ScaledFont>
      GetScaledFontForFont(mozilla::gfx::DrawTarget* aTarget, gfxFont *aFont) override;

    nsresult GetStandardFamilyName(const nsAString& aFontName, nsAString& aFamilyName) override;

    gfxFontGroup*
    CreateFontGroup(const mozilla::FontFamilyList& aFontFamilyList,
                    const gfxFontStyle *aStyle,
                    gfxUserFontSet *aUserFontSet) override;

    virtual gfxFontEntry* LookupLocalFont(const nsAString& aFontName,
                                          uint16_t aWeight,
                                          int16_t aStretch,
                                          bool aItalic) override;

    virtual gfxPlatformFontList* CreatePlatformFontList() override;

    virtual gfxFontEntry* MakePlatformFont(const nsAString& aFontName,
                                           uint16_t aWeight,
                                           int16_t aStretch,
                                           bool aItalic,
                                           const uint8_t* aFontData,
                                           uint32_t aLength) override;

    bool IsFontFormatSupported(nsIURI *aFontURI, uint32_t aFormatFlags) override;

    nsresult GetFontList(nsIAtom *aLangGroup,
                         const nsACString& aGenericFamily,
                         nsTArray<nsString>& aListOfFonts) override;
    nsresult UpdateFontList() override;

    virtual void GetCommonFallbackFonts(uint32_t aCh, uint32_t aNextCh,
                                        int32_t aRunScript,
                                        nsTArray<const char*>& aFontList) override;

    virtual bool CanRenderContentToDataSurface() const override {
      return true;
    }

    bool UseAcceleratedCanvas();

    virtual bool UseProgressivePaint() override;
    virtual already_AddRefed<mozilla::gfx::VsyncSource> CreateHardwareVsyncSource() override;

    // lower threshold on font anti-aliasing
    uint32_t GetAntiAliasingThreshold() { return mFontAntiAliasingThreshold; }

/* ATS acceleration functions for 10.4 */
ByteCount GetCachedDirSizeForFont(nsString name);
uint8_t *GetCachedDirForFont(nsString name);
void SetCachedDirForFont(nsString name, uint8_t* table, ByteCount sizer);
nsClassHashtable< nsStringHashKey, FontDirWrapper > PlatformFontDirCache;

private:
    virtual void GetPlatformCMSOutputProfile(void* &mem, size_t &size) override;

    // read in the pref value for the lower threshold on font anti-aliasing
    static uint32_t ReadAntiAliasingThreshold();

    uint32_t mFontAntiAliasingThreshold;
    
    int32_t mOSXVersion; // 10.4
};

#endif /* GFX_PLATFORM_MAC_H */
