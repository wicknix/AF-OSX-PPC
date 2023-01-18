/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_archivereader_domarchivefile_h__
#define mozilla_dom_archivereader_domarchivefile_h__

#include "mozilla/Attributes.h"
#include "mozilla/ErrorResult.h"
#include "mozilla/dom/File.h"

#include "ArchiveReader.h"

#include "ArchiveReaderCommon.h"
#include "zipstruct.h"

BEGIN_ARCHIVEREADER_NAMESPACE

/**
 * ArchiveZipFileImpl to FileImpl
 */
class ArchiveZipFileImpl : public FileImplBase
{
public:
  NS_DECL_ISUPPORTS_INHERITED

  ArchiveZipFileImpl(const nsAString& aName,
                     const nsAString& aContentType,
                     uint64_t aLength,
                     ZipCentral& aCentral,
                     FileImpl* aFileImpl)
  : FileImplBase(aName, aContentType, aLength),
    mCentral(aCentral),
    mFileImpl(aFileImpl),
    mFilename(aName)
  {
    MOZ_ASSERT(mFileImpl);
    MOZ_COUNT_CTOR(ArchiveZipFileImpl);
  }

  ArchiveZipFileImpl(const nsAString& aName,
                     const nsAString& aContentType,
                     uint64_t aStart,
                     uint64_t aLength,
                     ZipCentral& aCentral,
                     FileImpl* aFileImpl)
  : FileImplBase(aContentType, aStart, aLength),
    mCentral(aCentral),
    mFileImpl(aFileImpl),
    mFilename(aName)
  {
    MOZ_ASSERT(mFileImpl);
    MOZ_COUNT_CTOR(ArchiveZipFileImpl);
  }

  // Overrides:
  virtual nsresult GetInternalStream(nsIInputStream**) override;
protected:
  virtual ~ArchiveZipFileImpl()
  {
    MOZ_COUNT_DTOR(ArchiveZipFileImpl);
  }

  virtual already_AddRefed<FileImpl>
  CreateSlice(uint64_t aStart, uint64_t aLength, const nsAString& aContentType,
              mozilla::ErrorResult& aRv) override;

private: // Data
  ZipCentral mCentral;
  nsRefPtr<FileImpl> mFileImpl;

  nsString mFilename;
};

END_ARCHIVEREADER_NAMESPACE

#endif // mozilla_dom_archivereader_domarchivefile_h__
