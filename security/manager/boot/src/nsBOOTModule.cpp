/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/ModuleUtils.h"

#include "CertBlocklist.h"
#include "nsEntropyCollector.h"
#include "nsSecureBrowserUIImpl.h"
#include "nsSiteSecurityService.h"

NS_GENERIC_FACTORY_CONSTRUCTOR(nsEntropyCollector)
NS_GENERIC_FACTORY_CONSTRUCTOR(nsSecureBrowserUIImpl)
NS_GENERIC_FACTORY_CONSTRUCTOR_INIT(CertBlocklist, Init)
NS_GENERIC_FACTORY_CONSTRUCTOR_INIT(nsSiteSecurityService, Init)

NS_DEFINE_NAMED_CID(NS_ENTROPYCOLLECTOR_CID);
NS_DEFINE_NAMED_CID(NS_SECURE_BROWSER_UI_CID);
NS_DEFINE_NAMED_CID(NS_SITE_SECURITY_SERVICE_CID);
NS_DEFINE_NAMED_CID(NS_CERT_BLOCKLIST_CID);

static const mozilla::Module::CIDEntry kBOOTCIDs[] = {
  { &kNS_ENTROPYCOLLECTOR_CID, false, nullptr, nsEntropyCollectorConstructor },
  { &kNS_SECURE_BROWSER_UI_CID, false, nullptr, nsSecureBrowserUIImplConstructor },
  { &kNS_SITE_SECURITY_SERVICE_CID, false, nullptr, nsSiteSecurityServiceConstructor },
  { &kNS_CERT_BLOCKLIST_CID, false, nullptr, CertBlocklistConstructor},
  { nullptr }
};

static const mozilla::Module::ContractIDEntry kBOOTContracts[] = {
  { NS_ENTROPYCOLLECTOR_CONTRACTID, &kNS_ENTROPYCOLLECTOR_CID },
  { NS_SECURE_BROWSER_UI_CONTRACTID, &kNS_SECURE_BROWSER_UI_CID },
  { NS_SSSERVICE_CONTRACTID, &kNS_SITE_SECURITY_SERVICE_CID },
  { NS_CERTBLOCKLIST_CONTRACTID, &kNS_CERT_BLOCKLIST_CID },
  { nullptr }
};

static const mozilla::Module kBootModule = {
  mozilla::Module::kVersion,
  kBOOTCIDs,
  kBOOTContracts
};

NSMODULE_DEFN(BOOT) = &kBootModule;
