/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

const Cc = Components.classes;
const Ci = Components.interfaces;
const Cu = Components.utils;
const Cr = Components.results;

const kMainKey = "Software\\Microsoft\\Internet Explorer\\Main";
const kRegMultiSz = 7;

Cu.import("resource://gre/modules/XPCOMUtils.jsm");
Cu.import("resource://gre/modules/Services.jsm");
Cu.import("resource://gre/modules/NetUtil.jsm");
Cu.import("resource:///modules/MigrationUtils.jsm");

XPCOMUtils.defineLazyModuleGetter(this, "PlacesUtils",
                                  "resource://gre/modules/PlacesUtils.jsm");
XPCOMUtils.defineLazyModuleGetter(this, "ctypes",
                                  "resource://gre/modules/ctypes.jsm");
XPCOMUtils.defineLazyModuleGetter(this, "WindowsRegistry",
                                  "resource://gre/modules/WindowsRegistry.jsm");

Cu.importGlobalProperties(["File"]);

////////////////////////////////////////////////////////////////////////////////
//// Helpers.

let CtypesHelpers = {
  _structs: {},
  _functions: {},
  _libs: {},

  /**
   * Must be invoked once before first use of any of the provided helpers.
   */
  initialize: function CH_initialize() {
    const WORD = ctypes.uint16_t;
    const DWORD = ctypes.uint32_t;
    const BOOL = ctypes.int;

    this._structs.SYSTEMTIME = new ctypes.StructType('SYSTEMTIME', [
      {wYear: WORD},
      {wMonth: WORD},
      {wDayOfWeek: WORD},
      {wDay: WORD},
      {wHour: WORD},
      {wMinute: WORD},
      {wSecond: WORD},
      {wMilliseconds: WORD}
    ]);

    this._structs.FILETIME = new ctypes.StructType('FILETIME', [
      {dwLowDateTime: DWORD},
      {dwHighDateTime: DWORD}
    ]);

    try {
      this._libs.kernel32 = ctypes.open("Kernel32");
      this._functions.FileTimeToSystemTime =
        this._libs.kernel32.declare("FileTimeToSystemTime",
                                    ctypes.default_abi,
                                    BOOL,
                                    this._structs.FILETIME.ptr,
                                    this._structs.SYSTEMTIME.ptr);
    } catch (ex) {
      this.finalize();
    }
  },

  /**
   * Must be invoked once after last use of any of the provided helpers.
   */
  finalize: function CH_finalize() {
    this._structs = {};
    this._functions = {};
    for each (let lib in this._libs) {
      try {
        lib.close();
      } catch (ex) {}
    }
    this._libs = {};
  },

  /**
   * Converts a FILETIME struct (2 DWORDS), to a SYSTEMTIME struct.
   *
   * @param aTimeHi
   *        Least significant DWORD.
   * @param aTimeLo
   *        Most significant DWORD.
   * @return a Date object representing the converted datetime.
   */
  fileTimeToDate: function CH_fileTimeToDate(aTimeHi, aTimeLo) {
    let fileTime = this._structs.FILETIME();
    fileTime.dwLowDateTime = aTimeLo;
    fileTime.dwHighDateTime = aTimeHi;
    let systemTime = this._structs.SYSTEMTIME();
    let result = this._functions.FileTimeToSystemTime(fileTime.address(),
                                                      systemTime.address());
    if (result == 0)
      throw new Error(ctypes.winLastError);

    return new Date(systemTime.wYear,
                    systemTime.wMonth - 1,
                    systemTime.wDay,
                    systemTime.wHour,
                    systemTime.wMinute,
                    systemTime.wSecond,
                    systemTime.wMilliseconds);
  }
};

/**
 * Checks whether an host is an IP (v4 or v6) address.
 *
 * @param aHost
 *        The host to check.
 * @return whether aHost is an IP address.
 */
function hostIsIPAddress(aHost) {
  try {
    Services.eTLD.getBaseDomainFromHost(aHost);
  } catch (e if e.result == Cr.NS_ERROR_HOST_IS_IP_ADDRESS) {
    return true;
  } catch (e) {}
  return false;
}

////////////////////////////////////////////////////////////////////////////////
//// Resources

function Bookmarks() {
}

Bookmarks.prototype = {
  type: MigrationUtils.resourceTypes.BOOKMARKS,

  get exists() !!this._favoritesFolder,

  __favoritesFolder: null,
  get _favoritesFolder() {
    if (!this.__favoritesFolder) {
      let favoritesFolder = Services.dirsvc.get("Favs", Ci.nsIFile);
      if (favoritesFolder.exists() && favoritesFolder.isReadable())
        this.__favoritesFolder = favoritesFolder;
    }
    return this.__favoritesFolder;
  },

  __toolbarFolderName: null,
  get _toolbarFolderName() {
    if (!this.__toolbarFolderName) {
      // Retrieve the name of IE's favorites subfolder that holds the bookmarks
      // in the toolbar. This was previously stored in the registry and changed
      // in IE7 to always be called "Links".
      let folderName = WindowsRegistry.readRegKey(Ci.nsIWindowsRegKey.ROOT_KEY_CURRENT_USER,
                                                  "Software\\Microsoft\\Internet Explorer\\Toolbar",
                                                  "LinksFolderName");
      this.__toolbarFolderName = folderName || "Links";
    }
    return this.__toolbarFolderName;
  },

  migrate: function B_migrate(aCallback) {
    PlacesUtils.bookmarks.runInBatchMode({
      runBatched: (function migrateBatched() {
        // Import to the bookmarks menu.
        let destFolderId = PlacesUtils.bookmarksMenuFolderId;
        if (!MigrationUtils.isStartupMigration) {
          destFolderId =
            MigrationUtils.createImportedBookmarksFolder("IE", destFolderId);
        }

        this._migrateFolder(this._favoritesFolder, destFolderId);

        aCallback(true);
      }).bind(this)
    }, null);
  },

  _migrateFolder: function B__migrateFolder(aSourceFolder, aDestFolderId) {
    // TODO (bug 741993): the favorites order is stored in the Registry, at
    // HCU\Software\Microsoft\Windows\CurrentVersion\Explorer\MenuOrder\Favorites
    // Until we support it, bookmarks are imported in alphabetical order.
    let entries = aSourceFolder.directoryEntries;
    while (entries.hasMoreElements()) {
      let entry = entries.getNext().QueryInterface(Ci.nsIFile);

      // Make sure that entry.path == entry.target to not follow .lnk folder
      // shortcuts which could lead to infinite cycles.
      if (entry.isDirectory() && entry.path == entry.target) {
        let destFolderId;
        if (entry.leafName == this._toolbarFolderName &&
            entry.parent.equals(this._favoritesFolder)) {
          // Import to the bookmarks toolbar.
          destFolderId = PlacesUtils.toolbarFolderId;
          if (!MigrationUtils.isStartupMigration) {
            destFolderId =
              MigrationUtils.createImportedBookmarksFolder("IE", destFolderId);
          }
        }
        else {
          // Import to a new folder.
          destFolderId =
            PlacesUtils.bookmarks.createFolder(aDestFolderId, entry.leafName,
                                               PlacesUtils.bookmarks.DEFAULT_INDEX);
        }

        if (entry.isReadable()) {
          // Recursively import the folder.
          this._migrateFolder(entry, destFolderId);
        }
      }
      else {
        // Strip the .url extension, to both check this is a valid link file,
        // and get the associated title.
        let matches = entry.leafName.match(/(.+)\.url$/i);
        if (matches) {
          let fileHandler = Cc["@mozilla.org/network/protocol;1?name=file"].
                            getService(Ci.nsIFileProtocolHandler);
          let uri = fileHandler.readURLFile(entry);
          let title = matches[1];

          PlacesUtils.bookmarks.insertBookmark(aDestFolderId,
                                               uri,
                                               PlacesUtils.bookmarks.DEFAULT_INDEX,
                                               title);
        }
      }
    }
  }
};

function History() {
}

History.prototype = {
  type: MigrationUtils.resourceTypes.HISTORY,

  get exists() true,

  __typedURLs: null,
  get _typedURLs() {
    if (!this.__typedURLs) {
      // The list of typed URLs is a sort of annotation stored in the registry.
      // Currently, IE stores 25 entries and this value is not configurable,
      // but we just keep reading up to the first non-existing entry to support
      // possible future bumps of this limit.
      this.__typedURLs = {};
      let registry = Cc["@mozilla.org/windows-registry-key;1"].
                     createInstance(Ci.nsIWindowsRegKey);
      try {
        registry.open(Ci.nsIWindowsRegKey.ROOT_KEY_CURRENT_USER,
                      "Software\\Microsoft\\Internet Explorer\\TypedURLs",
                      Ci.nsIWindowsRegKey.ACCESS_READ);
        for (let entry = 1; registry.hasValue("url" + entry); entry++) {
          let url = registry.readStringValue("url" + entry);
          this.__typedURLs[url] = true;
        }
      } catch (ex) {
      } finally {
        registry.close();
      }
    }
    return this.__typedURLs;
  },

  migrate: function H_migrate(aCallback) {
    let places = [];
    let historyEnumerator = Cc["@mozilla.org/profile/migrator/iehistoryenumerator;1"].
                            createInstance(Ci.nsISimpleEnumerator);
    while (historyEnumerator.hasMoreElements()) {
      let entry = historyEnumerator.getNext().QueryInterface(Ci.nsIPropertyBag2);
      let uri = entry.get("uri").QueryInterface(Ci.nsIURI);
      // MSIE stores some types of URLs in its history that we don't handle,
      // like HTMLHelp and others.  Since we don't properly map handling for
      // all of them we just avoid importing them.
      if (["http", "https", "ftp", "file"].indexOf(uri.scheme) == -1) {
        continue;
      }

      let title = entry.get("title");
      // Embed visits have no title and don't need to be imported.
      if (title.length == 0) {
        continue;
      }

      // The typed urls are already fixed-up, so we can use them for comparison.
      let transitionType = this._typedURLs[uri.spec] ?
                             Ci.nsINavHistoryService.TRANSITION_TYPED :
                             Ci.nsINavHistoryService.TRANSITION_LINK;
      let lastVisitTime = entry.get("time");

      places.push(
        { uri: uri,
          title: title,
          visits: [{ transitionType: transitionType,
                     visitDate: lastVisitTime }]
        }
      );
    }

    // Check whether there is any history to import.
    if (places.length == 0) {
      aCallback(true);
      return;
    }

    PlacesUtils.asyncHistory.updatePlaces(places, {
      _success: false,
      handleResult: function() {
        // Importing any entry is considered a successful import.
        this._success = true;
      },
      handleError: function() {},
      handleCompletion: function() {
        aCallback(this._success);
      }
    });
  }
};

function Cookies() {
}

Cookies.prototype = {
  type: MigrationUtils.resourceTypes.COOKIES,

  get exists() !!this._cookiesFolder,

  __cookiesFolder: null,
  get _cookiesFolder() {
    // Cookies are stored in txt files, in a Cookies folder whose path varies
    // across the different OS versions.  CookD takes care of most of these
    // cases, though, in Windows Vista/7, UAC makes a difference.
    // If UAC is enabled, the most common destination is CookD/Low.  Though,
    // if the user runs the application in administrator mode or disables UAC,
    // cookies are stored in the original CookD destination.  Cause running the
    // browser in administrator mode is unsafe and discouraged, we just care
    // about the UAC state.
    if (!this.__cookiesFolder) {
      let cookiesFolder = Services.dirsvc.get("CookD", Ci.nsIFile);
      if (cookiesFolder.exists() && cookiesFolder.isReadable()) {
        // Check if UAC is enabled.
        if (Services.appinfo.QueryInterface(Ci.nsIWinAppHelper).userCanElevate) {
          cookiesFolder.append("Low");
        }
        this.__cookiesFolder = cookiesFolder;
      }
    }
    return this.__cookiesFolder;
  },

  migrate: function C_migrate(aCallback) {
    CtypesHelpers.initialize();

    let cookiesGenerator = (function genCookie() {
      let success = false;
      let entries = this._cookiesFolder.directoryEntries;
      while (entries.hasMoreElements()) {
        let entry = entries.getNext().QueryInterface(Ci.nsIFile);
        // Skip eventual bogus entries.
        if (!entry.isFile() || !/\.txt$/.test(entry.leafName))
          continue;

        this._readCookieFile(entry, function(aSuccess) {
          // Importing even a single cookie file is considered a success.
          if (aSuccess)
            success = true;
          try {
            cookiesGenerator.next();
          } catch (ex) {}
        });

        yield;
      }

      CtypesHelpers.finalize();

      aCallback(success);
    }).apply(this);
    cookiesGenerator.next();
  },

  _readCookieFile: function C__readCookieFile(aFile, aCallback) {
    let fileReader = Cc["@mozilla.org/files/filereader;1"].
                     createInstance(Ci.nsIDOMFileReader);
    fileReader.addEventListener("loadend", (function onLoadEnd() {
      fileReader.removeEventListener("loadend", onLoadEnd, false);

      if (fileReader.readyState != fileReader.DONE) {
        Cu.reportError("Could not read cookie contents: " + fileReader.error);
        aCallback(false);
        return;
      }

      let success = true;
      try {
        this._parseCookieBuffer(fileReader.result);
      } catch (ex) {
        Components.utils.reportError("Unable to migrate cookie: " + ex);
        success = false;
      } finally {
        aCallback(success);
      }
    }).bind(this), false);
    fileReader.readAsText(new File(aFile));
  },

  /**
   * Parses a cookie file buffer and returns an array of the contained cookies.
   *
   * The cookie file format is a newline-separated-values with a "*" used as
   * delimeter between multiple records.
   * Each cookie has the following fields:
   *  - name
   *  - value
   *  - host/path
   *  - flags
   *  - Expiration time most significant integer
   *  - Expiration time least significant integer
   *  - Creation time most significant integer
   *  - Creation time least significant integer
   *  - Record delimiter "*"
   *
   * @note All the times are in FILETIME format.
   */
  _parseCookieBuffer: function C__parseCookieBuffer(aTextBuffer) {
    // Note the last record is an empty string.
    let records = [r for each (r in aTextBuffer.split("*\n")) if (r)];
    for (let record of records) {
      let [name, value, hostpath, flags,
           expireTimeLo, expireTimeHi] = record.split("\n");

      // IE stores deleted cookies with a zero-length value, skip them.
      if (value.length == 0)
        continue;

      let hostLen = hostpath.indexOf("/");
      let host = hostpath.substr(0, hostLen);
      let path = hostpath.substr(hostLen);

      // For a non-null domain, assume it's what Mozilla considers
      // a domain cookie.  See bug 222343.
      if (host.length > 0) {
        // Fist delete any possible extant matching host cookie.
        Services.cookies.remove(host, name, path, false);
        // Now make it a domain cookie.
        if (host[0] != "." && !hostIsIPAddress(host))
          host = "." + host;
      }

      let expireTime = CtypesHelpers.fileTimeToDate(Number(expireTimeHi),
                                                    Number(expireTimeLo));
      Services.cookies.add(host,
                           path,
                           name,
                           value,
                           Number(flags) & 0x1, // secure
                           false, // httpOnly
                           false, // session
                           expireTime);
    }
  }
};


////////////////////////////////////////////////////////////////////////////////
//// Migrator

function IEProfileMigrator()
{
}

IEProfileMigrator.prototype = Object.create(MigratorPrototype);

IEProfileMigrator.prototype.getResources = function IE_getResources() {
  let resources = [
    new Bookmarks()
  , new History()
  , new Cookies()
  ];
  return [r for each (r in resources) if (r.exists)];
};

Object.defineProperty(IEProfileMigrator.prototype, "sourceHomePageURL", {
  get: function IE_get_sourceHomePageURL() {
    let defaultStartPage = WindowsRegistry.readRegKey(Ci.nsIWindowsRegKey.ROOT_KEY_LOCAL_MACHINE,
                                                      kMainKey, "Default_Page_URL");
    let startPage = WindowsRegistry.readRegKey(Ci.nsIWindowsRegKey.ROOT_KEY_CURRENT_USER,
                                               kMainKey, "Start Page");
    // If the user didn't customize the Start Page, he is still on the default
    // page, that may be considered the equivalent of our about:home.  There's
    // no reason to retain it, since it is heavily targeted to IE.
    let homepage = startPage != defaultStartPage ? startPage : "";

    // IE7+ supports secondary home pages located in a REG_MULTI_SZ key.  These
    // are in addition to the Start Page, and no empty entries are possible,
    // thus a Start Page is always defined if any of these exists, though it
    // may be the default one.
    let secondaryPages = WindowsRegistry.readRegKey(Ci.nsIWindowsRegKey.ROOT_KEY_CURRENT_USER,
                                                    kMainKey, "Secondary Start Pages");
    if (secondaryPages) {
      if (homepage)
        secondaryPages.unshift(homepage);
      homepage = secondaryPages.join("|");
    }

    return homepage;
  }
});

IEProfileMigrator.prototype.classDescription = "IE Profile Migrator";
IEProfileMigrator.prototype.contractID = "@mozilla.org/profile/migrator;1?app=browser&type=ie";
IEProfileMigrator.prototype.classID = Components.ID("{3d2532e3-4932-4774-b7ba-968f5899d3a4}");

this.NSGetFactory = XPCOMUtils.generateNSGetFactory([IEProfileMigrator]);
