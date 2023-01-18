/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

const {Cc, Ci, Cu, CC} = require("chrome");
const Services = require("Services");
const protocol = require("devtools/server/protocol");
const {method, RetVal} = protocol;
const {Promise: promise} = Cu.import("resource://gre/modules/Promise.jsm", {});
const {LongStringActor} = require("devtools/server/actors/string");
const {DebuggerServer} = require("devtools/server/main");

Cu.import("resource://gre/modules/PermissionsTable.jsm")

const APP_MAP = {
  '{ec8030f7-c20a-464f-9b0e-13a3a9e97384}': 'firefox',
  '{3550f703-e582-4d05-9a08-453d09bdfdc6}': 'thunderbird',
  '{92650c4d-4b8e-4d2a-b7eb-24ecf4f6b63a}': 'seamonkey',
  '{718e30fb-e89b-41dd-9da7-e25a45638b28}': 'sunbird',
  '{3c2e2abc-06d4-11e1-ac3b-374f68613e61}': 'b2g',
  '{aa3c5121-dab2-40e2-81ca-7ea25febc110}': 'mobile/android',
  '{a23983c0-fd0e-11dc-95ff-0800200c9a66}': 'mobile/xul'
}

let DeviceActor = exports.DeviceActor = protocol.ActorClass({
  typeName: "device",

  _desc: null,

  _getAppIniString : function(section, key) {
    let inifile = Services.dirsvc.get("GreD", Ci.nsIFile);
    inifile.append("application.ini");

    if (!inifile.exists()) {
      inifile = Services.dirsvc.get("CurProcD", Ci.nsIFile);
      inifile.append("application.ini");
    }

    if (!inifile.exists()) {
      return undefined;
    }

    let iniParser = Cc["@mozilla.org/xpcom/ini-parser-factory;1"].getService(Ci.nsIINIParserFactory).createINIParser(inifile);
    try {
      return iniParser.getString(section, key);
    } catch (e) {
      return undefined;
    }
  },

  _getSetting: function(name) {
    let deferred = promise.defer();

    if ("@mozilla.org/settingsService;1" in Cc) {
      let settingsService = Cc["@mozilla.org/settingsService;1"].getService(Ci.nsISettingsService);
      let req = settingsService.createLock().get(name, {
        handle: (name, value) => deferred.resolve(value),
        handleError: (error) => deferred.reject(error),
      });
    } else {
      deferred.reject(new Error("No settings service"));
    }
    return deferred.promise;
  },

  getDescription: method(function() {
    // Most of this code is inspired from Nightly Tester Tools:
    // https://wiki.mozilla.org/Auto-tools/Projects/NightlyTesterTools

    let appInfo = Services.appinfo;
    let win = Services.wm.getMostRecentWindow(DebuggerServer.chromeWindowType);

    desc = {
      appid: appInfo.ID,
      apptype: APP_MAP[appInfo.ID],
      vendor: appInfo.vendor,
      name: appInfo.name,
      version: appInfo.version,
      appbuildid: appInfo.appBuildID,
      platformbuildid: appInfo.platformBuildID,
      platformversion: appInfo.platformVersion,
      geckobuildid: appInfo.platformBuildID,
      geckoversion: appInfo.platformVersion,
      changeset: this._getAppIniString("App", "SourceStamp"),
      locale: Cc["@mozilla.org/chrome/chrome-registry;1"].getService(Ci.nsIXULChromeRegistry).getSelectedLocale("global"),
      os: null,
      hardware: "unknown",
      processor: appInfo.XPCOMABI.split("-")[0],
      compiler: appInfo.XPCOMABI.split("-")[1],
      brandName: null,
      channel: null,
      profile: null,
    };
    if (win) {
      let utils = win.QueryInterface(Ci.nsIInterfaceRequestor).getInterface(Ci.nsIDOMWindowUtils);
      desc.dpi = utils.displayDPI;
      desc.useragent = win.navigator.userAgent;
      desc.width = win.screen.width;
      desc.height = win.screen.height;
    }

    // Profile
    let profd = Services.dirsvc.get("ProfD", Ci.nsILocalFile);
    let profservice = Cc["@mozilla.org/toolkit/profile-service;1"].getService(Ci.nsIToolkitProfileService);
    var profiles = profservice.profiles;
    while (profiles.hasMoreElements()) {
      let profile = profiles.getNext().QueryInterface(Ci.nsIToolkitProfile);
      if (profile.rootDir.path == profd.path) {
        desc.profile = profile.name;
        break;
      }
    }

    if (!desc.profile) {
      desc.profile = profd.leafName;
    }

    // Channel
    try {
      desc.channel = Services.prefs.getCharPref('app.update.channel');
    } catch(e) {}

    if (desc.apptype == "b2g") {
      // B2G specific
      desc.os = "B2G";

      return this._getSetting('deviceinfo.hardware')
      .then(value => desc.hardware = value)
      .then(() => this._getSetting('deviceinfo.os'))
      .then(value => desc.version = value)
      .then(() => desc);
    }

    // Not B2G
    desc.os = appInfo.OS;
    let bundle = Services.strings.createBundle("chrome://branding/locale/brand.properties");
    if (bundle) {
      desc.brandName = bundle.GetStringFromName("brandFullName");
    }

    return desc;

  }, {request: {},response: { value: RetVal("json")}}),

  getWallpaper: method(function() {
    let deferred = promise.defer();
    this._getSetting("wallpaper.image").then((blob) => {
      let FileReader = CC("@mozilla.org/files/filereader;1");
      let reader = new FileReader();
      let conn = this.conn;
      reader.addEventListener("load", function() {
        let str = new LongStringActor(conn, reader.result);
        deferred.resolve(str);
      });
      reader.addEventListener("error", function() {
        deferred.reject(reader.error);
      });
      reader.readAsDataURL(blob);
    });
    return deferred.promise;
  }, {request: {},response: { value: RetVal("longstring")}}),

  screenshotToDataURL: method(function() {
    let window = Services.wm.getMostRecentWindow(DebuggerServer.chromeWindowType);
    let canvas = window.document.createElementNS("http://www.w3.org/1999/xhtml", "canvas");
    let width = window.innerWidth;
    let height = window.innerHeight;
    canvas.setAttribute('width', width);
    canvas.setAttribute('height', height);
    let context = canvas.getContext('2d');
    let flags =
          context.DRAWWINDOW_DRAW_CARET |
          context.DRAWWINDOW_DRAW_VIEW |
          context.DRAWWINDOW_USE_WIDGET_LAYERS;
    context.drawWindow(window, 0, 0, width, height, 'rgb(255,255,255)', flags);
    let dataURL = canvas.toDataURL('image/png')
    return new LongStringActor(this.conn, dataURL);
  }, {request: {},response: { value: RetVal("longstring")}}),

  getRawPermissionsTable: method(function() {
    return {
      rawPermissionsTable: PermissionsTable,
      UNKNOWN_ACTION: Ci.nsIPermissionManager.UNKNOWN_ACTION,
      ALLOW_ACTION: Ci.nsIPermissionManager.ALLOW_ACTION,
      DENY_ACTION: Ci.nsIPermissionManager.DENY_ACTION,
      PROMPT_ACTION: Ci.nsIPermissionManager.PROMPT_ACTION
    };
  }, {request: {},response: { value: RetVal("json")}})
});

let DeviceFront = protocol.FrontClass(DeviceActor, {
  initialize: function(client, form) {
    protocol.Front.prototype.initialize.call(this, client);
    this.actorID = form.deviceActor;
    this.manage(this);
  },

  screenshotToBlob: function() {
    return this.screenshotToDataURL().then(longstr => {
      return longstr.string().then(dataURL => {
        let deferred = promise.defer();
        longstr.release().then(null, Cu.reportError);
        let req = Cc["@mozilla.org/xmlextras/xmlhttprequest;1"].createInstance(Ci.nsIXMLHttpRequest);
        req.open("GET", dataURL, true);
        req.responseType = "blob";
        req.onload = () => {
          deferred.resolve(req.response);
        };
        req.onerror = () => {
          deferred.reject(req.status);
        }
        req.send();
        return deferred.promise;
      });
    });
  },
});

const _knownDeviceFronts = new WeakMap();

exports.getDeviceFront = function(client, form) {
  if (!form.deviceActor) {
    return null;
  }

  if (_knownDeviceFronts.has(client)) {
    return _knownDeviceFronts.get(client);
  }

  let front = new DeviceFront(client, form);
  _knownDeviceFronts.set(client, front);
  return front;
};
