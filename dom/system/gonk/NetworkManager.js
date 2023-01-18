/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

const {classes: Cc, interfaces: Ci, utils: Cu, results: Cr} = Components;

Cu.import("resource://gre/modules/XPCOMUtils.jsm");
Cu.import("resource://gre/modules/Services.jsm");
Cu.import("resource://gre/modules/FileUtils.jsm");
Cu.import("resource://gre/modules/systemlibs.js");
Cu.import("resource://gre/modules/Promise.jsm");

const NETWORKMANAGER_CONTRACTID = "@mozilla.org/network/manager;1";
const NETWORKMANAGER_CID =
  Components.ID("{33901e46-33b8-11e1-9869-f46d04d25bcc}");

const DEFAULT_PREFERRED_NETWORK_TYPE = Ci.nsINetworkInterface.NETWORK_TYPE_WIFI;

XPCOMUtils.defineLazyServiceGetter(this, "gSettingsService",
                                   "@mozilla.org/settingsService;1",
                                   "nsISettingsService");
XPCOMUtils.defineLazyGetter(this, "ppmm", function() {
  return Cc["@mozilla.org/parentprocessmessagemanager;1"]
         .getService(Ci.nsIMessageBroadcaster);
});

XPCOMUtils.defineLazyServiceGetter(this, "gDNSService",
                                   "@mozilla.org/network/dns-service;1",
                                   "nsIDNSService");

XPCOMUtils.defineLazyServiceGetter(this, "gNetworkService",
                                   "@mozilla.org/network/service;1",
                                   "nsINetworkService");

XPCOMUtils.defineLazyServiceGetter(this, "gMobileConnectionService",
                                   "@mozilla.org/mobileconnection/mobileconnectionservice;1",
                                   "nsIMobileConnectionService");

const TOPIC_INTERFACE_REGISTERED     = "network-interface-registered";
const TOPIC_INTERFACE_UNREGISTERED   = "network-interface-unregistered";
const TOPIC_ACTIVE_CHANGED           = "network-active-changed";
const TOPIC_MOZSETTINGS_CHANGED      = "mozsettings-changed";
const TOPIC_PREF_CHANGED             = "nsPref:changed";
const TOPIC_XPCOM_SHUTDOWN           = "xpcom-shutdown";
const TOPIC_CONNECTION_STATE_CHANGED = "network-connection-state-changed";
const PREF_MANAGE_OFFLINE_STATUS     = "network.gonk.manage-offline-status";

const POSSIBLE_USB_INTERFACE_NAME = "rndis0,usb0";
const DEFAULT_USB_INTERFACE_NAME  = "rndis0";
const DEFAULT_3G_INTERFACE_NAME   = "rmnet0";
const DEFAULT_WIFI_INTERFACE_NAME = "wlan0";

// The kernel's proc entry for network lists.
const KERNEL_NETWORK_ENTRY = "/sys/class/net";

const TETHERING_TYPE_WIFI = "WiFi";
const TETHERING_TYPE_USB  = "USB";

const WIFI_FIRMWARE_AP            = "AP";
const WIFI_FIRMWARE_STATION       = "STA";
const WIFI_SECURITY_TYPE_NONE     = "open";
const WIFI_SECURITY_TYPE_WPA_PSK  = "wpa-psk";
const WIFI_SECURITY_TYPE_WPA2_PSK = "wpa2-psk";
const WIFI_CTRL_INTERFACE         = "wl0.1";

const NETWORK_INTERFACE_UP   = "up";
const NETWORK_INTERFACE_DOWN = "down";

const TETHERING_STATE_ONGOING = "ongoing";
const TETHERING_STATE_IDLE    = "idle";
const TETHERING_STATE_ACTIVE  = "active";

// Settings DB path for USB tethering.
const SETTINGS_USB_ENABLED             = "tethering.usb.enabled";
const SETTINGS_USB_IP                  = "tethering.usb.ip";
const SETTINGS_USB_PREFIX              = "tethering.usb.prefix";
const SETTINGS_USB_DHCPSERVER_STARTIP  = "tethering.usb.dhcpserver.startip";
const SETTINGS_USB_DHCPSERVER_ENDIP    = "tethering.usb.dhcpserver.endip";
const SETTINGS_USB_DNS1                = "tethering.usb.dns1";
const SETTINGS_USB_DNS2                = "tethering.usb.dns2";

// Settings DB path for WIFI tethering.
const SETTINGS_WIFI_DHCPSERVER_STARTIP = "tethering.wifi.dhcpserver.startip";
const SETTINGS_WIFI_DHCPSERVER_ENDIP   = "tethering.wifi.dhcpserver.endip";

// Settings DB patch for dun required setting.
const SETTINGS_DUN_REQUIRED = "tethering.dun.required";

// Default value for USB tethering.
const DEFAULT_USB_IP                   = "192.168.0.1";
const DEFAULT_USB_PREFIX               = "24";
const DEFAULT_USB_DHCPSERVER_STARTIP   = "192.168.0.10";
const DEFAULT_USB_DHCPSERVER_ENDIP     = "192.168.0.30";

const DEFAULT_DNS1                     = "8.8.8.8";
const DEFAULT_DNS2                     = "8.8.4.4";

const DEFAULT_WIFI_DHCPSERVER_STARTIP  = "192.168.1.10";
const DEFAULT_WIFI_DHCPSERVER_ENDIP    = "192.168.1.30";

const IPV4_ADDRESS_ANY                 = "0.0.0.0";
const IPV6_ADDRESS_ANY                 = "::0";

const IPV4_MAX_PREFIX_LENGTH           = 32;
const IPV6_MAX_PREFIX_LENGTH           = 128;

const SETTINGS_DATA_DEFAULT_SERVICE_ID = "ril.data.defaultServiceId";
const MOBILE_DUN_CONNECT_TIMEOUT       = 30000;
const MOBILE_DUN_RETRY_INTERVAL        = 5000;
const MOBILE_DUN_MAX_RETRIES           = 5;

// Connection Type for Network Information API
const CONNECTION_TYPE_CELLULAR  = 0;
const CONNECTION_TYPE_BLUETOOTH = 1;
const CONNECTION_TYPE_ETHERNET  = 2;
const CONNECTION_TYPE_WIFI      = 3;
const CONNECTION_TYPE_OTHER     = 4;
const CONNECTION_TYPE_NONE      = 5;

let DEBUG = false;

// Read debug setting from pref.
try {
  let debugPref = Services.prefs.getBoolPref("network.debugging.enabled");
  DEBUG = DEBUG || debugPref;
} catch (e) {}

function defineLazyRegExp(obj, name, pattern) {
  obj.__defineGetter__(name, function() {
    delete obj[name];
    return obj[name] = new RegExp(pattern);
  });
}

function NetworkInterfaceLinks()
{
  this.resetLinks();
}
NetworkInterfaceLinks.prototype = {
  linkRoutes: null,
  gateways: null,
  interfaceName: null,
  extraRoutes: null,

  setLinks: function(linkRoutes, gateways, interfaceName) {
    this.linkRoutes = linkRoutes;
    this.gateways = gateways;
    this.interfaceName = interfaceName;
  },

  resetLinks: function() {
    this.linkRoutes = [];
    this.gateways = [];
    this.interfaceName = "";
    this.extraRoutes = [];
  },

  compareGateways: function(gateways) {
    if (this.gateways.length != gateways.length) {
      return false;
    }

    for (let i = 0; i < this.gateways.length; i++) {
      if (this.gateways[i] != gateways[i]) {
        return false;
      }
    }

    return true;
  }
};

/**
 * This component watches for network interfaces changing state and then
 * adjusts routes etc. accordingly.
 */
function NetworkManager() {
  this.networkInterfaces = {};
  this.networkInterfaceLinks = {};
  Services.obs.addObserver(this, TOPIC_XPCOM_SHUTDOWN, false);
  Services.obs.addObserver(this, TOPIC_MOZSETTINGS_CHANGED, false);

  try {
    this._manageOfflineStatus =
      Services.prefs.getBoolPref(PREF_MANAGE_OFFLINE_STATUS);
  } catch(ex) {
    // Ignore.
  }
  Services.prefs.addObserver(PREF_MANAGE_OFFLINE_STATUS, this, false);

  // Possible usb tethering interfaces for different gonk platform.
  this.possibleInterface = POSSIBLE_USB_INTERFACE_NAME.split(",");

  this._dataDefaultServiceId = 0;

  // Default values for internal and external interfaces.
  this._tetheringInterface = Object.create(null);
  this._tetheringInterface[TETHERING_TYPE_USB] = {
    externalInterface: DEFAULT_3G_INTERFACE_NAME,
    internalInterface: DEFAULT_USB_INTERFACE_NAME
  };
  this._tetheringInterface[TETHERING_TYPE_WIFI] = {
    externalInterface: DEFAULT_3G_INTERFACE_NAME,
    internalInterface: DEFAULT_WIFI_INTERFACE_NAME
  };

  this.initTetheringSettings();

  let settingsLock = gSettingsService.createLock();
  // Read the default service id for data call.
  settingsLock.get(SETTINGS_DATA_DEFAULT_SERVICE_ID, this);
  // Read usb tethering data from settings DB.
  settingsLock.get(SETTINGS_USB_IP, this);
  settingsLock.get(SETTINGS_USB_PREFIX, this);
  settingsLock.get(SETTINGS_USB_DHCPSERVER_STARTIP, this);
  settingsLock.get(SETTINGS_USB_DHCPSERVER_ENDIP, this);
  settingsLock.get(SETTINGS_USB_DNS1, this);
  settingsLock.get(SETTINGS_USB_DNS2, this);
  settingsLock.get(SETTINGS_USB_ENABLED, this);

  // Read wifi tethering data from settings DB.
  settingsLock.get(SETTINGS_WIFI_DHCPSERVER_STARTIP, this);
  settingsLock.get(SETTINGS_WIFI_DHCPSERVER_ENDIP, this);

  this._usbTetheringSettingsToRead = [SETTINGS_USB_IP,
                                      SETTINGS_USB_PREFIX,
                                      SETTINGS_USB_DHCPSERVER_STARTIP,
                                      SETTINGS_USB_DHCPSERVER_ENDIP,
                                      SETTINGS_USB_DNS1,
                                      SETTINGS_USB_DNS2,
                                      SETTINGS_USB_ENABLED,
                                      SETTINGS_WIFI_DHCPSERVER_STARTIP,
                                      SETTINGS_WIFI_DHCPSERVER_ENDIP];

  this.wantConnectionEvent = null;
  this.setAndConfigureActive();

  ppmm.addMessageListener('NetworkInterfaceList:ListInterface', this);

  // Used in resolveHostname().
  defineLazyRegExp(this, "REGEXP_IPV4", "^\\d{1,3}(?:\\.\\d{1,3}){3}$");
  defineLazyRegExp(this, "REGEXP_IPV6", "^[\\da-fA-F]{4}(?::[\\da-fA-F]{4}){7}$");
}
NetworkManager.prototype = {
  classID:   NETWORKMANAGER_CID,
  classInfo: XPCOMUtils.generateCI({classID: NETWORKMANAGER_CID,
                                    contractID: NETWORKMANAGER_CONTRACTID,
                                    classDescription: "Network Manager",
                                    interfaces: [Ci.nsINetworkManager]}),
  QueryInterface: XPCOMUtils.generateQI([Ci.nsINetworkManager,
                                         Ci.nsISupportsWeakReference,
                                         Ci.nsIObserver,
                                         Ci.nsISettingsServiceCallback]),

  // nsIObserver

  observe: function(subject, topic, data) {
    switch (topic) {
      case TOPIC_MOZSETTINGS_CHANGED:
        if ("wrappedJSObject" in subject) {
          subject = subject.wrappedJSObject;
        }
        this.handle(subject.key, subject.value);
        break;
      case TOPIC_PREF_CHANGED:
        this._manageOfflineStatus =
          Services.prefs.getBoolPref(PREF_MANAGE_OFFLINE_STATUS);
        debug(PREF_MANAGE_OFFLINE_STATUS + " has changed to " + this._manageOfflineStatus);
        break;
      case TOPIC_XPCOM_SHUTDOWN:
        Services.obs.removeObserver(this, TOPIC_XPCOM_SHUTDOWN);
        Services.obs.removeObserver(this, TOPIC_MOZSETTINGS_CHANGED);

        this.dunConnectTimer.cancel();
        this.dunRetryTimer.cancel();
        break;
    }
  },

  receiveMessage: function(aMsg) {
    switch (aMsg.name) {
      case "NetworkInterfaceList:ListInterface": {
        let excludeMms = aMsg.json.excludeMms;
        let excludeSupl = aMsg.json.excludeSupl;
        let excludeIms = aMsg.json.excludeIms;
        let excludeDun = aMsg.json.excludeDun;
        let interfaces = [];

        for each (let i in this.networkInterfaces) {
          if ((i.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_MMS && excludeMms) ||
              (i.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_SUPL && excludeSupl) ||
              (i.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_IMS && excludeIms) ||
              (i.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN && excludeDun)) {
            continue;
          }

          let ips = {};
          let prefixLengths = {};
          i.getAddresses(ips, prefixLengths);

          interfaces.push({
            state: i.state,
            type: i.type,
            name: i.name,
            ips: ips.value,
            prefixLengths: prefixLengths.value,
            gateways: i.getGateways(),
            dnses: i.getDnses(),
            httpProxyHost: i.httpProxyHost,
            httpProxyPort: i.httpProxyPort
          });
        }
        return interfaces;
      }
    }
  },

  getNetworkId: function(network) {
    let id = "device";
    try {
      if (network instanceof Ci.nsIRilNetworkInterface) {
        let rilNetwork = network.QueryInterface(Ci.nsIRilNetworkInterface);
        id = "ril" + rilNetwork.serviceId;
      }
    } catch (e) {}

    return id + "-" + network.type;
  },

  // nsINetworkManager

  registerNetworkInterface: function(network) {
    if (!(network instanceof Ci.nsINetworkInterface)) {
      throw Components.Exception("Argument must be nsINetworkInterface.",
                                 Cr.NS_ERROR_INVALID_ARG);
    }
    let networkId = this.getNetworkId(network);
    if (networkId in this.networkInterfaces) {
      throw Components.Exception("Network with that type already registered!",
                                 Cr.NS_ERROR_INVALID_ARG);
    }
    this.networkInterfaces[networkId] = network;
    this.networkInterfaceLinks[networkId] = new NetworkInterfaceLinks();

    if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN) {
      debug("Force setting " + SETTINGS_DUN_REQUIRED + " to true.");
      this.tetheringSettings[SETTINGS_DUN_REQUIRED] = true;
    }

    Services.obs.notifyObservers(network, TOPIC_INTERFACE_REGISTERED, null);
    debug("Network '" + networkId + "' registered.");
  },

  _addSubnetRoutes: function(network) {
    let ips = {};
    let prefixLengths = {};
    let length = network.getAddresses(ips, prefixLengths);
    for (let i = 0; i < length; i++) {
      debug('Adding subnet routes: ' + ips.value[i] + '/' + prefixLengths.value[i]);
      gNetworkService.modifyRoute(Ci.nsINetworkService.MODIFY_ROUTE_ADD,
                                  network.name, ips.value[i], prefixLengths.value[i])
        .catch((aError) => {
          debug("_addSubnetRoutes error: " + aError);
        });
    }
  },

  updateNetworkInterface: function(network) {
    if (!(network instanceof Ci.nsINetworkInterface)) {
      throw Components.Exception("Argument must be nsINetworkInterface.",
                                 Cr.NS_ERROR_INVALID_ARG);
    }
    let networkId = this.getNetworkId(network);
    if (!(networkId in this.networkInterfaces)) {
      throw Components.Exception("No network with that type registered.",
                                 Cr.NS_ERROR_INVALID_ARG);
    }
    debug("Network " + network.type + "/" + network.name +
          " changed state to " + network.state);

    // Note that since Lollipop we need to allocate and initialize
    // something through netd, so we add createNetwork/destroyNetwork
    // to deal with that explicitly.

    switch (network.state) {
      case Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED:
        gNetworkService.createNetwork(network.name, () => {
          // Add host route for data calls
          if (this.isNetworkTypeMobile(network.type)) {
            let currentInterfaceLinks = this.networkInterfaceLinks[networkId];
            let newLinkRoutes = network.getDnses().concat(network.httpProxyHost);
            // If gateways have changed, remove all old routes first.
            this._handleGateways(networkId, network.getGateways())
              .then(() => this._updateRoutes(currentInterfaceLinks.linkRoutes,
                                             newLinkRoutes,
                                             network.getGateways(), network.name))
              .then(() => currentInterfaceLinks.setLinks(newLinkRoutes,
                                                         network.getGateways(),
                                                         network.name));
          }

          // Remove pre-created default route and let setAndConfigureActive()
          // to set default route only on preferred network
          gNetworkService.removeDefaultRoute(network);

          // Dun type is a special case where we add the default route to a
          // secondary table.
          if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN) {
            this.setSecondaryDefaultRoute(network);
          }

          this._addSubnetRoutes(network);
          this.setAndConfigureActive();

          // Update data connection when Wifi connected/disconnected
          if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_WIFI && this.mRil) {
            for (let i = 0; i < this.mRil.numRadioInterfaces; i++) {
              this.mRil.getRadioInterface(i).updateRILNetworkInterface();
            }
          }

          this.onConnectionChanged(network);

          // Probing the public network accessibility after routing table is ready
          CaptivePortalDetectionHelper
            .notify(CaptivePortalDetectionHelper.EVENT_CONNECT, this.active);

          // Notify outer modules like MmsService to start the transaction after
          // the configuration of the network interface is done.
          Services.obs.notifyObservers(network, TOPIC_CONNECTION_STATE_CHANGED,
                                       this.convertConnectionType(network));
        });

        break;
      case Ci.nsINetworkInterface.NETWORK_STATE_DISCONNECTED:
        // Remove host route for data calls
        if (this.isNetworkTypeMobile(network.type)) {
          this._cleanupAllHostRoutes(networkId);
        }
        // Remove secondary default route for dun.
        if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN) {
          this.removeSecondaryDefaultRoute(network);
        }
        // Remove routing table in /proc/net/route
        if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_WIFI) {
          gNetworkService.resetRoutingTable(network);
        } else if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE) {
          gNetworkService.removeDefaultRoute(network);
        }
        // Clear http proxy on active network.
        if (this.active && network.type == this.active.type) {
          gNetworkService.clearNetworkProxy();
        }

        // Abort ongoing captive portal detection on the wifi interface
        CaptivePortalDetectionHelper
          .notify(CaptivePortalDetectionHelper.EVENT_DISCONNECT, network);
        this.setAndConfigureActive();

        // Update data connection when Wifi connected/disconnected
        if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_WIFI && this.mRil) {
          for (let i = 0; i < this.mRil.numRadioInterfaces; i++) {
            this.mRil.getRadioInterface(i).updateRILNetworkInterface();
          }
        }

        gNetworkService.destroyNetwork(network.name, () => {
          // Notify outer modules like MmsService to start the transaction after
          // the configuration of the network interface is done.
          Services.obs.notifyObservers(network, TOPIC_CONNECTION_STATE_CHANGED,
                                       this.convertConnectionType(network));
        });
        break;
    }
  },

  unregisterNetworkInterface: function(network) {
    if (!(network instanceof Ci.nsINetworkInterface)) {
      throw Components.Exception("Argument must be nsINetworkInterface.",
                                 Cr.NS_ERROR_INVALID_ARG);
    }
    let networkId = this.getNetworkId(network);
    if (!(networkId in this.networkInterfaces)) {
      throw Components.Exception("No network with that type registered.",
                                 Cr.NS_ERROR_INVALID_ARG);
    }

    // This is for in case a network gets unregistered without being
    // DISCONNECTED.
    if (this.isNetworkTypeMobile(network.type)) {
      this._cleanupAllHostRoutes(networkId);
    }

    delete this.networkInterfaces[networkId];

    if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN) {
      this.tetheringSettings[SETTINGS_DUN_REQUIRED] =
        libcutils.property_get("ro.tethering.dun_required") === "1";
    }

    Services.obs.notifyObservers(network, TOPIC_INTERFACE_UNREGISTERED, null);
    debug("Network '" + networkId + "' unregistered.");
  },

  _manageOfflineStatus: true,

  networkInterfaces: null,

  networkInterfaceLinks: null,

  _dataDefaultServiceId: null,

  _preferredNetworkType: DEFAULT_PREFERRED_NETWORK_TYPE,
  get preferredNetworkType() {
    return this._preferredNetworkType;
  },
  set preferredNetworkType(val) {
    if ([Ci.nsINetworkInterface.NETWORK_TYPE_WIFI,
         Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE].indexOf(val) == -1) {
      throw "Invalid network type";
    }
    this._preferredNetworkType = val;
  },

  active: null,
  _overriddenActive: null,

  overrideActive: function(network) {
    if ([Ci.nsINetworkInterface.NETWORK_TYPE_WIFI,
         Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE].indexOf(val) == -1) {
      throw "Invalid network type";
    }

    this._overriddenActive = network;
    this.setAndConfigureActive();
  },

  _updateRoutes: function(oldLinks, newLinks, gateways, interfaceName) {
    // Returns items that are in base but not in target.
    function getDifference(base, target) {
      return base.filter(function(i) { return target.indexOf(i) < 0; });
    }

    let addedLinks = getDifference(newLinks, oldLinks);
    let removedLinks = getDifference(oldLinks, newLinks);

    if (addedLinks.length === 0 && removedLinks.length === 0) {
      return Promise.resolve();
    }

    return this._setHostRoutes(false, removedLinks, interfaceName, gateways)
      .then(this._setHostRoutes(true, addedLinks, interfaceName, gateways));
  },

  _setHostRoutes: function(doAdd, ipAddresses, networkName, gateways) {
    let getMaxPrefixLength = (aIp) => {
      return aIp.match(this.REGEXP_IPV4) ? IPV4_MAX_PREFIX_LENGTH : IPV6_MAX_PREFIX_LENGTH;
    }

    let promises = [];

    ipAddresses.forEach((aIpAddress) => {
      let gateway = this.selectGateway(gateways, aIpAddress);
      if (gateway) {
        promises.push((doAdd)
          ? gNetworkService.modifyRoute(Ci.nsINetworkService.MODIFY_ROUTE_ADD,
                                        networkName, aIpAddress,
                                        getMaxPrefixLength(aIpAddress), gateway)
          : gNetworkService.modifyRoute(Ci.nsINetworkService.MODIFY_ROUTE_REMOVE,
                                        networkName, aIpAddress,
                                        getMaxPrefixLength(aIpAddress), gateway));
      }
    });

    return Promise.all(promises);
  },

  isValidatedNetwork: function(network) {
    let isValid = false;
    try {
      isValid = (this.getNetworkId(network) in this.networkInterfaces);
    } catch (e) {
      debug("Invalid network interface: " + e);
    }

    return isValid;
  },

  addHostRoute: function(network, host) {
    if (!this.isValidatedNetwork(network)) {
      return Promise.reject("Invalid network interface.");
    }

    return this.resolveHostname(network, host)
      .then((ipAddresses) => {
        let promises = [];
        let networkId = this.getNetworkId(network);

        ipAddresses.forEach((aIpAddress) => {
          let promise =
            this._setHostRoutes(true, [aIpAddress], network.name, network.getGateways())
              .then(() => this.networkInterfaceLinks[networkId].extraRoutes.push(aIpAddress));

          promises.push(promise);
        });

        return Promise.all(promises);
      });
  },

  removeHostRoute: function(network, host) {
    if (!this.isValidatedNetwork(network)) {
      return Promise.reject("Invalid network interface.");
    }

    return this.resolveHostname(network, host)
      .then((ipAddresses) => {
        let promises = [];
        let networkId = this.getNetworkId(network);

        ipAddresses.forEach((aIpAddress) => {
          let found = this.networkInterfaceLinks[networkId].extraRoutes.indexOf(aIpAddress);
          if (found < 0) {
            return; // continue
          }

          let promise =
            this._setHostRoutes(false, [aIpAddress], network.name, network.getGateways())
              .then(() => {
                this.networkInterfaceLinks[networkId].extraRoutes.splice(found, 1);
              }, () => {
                // We should remove it even if the operation failed.
                this.networkInterfaceLinks[networkId].extraRoutes.splice(found, 1);
              });
          promises.push(promise);
        });

        return Promise.all(promises);
      });
  },

  isNetworkTypeSecondaryMobile: function(type) {
    return (type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_MMS ||
            type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_SUPL ||
            type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_IMS ||
            type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN);
  },

  isNetworkTypeMobile: function(type) {
    return (type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE ||
            this.isNetworkTypeSecondaryMobile(type));
  },

  _handleGateways: function(networkId, gateways) {
    let currentNetworkLinks = this.networkInterfaceLinks[networkId];
    if (currentNetworkLinks.gateways.length == 0 ||
        currentNetworkLinks.compareGateways(gateways)) {
      return Promise.resolve();
    }

    let currentExtraRoutes = currentNetworkLinks.extraRoutes;
    return this._cleanupAllHostRoutes(networkId)
      .then(() => {
        // If gateways have changed, re-add extra host routes with new gateways.
        if (currentExtraRoutes.length > 0) {
          this._setHostRoutes(true,
                              currentExtraRoutes,
                              currentNetworkLinks.interfaceName,
                              gateways)
          .then(() => {
            currentNetworkLinks.extraRoutes = currentExtraRoutes;
          });
        }
      });
  },

  _cleanupAllHostRoutes: function(networkId) {
    let currentNetworkLinks = this.networkInterfaceLinks[networkId];
    let hostRoutes = currentNetworkLinks.linkRoutes.concat(
      currentNetworkLinks.extraRoutes);

    if (hostRoutes.length === 0) {
      return Promise.resolve();
    }

    return this._setHostRoutes(false,
                               hostRoutes,
                               currentNetworkLinks.interfaceName,
                               currentNetworkLinks.gateways)
      .catch((aError) => {
        debug("Error (" + aError + ") on _cleanupAllHostRoutes, keep proceeding.");
      })
      .then(() => currentNetworkLinks.resetLinks());
  },

  selectGateway: function(gateways, host) {
    for (let i = 0; i < gateways.length; i++) {
      let gateway = gateways[i];
      if (gateway.match(this.REGEXP_IPV4) && host.match(this.REGEXP_IPV4) ||
          gateway.indexOf(":") != -1 && host.indexOf(":") != -1) {
        return gateway;
      }
    }
    return null;
  },

  setSecondaryDefaultRoute: function(network) {
    let gateways = network.getGateways();
    for (let i = 0; i < gateways.length; i++) {
      let isIPv6 = (gateways[i].indexOf(":") != -1) ? true : false;
      // First, we need to add a host route to the gateway in the secondary
      // routing table to make the gateway reachable. Host route takes the max
      // prefix and gateway address 'any'.
      let route = {
        ip: gateways[i],
        prefix: isIPv6 ? IPV6_MAX_PREFIX_LENGTH : IPV4_MAX_PREFIX_LENGTH,
        gateway: isIPv6 ? IPV6_ADDRESS_ANY : IPV4_ADDRESS_ANY
      };
      gNetworkService.addSecondaryRoute(network.name, route);
      // Now we can add the default route through gateway. Default route takes the
      // min prefix and destination ip 'any'.
      route.ip = isIPv6 ? IPV6_ADDRESS_ANY : IPV4_ADDRESS_ANY;
      route.prefix = 0;
      route.gateway = gateways[i];
      gNetworkService.addSecondaryRoute(network.name, route);
    }
  },

  removeSecondaryDefaultRoute: function(network) {
    let gateways = network.getGateways();
    for (let i = 0; i < gateways.length; i++) {
      let isIPv6 = (gateways[i].indexOf(":") != -1) ? true : false;
      // Remove both default route and host route.
      let route = {
        ip: isIPv6 ? IPV6_ADDRESS_ANY : IPV4_ADDRESS_ANY,
        prefix: 0,
        gateway: gateways[i]
      };
      gNetworkService.removeSecondaryRoute(network.name, route);

      route.ip = gateways[i];
      route.prefix = isIPv6 ? IPV6_MAX_PREFIX_LENGTH : IPV4_MAX_PREFIX_LENGTH;
      route.gateway = isIPv6 ? IPV6_ADDRESS_ANY : IPV4_ADDRESS_ANY;
      gNetworkService.removeSecondaryRoute(network.name, route);
    }
  },

  /**
   * Determine the active interface and configure it.
   */
  setAndConfigureActive: function() {
    debug("Evaluating whether active network needs to be changed.");
    let oldActive = this.active;

    if (this._overriddenActive) {
      debug("We have an override for the active network: " +
            this._overriddenActive.name);
      // The override was just set, so reconfigure the network.
      if (this.active != this._overriddenActive) {
        this.active = this._overriddenActive;
        this._setDefaultRouteAndDNS(this.active, oldActive);
        Services.obs.notifyObservers(this.active, TOPIC_ACTIVE_CHANGED, null);
      }
      return;
    }

    // The active network is already our preferred type.
    if (this.active &&
        this.active.state == Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED &&
        this.active.type == this._preferredNetworkType) {
      debug("Active network is already our preferred type.");
      this._setDefaultRouteAndDNS(this.active, oldActive);
      return;
    }

    // Find a suitable network interface to activate.
    this.active = null;

    let defaultDataNetwork;
    for each (let network in this.networkInterfaces) {
      if (network.state != Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED) {
        continue;
      }

      if (network.type == Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE) {
        defaultDataNetwork = network;
      }

      this.active = network;
      if (network.type == this.preferredNetworkType) {
        debug("Found our preferred type of network: " + network.name);
        break;
      }
    }

    if (this.active) {
      // Give higher priority to default data APN than secondary APN.
      // If default data APN is not connected, we still set default route
      // and DNS on secondary APN.
      if (defaultDataNetwork &&
          this.isNetworkTypeSecondaryMobile(this.active.type) &&
          this.active.type != this.preferredNetworkType) {
        this.active = defaultDataNetwork;
      }
      // Don't set default route on secondary APN
      if (this.isNetworkTypeSecondaryMobile(this.active.type)) {
        gNetworkService.setDNS(this.active, function() {});
      } else {
        this._setDefaultRouteAndDNS(this.active, oldActive);
      }
    }

    if (this.active != oldActive) {
      Services.obs.notifyObservers(this.active, TOPIC_ACTIVE_CHANGED, null);
    }

    if (this._manageOfflineStatus) {
      Services.io.offline = !this.active;
    }
  },

  resolveHostname: function(network, hostname) {
    // Sanity check for null, undefined and empty string... etc.
    if (!hostname) {
      return Promise.reject(new Error("hostname is empty: " + hostname));
    }

    if (hostname.match(this.REGEXP_IPV4) ||
        hostname.match(this.REGEXP_IPV6)) {
      return Promise.resolve([hostname]);
    }

    let deferred = Promise.defer();
    let onLookupComplete = (aRequest, aRecord, aStatus) => {
      if (!Components.isSuccessCode(aStatus)) {
        deferred.reject(new Error(
          "Failed to resolve '" + hostname + "', with status: " + aStatus));
        return;
      }

      let retval = [];
      while (aRecord.hasMore()) {
        retval.push(aRecord.getNextAddrAsString());
      }

      if (!retval.length) {
        deferred.reject(new Error("No valid address after DNS lookup!"));
        return;
      }

      if (DEBUG) debug("hostname is resolved: " + hostname);
      if (DEBUG) debug("Addresses: " + JSON.stringify(retval));

      deferred.resolve(retval);
    };

    // Bug 1058282 - Explicitly request ipv4 to get around 8.8.8.8 probe at
    // http://androidxref.com/4.3_r2.1/xref/bionic/libc/netbsd/net/getaddrinfo.c#1923
    //
    // Whenever MMS connection is the only network interface, there is no
    // default route so that any ip probe will fail.
    let flags = 0;
    if (network.type === Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_MMS) {
      flags |= Ci.nsIDNSService.RESOLVE_DISABLE_IPV6;
    }

    // TODO: Bug 992772 - Resolve the hostname with specified networkInterface.
    gDNSService.asyncResolve(hostname, flags, onLookupComplete, Services.tm.mainThread);

    return deferred.promise;
  },

  convertConnectionType: function(network) {
    // If there is internal interface change (e.g., MOBILE_MMS, MOBILE_SUPL),
    // the function will return null so that it won't trigger type change event
    // in NetworkInformation API.
    if (network.type != Ci.nsINetworkInterface.NETWORK_TYPE_WIFI &&
        network.type != Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE) {
      return null;
    }

    if (network.state == Ci.nsINetworkInterface.NETWORK_STATE_DISCONNECTED) {
      return CONNECTION_TYPE_NONE;
    }

    switch (network.type) {
      case Ci.nsINetworkInterface.NETWORK_TYPE_WIFI:
        return CONNECTION_TYPE_WIFI;
      case Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE:
        return CONNECTION_TYPE_CELLULAR;
    }
  },

  // nsISettingsServiceCallback

  tetheringSettings: {},

  initTetheringSettings: function() {
    this.tetheringSettings[SETTINGS_USB_ENABLED] = false;
    this.tetheringSettings[SETTINGS_USB_IP] = DEFAULT_USB_IP;
    this.tetheringSettings[SETTINGS_USB_PREFIX] = DEFAULT_USB_PREFIX;
    this.tetheringSettings[SETTINGS_USB_DHCPSERVER_STARTIP] = DEFAULT_USB_DHCPSERVER_STARTIP;
    this.tetheringSettings[SETTINGS_USB_DHCPSERVER_ENDIP] = DEFAULT_USB_DHCPSERVER_ENDIP;
    this.tetheringSettings[SETTINGS_USB_DNS1] = DEFAULT_DNS1;
    this.tetheringSettings[SETTINGS_USB_DNS2] = DEFAULT_DNS2;

    this.tetheringSettings[SETTINGS_WIFI_DHCPSERVER_STARTIP] = DEFAULT_WIFI_DHCPSERVER_STARTIP;
    this.tetheringSettings[SETTINGS_WIFI_DHCPSERVER_ENDIP]   = DEFAULT_WIFI_DHCPSERVER_ENDIP;

    this.tetheringSettings[SETTINGS_DUN_REQUIRED] =
      libcutils.property_get("ro.tethering.dun_required") === "1";
  },

  _usbTetheringRequestCount: 0,

  handle: function(aName, aResult) {
    switch(aName) {
      case SETTINGS_DATA_DEFAULT_SERVICE_ID:
        this._dataDefaultServiceId = aResult || 0;
        debug("'_dataDefaultServiceId' is now " + this._dataDefaultServiceId);
        break;
      case SETTINGS_USB_ENABLED:
        this._oldUsbTetheringEnabledState = this.tetheringSettings[SETTINGS_USB_ENABLED];
      case SETTINGS_USB_IP:
      case SETTINGS_USB_PREFIX:
      case SETTINGS_USB_DHCPSERVER_STARTIP:
      case SETTINGS_USB_DHCPSERVER_ENDIP:
      case SETTINGS_USB_DNS1:
      case SETTINGS_USB_DNS2:
      case SETTINGS_WIFI_DHCPSERVER_STARTIP:
      case SETTINGS_WIFI_DHCPSERVER_ENDIP:
        if (aResult !== null) {
          this.tetheringSettings[aName] = aResult;
        }
        debug("'" + aName + "'" + " is now " + this.tetheringSettings[aName]);
        let index = this._usbTetheringSettingsToRead.indexOf(aName);

        if (index != -1) {
          this._usbTetheringSettingsToRead.splice(index, 1);
        }

        if (this._usbTetheringSettingsToRead.length) {
          debug("We haven't read completely the usb Tethering data from settings db.");
          break;
        }

        if (this._oldUsbTetheringEnabledState === this.tetheringSettings[SETTINGS_USB_ENABLED]) {
          debug("No changes for SETTINGS_USB_ENABLED flag. Nothing to do.");
          this.handlePendingWifiTetheringRequest();
          break;
        }

        this._usbTetheringRequestCount++;
        if (this._usbTetheringRequestCount === 1) {
          if (this._wifiTetheringRequestOngoing) {
            debug('USB tethering request is blocked by ongoing wifi tethering request.');
          } else {
            this.handleLastUsbTetheringRequest();
          }
        }
        break;
    };
  },

  handleError: function(aErrorMessage) {
    debug("There was an error while reading Tethering settings.");
    this.tetheringSettings = {};
    this.tetheringSettings[SETTINGS_USB_ENABLED] = false;
  },

  getNetworkInterface: function(type, serviceId) {
    for each (let network in this.networkInterfaces) {
      if (network.type == type) {
        try {
          if (network instanceof Ci.nsIRilNetworkInterface) {
            let rilNetwork = network.QueryInterface(Ci.nsIRilNetworkInterface);
            if (rilNetwork.serviceId != serviceId) {
              continue;
            }
          }
        } catch (e) {}
        return network;
      }
    }
    return null;
  },

  _usbTetheringAction: TETHERING_STATE_IDLE,

  _usbTetheringSettingsToRead: [],

  _oldUsbTetheringEnabledState: null,

  // External and internal interface name.
  _tetheringInterface: null,

  handleLastUsbTetheringRequest: function() {
    debug('handleLastUsbTetheringRequest... ' + this._usbTetheringRequestCount);

    if (this._usbTetheringRequestCount === 0) {
      if (this.wantConnectionEvent) {
        if (this.tetheringSettings[SETTINGS_USB_ENABLED]) {
          this.wantConnectionEvent.call(this);
        }
        this.wantConnectionEvent = null;
      }
      this.handlePendingWifiTetheringRequest();
      return;
    }

    // Cancel the accumlated count to 1 since we only care about the
    // last state.
    this._usbTetheringRequestCount = 1;
    this.handleUSBTetheringToggle(this.tetheringSettings[SETTINGS_USB_ENABLED]);
    this.wantConnectionEvent = null;
  },

  handlePendingWifiTetheringRequest: function() {
    if (this._pendingWifiTetheringRequestArgs) {
      this.setWifiTethering.apply(this, this._pendingWifiTetheringRequestArgs);
      this._pendingWifiTetheringRequestArgs = null;
    }
  },

  dunConnectTimer: Cc["@mozilla.org/timer;1"].createInstance(Ci.nsITimer),
  /**
   * Callback when dun connection fails to connect within timeout.
   */
  onDunConnectTimerTimeout: function() {
    while (this._pendingTetheringRequests.length > 0) {
      debug("onDunConnectTimerTimeout: callback without network info.");
      let callback = this._pendingTetheringRequests.shift();
      if (typeof callback === 'function') {
        callback();
      }
    }
  },

  dunRetryTimes: 0,
  dunRetryTimer: Cc["@mozilla.org/timer;1"].createInstance(Ci.nsITimer),
  setupDunConnection: function() {
    this.dunRetryTimer.cancel();
    let connection =
      gMobileConnectionService.getItemByServiceId(this._dataDefaultServiceId);
    let data = connection && connection.data;
    if (data && data.state === "registered") {
      let ril = this.mRil.getRadioInterface(this._dataDefaultServiceId);

      this.dunRetryTimes = 0;
      ril.setupDataCallByType(Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN);
      this.dunConnectTimer.cancel();
      this.dunConnectTimer.
        initWithCallback(this.onDunConnectTimerTimeout.bind(this),
                         MOBILE_DUN_CONNECT_TIMEOUT, Ci.nsITimer.TYPE_ONE_SHOT);
      return;
    }

    if (this.dunRetryTimes++ >= this.MOBILE_DUN_MAX_RETRIES) {
      debug("setupDunConnection: max retries reached.");
      this.dunRetryTimes = 0;
      // same as dun connect timeout.
      this.onDunConnectTimerTimeout();
      return;
    }

    debug("Data not ready, retry dun after " + MOBILE_DUN_RETRY_INTERVAL + " ms.");
    this.dunRetryTimer.
      initWithCallback(this.setupDunConnection.bind(this),
                       MOBILE_DUN_RETRY_INTERVAL, Ci.nsITimer.TYPE_ONE_SHOT);
  },

  _pendingTetheringRequests: [],
  _dunActiveUsers: 0,
  handleDunConnection: function(enable, callback) {
    debug("handleDunConnection: " + enable);
    let dun = this.getNetworkInterface(
      Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN, this._dataDefaultServiceId);

    if (!enable) {
      this._dunActiveUsers--;
      if (this._dunActiveUsers > 0) {
        debug("Dun still needed by others, do not disconnect.")
        return;
      }

      this.dunRetryTimes = 0;
      this.dunRetryTimer.cancel();
      this.dunConnectTimer.cancel();
      this._pendingTetheringRequests = [];

      if (dun && (dun.state == Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED)) {
        this.mRil.getRadioInterface(this._dataDefaultServiceId)
          .deactivateDataCallByType(Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN);
      }
      return;
    }

    this._dunActiveUsers++;
    if (!dun || (dun.state != Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED)) {
      debug("DUN data call inactive, setup dun data call!")
      this._pendingTetheringRequests.push(callback);
      this.dunRetryTimes = 0;
      this.setupDunConnection();

      return;
    }
    this._tetheringInterface[TETHERING_TYPE_USB].externalInterface = dun.name;
    callback(dun);
  },

  handleUSBTetheringToggle: function(enable) {
    debug("handleUSBTetheringToggle: " + enable);
    if (enable &&
        (this._usbTetheringAction === TETHERING_STATE_ONGOING ||
         this._usbTetheringAction === TETHERING_STATE_ACTIVE)) {
      debug("Usb tethering already connecting/connected.");
      this._usbTetheringRequestCount = 0;
      this.handlePendingWifiTetheringRequest();
      return;
    }

    if (!enable &&
        this._usbTetheringAction === TETHERING_STATE_IDLE) {
      debug("Usb tethering already disconnected.");
      this._usbTetheringRequestCount = 0;
      this.handlePendingWifiTetheringRequest();
      return;
    }

    if (!enable) {
      this.tetheringSettings[SETTINGS_USB_ENABLED] = false;
      gNetworkService.enableUsbRndis(false, this.enableUsbRndisResult.bind(this));
      return;
    }

    this.tetheringSettings[SETTINGS_USB_ENABLED] = true;
    this._usbTetheringAction = TETHERING_STATE_ONGOING;

    if (this.tetheringSettings[SETTINGS_DUN_REQUIRED]) {
      this.handleDunConnection(true, function(network) {
        if (!network){
          this.usbTetheringResultReport("Dun connection failed");
          return;
        }
        this._tetheringInterface[TETHERING_TYPE_USB].externalInterface = network.name;
        gNetworkService.enableUsbRndis(true, this.enableUsbRndisResult.bind(this));
      }.bind(this));
      return;
    }

    if (this.active) {
      this._tetheringInterface[TETHERING_TYPE_USB].externalInterface = this.active.name;
    } else {
      let mobile = this.getNetworkInterface(
        Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE, this._dataDefaultServiceId);
      if (mobile && mobile.name) {
        this._tetheringInterface[TETHERING_TYPE_USB].externalInterface = mobile.name;
      }
    }
    gNetworkService.enableUsbRndis(true, this.enableUsbRndisResult.bind(this));
  },

  getUSBTetheringParameters: function(enable, tetheringinterface) {
    let interfaceIp;
    let prefix;
    let wifiDhcpStartIp;
    let wifiDhcpEndIp;
    let usbDhcpStartIp;
    let usbDhcpEndIp;
    let dns1;
    let dns2;
    let internalInterface = tetheringinterface.internalInterface;
    let externalInterface = tetheringinterface.externalInterface;

    interfaceIp = this.tetheringSettings[SETTINGS_USB_IP];
    prefix = this.tetheringSettings[SETTINGS_USB_PREFIX];
    wifiDhcpStartIp = this.tetheringSettings[SETTINGS_WIFI_DHCPSERVER_STARTIP];
    wifiDhcpEndIp = this.tetheringSettings[SETTINGS_WIFI_DHCPSERVER_ENDIP];
    usbDhcpStartIp = this.tetheringSettings[SETTINGS_USB_DHCPSERVER_STARTIP];
    usbDhcpEndIp = this.tetheringSettings[SETTINGS_USB_DHCPSERVER_ENDIP];
    dns1 = this.tetheringSettings[SETTINGS_USB_DNS1];
    dns2 = this.tetheringSettings[SETTINGS_USB_DNS2];

    // Using the default values here until application support these settings.
    if (interfaceIp == "" || prefix == "" ||
        wifiDhcpStartIp == "" || wifiDhcpEndIp == "" ||
        usbDhcpStartIp == "" || usbDhcpEndIp == "") {
      debug("Invalid subnet information.");
      return null;
    }

    return {
      ifname: internalInterface,
      ip: interfaceIp,
      prefix: prefix,
      wifiStartIp: wifiDhcpStartIp,
      wifiEndIp: wifiDhcpEndIp,
      usbStartIp: usbDhcpStartIp,
      usbEndIp: usbDhcpEndIp,
      dns1: dns1,
      dns2: dns2,
      internalIfname: internalInterface,
      externalIfname: externalInterface,
      enable: enable,
      link: enable ? NETWORK_INTERFACE_UP : NETWORK_INTERFACE_DOWN
    };
  },

  notifyError: function(resetSettings, callback, msg) {
    if (resetSettings) {
      let settingsLock = gSettingsService.createLock();
      // Disable wifi tethering with a useful error message for the user.
      settingsLock.set("tethering.wifi.enabled", false, null, msg);
    }

    debug("setWifiTethering: " + (msg ? msg : "success"));

    if (callback) {
      // Callback asynchronously to avoid netsted toggling.
      Services.tm.currentThread.dispatch(() => {
        callback.wifiTetheringEnabledChange(msg);
      }, Ci.nsIThread.DISPATCH_NORMAL);
    }
  },

  _wifiTetheringRequestOngoing: false,
  enableWifiTethering: function(enable, config, callback) {
    // Fill in config's required fields.
    config.ifname         = this._tetheringInterface[TETHERING_TYPE_WIFI].internalInterface;
    config.internalIfname = this._tetheringInterface[TETHERING_TYPE_WIFI].internalInterface;
    config.externalIfname = this._tetheringInterface[TETHERING_TYPE_WIFI].externalInterface;

    this._wifiTetheringRequestOngoing = true;
    gNetworkService.setWifiTethering(enable, config, (function(error) {
      // Disconnect dun on error or when wifi tethering is disabled.
      if (this.tetheringSettings[SETTINGS_DUN_REQUIRED] &&
          (!enable || error)) {
        this.handleDunConnection(false);
      }

      let resetSettings = error;
      debug('gNetworkService.setWifiTethering finished');
      this.notifyError(resetSettings, callback, error);
      this._wifiTetheringRequestOngoing = false;
      if (this._usbTetheringRequestCount > 0) {
        debug('Perform pending USB tethering requests.');
        this.handleLastUsbTetheringRequest();
      }
    }).bind(this));
  },

  _pendingWifiTetheringRequestArgs: null,
  // Enable/disable WiFi tethering by sending commands to netd.
  setWifiTethering: function(enable, network, config, callback) {
    debug("setWifiTethering: " + enable);
    if (!network) {
      this.notifyError(true, callback, "invalid network information");
      return;
    }

    if (!config) {
      this.notifyError(true, callback, "invalid configuration");
      return;
    }

    if (this._usbTetheringRequestCount > 0) {
      // If there's still pending usb tethering request, save
      // the request params and redo |setWifiTethering| on
      // usb tethering task complete.
      debug('USB tethering request is being processed. Queue this wifi tethering request.');
      this._pendingWifiTetheringRequestArgs = Array.prototype.slice.call(arguments);
      debug('Pending args: ' + JSON.stringify(this._pendingWifiTetheringRequestArgs));
      return;
    }

    if (!enable) {
      this.enableWifiTethering(false, config, callback);
      return;
    }

    this._tetheringInterface[TETHERING_TYPE_WIFI].internalInterface = network.name;

    if (this.tetheringSettings[SETTINGS_DUN_REQUIRED]) {
      this.handleDunConnection(true, function(config, callback, network) {
        if (!network) {
          this.notifyError(true, callback, "Dun connection failed");
          return;
        }
        this._tetheringInterface[TETHERING_TYPE_WIFI].externalInterface = network.name;
        this.enableWifiTethering(true, config, callback);
      }.bind(this, config, callback));
      return;
    }

    let mobile = this.getNetworkInterface(
      Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE, this._dataDefaultServiceId);
    // Update the real interface name
    if (mobile && mobile.name) {
      this._tetheringInterface[TETHERING_TYPE_WIFI].externalInterface = mobile.name;
    }

    this.enableWifiTethering(true, config, callback);
  },

  // Enable/disable USB tethering by sending commands to netd.
  setUSBTethering: function(enable, tetheringInterface, callback) {
    let params = this.getUSBTetheringParameters(enable, tetheringInterface);

    if (params === null) {
      gNetworkService.enableUsbRndis(false, function() {
        this.usbTetheringResultReport("Invalid parameters");
      });
      return;
    }

    gNetworkService.setUSBTethering(enable, params, callback);
  },

  getUsbInterface: function() {
    // Find the rndis interface.
    for (let i = 0; i < this.possibleInterface.length; i++) {
      try {
        let file = new FileUtils.File(KERNEL_NETWORK_ENTRY + "/" +
                                      this.possibleInterface[i]);
        if (file.exists()) {
          return this.possibleInterface[i];
        }
      } catch (e) {
        debug("Not " + this.possibleInterface[i] + " interface.");
      }
    }
    debug("Can't find rndis interface in possible lists.");
    return DEFAULT_USB_INTERFACE_NAME;
  },

  enableUsbRndisResult: function(success, enable) {
    if (success) {
      // If enable is false, don't find usb interface cause it is already down,
      // just use the internal interface in settings.
      if (enable) {
        this._tetheringInterface[TETHERING_TYPE_USB].internalInterface = this.getUsbInterface();
      }
      this.setUSBTethering(enable,
                           this._tetheringInterface[TETHERING_TYPE_USB],
                           this.usbTetheringResultReport.bind(this, enable));
    } else {
      this.usbTetheringResultReport(enable, "enableUsbRndisResult failure");
      throw new Error("failed to set USB Function to adb");
    }
  },

  usbTetheringResultReport: function(enable, error) {
    this._usbTetheringRequestCount--;

    let settingsLock = gSettingsService.createLock();

    debug('usbTetheringResultReport callback. enable: ' + enable + ', error: ' + error);

    // Disable tethering settings when fail to enable it.
    if (error) {
      this.tetheringSettings[SETTINGS_USB_ENABLED] = false;
      settingsLock.set("tethering.usb.enabled", false, null);
      // Skip others request when we found an error.
      this._usbTetheringRequestCount = 0;
      this._usbTetheringAction = TETHERING_STATE_IDLE;
      if (this.tetheringSettings[SETTINGS_DUN_REQUIRED]) {
        this.handleDunConnection(false);
      }
    } else {
      if (enable) {
        this._usbTetheringAction = TETHERING_STATE_ACTIVE;
      } else {
        this._usbTetheringAction = TETHERING_STATE_IDLE;
        if (this.tetheringSettings[SETTINGS_DUN_REQUIRED]) {
          this.handleDunConnection(false);
        }
      }

      this.handleLastUsbTetheringRequest();
    }
  },

  onConnectionChangedReport: function(success, externalIfname) {
    debug("onConnectionChangedReport result: success " + success);

    if (success) {
      // Update the external interface.
      this._tetheringInterface[TETHERING_TYPE_USB].externalInterface = externalIfname;
      debug("Change the interface name to " + externalIfname);
    }
  },

  onConnectionChanged: function(network) {
    if (network.state != Ci.nsINetworkInterface.NETWORK_STATE_CONNECTED) {
      debug("We are only interested in CONNECTED event");
      return;
    }

    if (this.tetheringSettings[SETTINGS_DUN_REQUIRED] &&
        network.type === Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN) {
      this.dunConnectTimer.cancel();
      debug("DUN data call connected, process callbacks.");
      while (this._pendingTetheringRequests.length > 0) {
        let callback = this._pendingTetheringRequests.shift();
        if (typeof callback === 'function') {
          callback(network);
        }
      }
      return;
    }

    if (!this.tetheringSettings[SETTINGS_USB_ENABLED]) {
      debug("Usb tethering settings is not enabled");
      return;
    }

    if (this.tetheringSettings[SETTINGS_DUN_REQUIRED] &&
        network.type === Ci.nsINetworkInterface.NETWORK_TYPE_MOBILE_DUN &&
        this._tetheringInterface[TETHERING_TYPE_USB].externalInterface ===
        network.name) {
      debug("Dun required and dun interface is the same");
      return;
    }

    if (this._tetheringInterface[TETHERING_TYPE_USB].externalInterface ===
        this.active.name) {
      debug("The active interface is the same");
      return;
    }

    let previous = {
      internalIfname: this._tetheringInterface[TETHERING_TYPE_USB].internalInterface,
      externalIfname: this._tetheringInterface[TETHERING_TYPE_USB].externalInterface
    };

    let current = {
      internalIfname: this._tetheringInterface[TETHERING_TYPE_USB].internalInterface,
      externalIfname: network.name
    };

    let callback = (function() {
      // Update external network interface.
      debug("Update upstream interface to " + network.name);
      gNetworkService.updateUpStream(previous, current, this.onConnectionChangedReport.bind(this));
    }).bind(this);

    if (this._usbTetheringAction === TETHERING_STATE_ONGOING) {
      debug("Postpone the event and handle it when state is idle.");
      this.wantConnectionEvent = callback;
      return;
    }
    this.wantConnectionEvent = null;

    callback.call(this);
  },

  _setDefaultRouteAndDNS: function(network, oldInterface) {
    gNetworkService.setDefaultRoute(network, oldInterface, function(success) {
      if (!success) {
        gNetworkService.destroyNetwork(network, function() {});
        return;
      }
      gNetworkService.setDNS(network, function(result) {
        gNetworkService.setNetworkProxy(network);
      });
    });
  },
};

let CaptivePortalDetectionHelper = (function() {

  const EVENT_CONNECT = "Connect";
  const EVENT_DISCONNECT = "Disconnect";
  let _ongoingInterface = null;
  let _available = ("nsICaptivePortalDetector" in Ci);
  let getService = function() {
    return Cc['@mozilla.org/toolkit/captive-detector;1']
             .getService(Ci.nsICaptivePortalDetector);
  };

  let _performDetection = function(interfaceName, callback) {
    let capService = getService();
    let capCallback = {
      QueryInterface: XPCOMUtils.generateQI([Ci.nsICaptivePortalCallback]),
      prepare: function() {
        capService.finishPreparation(interfaceName);
      },
      complete: function(success) {
        _ongoingInterface = null;
        callback(success);
      }
    };

    // Abort any unfinished captive portal detection.
    if (_ongoingInterface != null) {
      capService.abort(_ongoingInterface);
      _ongoingInterface = null;
    }
    try {
      capService.checkCaptivePortal(interfaceName, capCallback);
      _ongoingInterface = interfaceName;
    } catch (e) {
      debug('Fail to detect captive portal due to: ' + e.message);
    }
  };

  let _abort = function(interfaceName) {
    if (_ongoingInterface !== interfaceName) {
      return;
    }

    let capService = getService();
    capService.abort(_ongoingInterface);
    _ongoingInterface = null;
  };

  return {
    EVENT_CONNECT: EVENT_CONNECT,
    EVENT_DISCONNECT: EVENT_DISCONNECT,
    notify: function(eventType, network) {
      switch (eventType) {
        case EVENT_CONNECT:
          // perform captive portal detection on wifi interface
          if (_available && network &&
              network.type == Ci.nsINetworkInterface.NETWORK_TYPE_WIFI) {
            _performDetection(network.name, function() {
              // TODO: bug 837600
              // We can disconnect wifi in here if user abort the login procedure.
            });
          }

          break;
        case EVENT_DISCONNECT:
          if (_available &&
              network.type == Ci.nsINetworkInterface.NETWORK_TYPE_WIFI) {
            _abort(network.name);
          }
          break;
      }
    }
  };
}());

XPCOMUtils.defineLazyGetter(NetworkManager.prototype, "mRil", function() {
  try {
    return Cc["@mozilla.org/ril;1"].getService(Ci.nsIRadioInterfaceLayer);
  } catch (e) {}

  return null;
});

this.NSGetFactory = XPCOMUtils.generateNSGetFactory([NetworkManager]);


let debug;
if (DEBUG) {
  debug = function(s) {
    dump("-*- NetworkManager: " + s + "\n");
  };
} else {
  debug = function(s) {};
}
