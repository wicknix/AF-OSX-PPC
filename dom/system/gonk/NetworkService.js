/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

const {classes: Cc, interfaces: Ci, utils: Cu, results: Cr} = Components;

Cu.import("resource://gre/modules/XPCOMUtils.jsm");
Cu.import("resource://gre/modules/Services.jsm");
Cu.import("resource://gre/modules/NetUtil.jsm");
Cu.import("resource://gre/modules/FileUtils.jsm");
Cu.import("resource://gre/modules/Promise.jsm");

const NETWORKSERVICE_CONTRACTID = "@mozilla.org/network/service;1";
const NETWORKSERVICE_CID = Components.ID("{baec696c-c78d-42db-8b44-603f8fbfafb4}");

XPCOMUtils.defineLazyServiceGetter(this, "gNetworkWorker",
                                   "@mozilla.org/network/worker;1",
                                   "nsINetworkWorker");

// 1xx - Requested action is proceeding
const NETD_COMMAND_PROCEEDING   = 100;
// 2xx - Requested action has been successfully completed
const NETD_COMMAND_OKAY         = 200;
// 4xx - The command is accepted but the requested action didn't
// take place.
const NETD_COMMAND_FAIL         = 400;
// 5xx - The command syntax or parameters error
const NETD_COMMAND_ERROR        = 500;
// 6xx - Unsolicited broadcasts
const NETD_COMMAND_UNSOLICITED  = 600;

const WIFI_CTRL_INTERFACE = "wl0.1";

const MANUAL_PROXY_CONFIGURATION = 1;

let DEBUG = false;

// Read debug setting from pref.
try {
  let debugPref = Services.prefs.getBoolPref("network.debugging.enabled");
  DEBUG = DEBUG || debugPref;
} catch (e) {}

function netdResponseType(code) {
  return Math.floor(code / 100) * 100;
}

function isError(code) {
  let type = netdResponseType(code);
  return (type !== NETD_COMMAND_PROCEEDING && type !== NETD_COMMAND_OKAY);
}

function debug(msg) {
  dump("-*- NetworkService: " + msg + "\n");
}

function Task(id, params, setupFunction) {
  this.id = id;
  this.params = params;
  this.setupFunction = setupFunction;
}

function NetworkWorkerRequestQueue(networkService) {
  this.networkService = networkService;
  this.tasks = [];
}
NetworkWorkerRequestQueue.prototype = {
  runQueue: function() {
    if (this.tasks.length === 0) {
      return;
    }

    let task = this.tasks[0];
    if (DEBUG) debug("run task id: " + task.id);

    if (typeof task.setupFunction === 'function') {
      // If setupFunction returns false, skip sending to Network Worker but call
      // handleWorkerMessage() directly with task id, as if the response was
      // returned from Network Worker.
      if (!task.setupFunction()) {
        this.networkService.handleWorkerMessage({id: task.id});
        return;
      }
    }

    gNetworkWorker.postMessage(task.params);
  },

  enqueue: function(id, params, setupFunction) {
    if (DEBUG) debug("enqueue id: " + id);
    this.tasks.push(new Task(id, params, setupFunction));

    if (this.tasks.length === 1) {
      this.runQueue();
    }
  },

  dequeue: function(id) {
    if (DEBUG) debug("dequeue id: " + id);

    if (!this.tasks.length || this.tasks[0].id != id) {
      if (DEBUG) debug("Id " + id + " is not on top of the queue");
      return;
    }

    this.tasks.shift();
    if (this.tasks.length > 0) {
      // Run queue on the next tick.
      Services.tm.currentThread.dispatch(() => {
        this.runQueue();
      }, Ci.nsIThread.DISPATCH_NORMAL);
    }
  }
};


/**
 * This component watches for network interfaces changing state and then
 * adjusts routes etc. accordingly.
 */
function NetworkService() {
  if(DEBUG) debug("Starting net_worker.");

  let self = this;

  if (gNetworkWorker) {
    let networkListener = {
      onEvent: function(event) {
        self.handleWorkerMessage(event);
      }
    };
    gNetworkWorker.start(networkListener);
  }
  // Callbacks to invoke when a reply arrives from the net_worker.
  this.controlCallbacks = Object.create(null);

  this.addedRoutes = new Map();
  this.netWorkerRequestQueue = new NetworkWorkerRequestQueue(this);
  this.shutdown = false;
  Services.obs.addObserver(this, "xpcom-shutdown", false);
}

NetworkService.prototype = {
  classID:   NETWORKSERVICE_CID,
  classInfo: XPCOMUtils.generateCI({classID: NETWORKSERVICE_CID,
                                    contractID: NETWORKSERVICE_CONTRACTID,
                                    classDescription: "Network Service",
                                    interfaces: [Ci.nsINetworkService]}),
  QueryInterface: XPCOMUtils.generateQI([Ci.nsINetworkService]),

  // Helpers

  addedRoutes: null,
  idgen: 0,
  controlMessage: function(params, callback, setupFunction) {
    if (this.shutdown) {
      return;
    }

    let id = this.idgen++;
    params.id = id;
    if (callback) {
      this.controlCallbacks[id] = callback;
    }

    // For now, we use setupFunction to determine if this command needs to be
    // queued or not.
    if (setupFunction) {
      this.netWorkerRequestQueue.enqueue(id, params, setupFunction);
      return;
    }

    if (gNetworkWorker) {
      gNetworkWorker.postMessage(params);
    }
  },

  handleWorkerMessage: function(response) {
    if(DEBUG) debug("NetworkManager received message from worker: " + JSON.stringify(response));
    let id = response.id;
    if (response.broadcast === true) {
      Services.obs.notifyObservers(null, response.topic, response.reason);
      return;
    }
    let callback = this.controlCallbacks[id];
    if (callback) {
      callback.call(this, response);
      delete this.controlCallbacks[id];
    }

    this.netWorkerRequestQueue.dequeue(id);
  },

  // nsINetworkService

  getNetworkInterfaceStats: function(networkName, callback) {
    if(DEBUG) debug("getNetworkInterfaceStats for " + networkName);

    let file = new FileUtils.File("/proc/net/dev");
    if (!file) {
      callback.networkStatsAvailable(false, 0, 0, Date.now());
      return;
    }

    NetUtil.asyncFetch2(file, function(inputStream, status) {
      let rxBytes = 0,
          txBytes = 0,
          now = Date.now();

      if (Components.isSuccessCode(status)) {
        // Find record for corresponding interface.
        let statExpr = /(\S+): +(\d+) +\d+ +\d+ +\d+ +\d+ +\d+ +\d+ +\d+ +(\d+) +\d+ +\d+ +\d+ +\d+ +\d+ +\d+ +\d+/;
        let data =
          NetUtil.readInputStreamToString(inputStream, inputStream.available())
                 .split("\n");
        for (let i = 2; i < data.length; i++) {
          let parseResult = statExpr.exec(data[i]);
          if (parseResult && parseResult[1] === networkName) {
            rxBytes = parseInt(parseResult[2], 10);
            txBytes = parseInt(parseResult[3], 10);
            break;
          }
        }
      }

      // netd always return success even interface doesn't exist.
      callback.networkStatsAvailable(true, rxBytes, txBytes, now);
    },
    null,      // aLoadingNode
    Services.scriptSecurityManager.getSystemPrincipal(),
    null,      // aTriggeringPrincipal
    Ci.nsILoadInfo.SEC_NORMAL,
    Ci.nsIContentPolicy.TYPE_OTHER);
  },

  setNetworkInterfaceAlarm: function(networkName, threshold, callback) {
    if (!networkName) {
      callback.networkUsageAlarmResult(-1);
      return;
    }

    let self = this;
    this._disableNetworkInterfaceAlarm(networkName, function(result) {
      if (threshold < 0) {
        if (!isError(result.resultCode)) {
          callback.networkUsageAlarmResult(null);
          return;
        }
        callback.networkUsageAlarmResult(result.reason);
        return
      }

      self._setNetworkInterfaceAlarm(networkName, threshold, callback);
    });
  },

  _setNetworkInterfaceAlarm: function(networkName, threshold, callback) {
    if(DEBUG) debug("setNetworkInterfaceAlarm for " + networkName + " at " + threshold + "bytes");

    let params = {
      cmd: "setNetworkInterfaceAlarm",
      ifname: networkName,
      threshold: threshold
    };

    params.report = true;

    this.controlMessage(params, function(result) {
      if (!isError(result.resultCode)) {
        callback.networkUsageAlarmResult(null);
        return;
      }

      this._enableNetworkInterfaceAlarm(networkName, threshold, callback);
    });
  },

  _enableNetworkInterfaceAlarm: function(networkName, threshold, callback) {
    if(DEBUG) debug("enableNetworkInterfaceAlarm for " + networkName + " at " + threshold + "bytes");

    let params = {
      cmd: "enableNetworkInterfaceAlarm",
      ifname: networkName,
      threshold: threshold
    };

    params.report = true;

    this.controlMessage(params, function(result) {
      if (!isError(result.resultCode)) {
        callback.networkUsageAlarmResult(null);
        return;
      }
      callback.networkUsageAlarmResult(result.reason);
    });
  },

  _disableNetworkInterfaceAlarm: function(networkName, callback) {
    if(DEBUG) debug("disableNetworkInterfaceAlarm for " + networkName);

    let params = {
      cmd: "disableNetworkInterfaceAlarm",
      ifname: networkName,
    };

    params.report = true;

    this.controlMessage(params, function(result) {
      callback(result);
    });
  },

  setWifiOperationMode: function(interfaceName, mode, callback) {
    if(DEBUG) debug("setWifiOperationMode on " + interfaceName + " to " + mode);

    let params = {
      cmd: "setWifiOperationMode",
      ifname: interfaceName,
      mode: mode
    };

    params.report = true;

    this.controlMessage(params, function(result) {
      if (isError(result.resultCode)) {
        callback.wifiOperationModeResult("netd command error");
      } else {
        callback.wifiOperationModeResult(null);
      }
    });
  },

  resetRoutingTable: function(network) {
    let options = {
      cmd: "removeNetworkRoute",
      ifname: network.name
    };

    this.controlMessage(options);
  },

  setDNS: function(networkInterface, callback) {
    if (DEBUG) debug("Going DNS to " + networkInterface.name);
    let dnses = networkInterface.getDnses();
    let options = {
      cmd: "setDNS",
      ifname: networkInterface.name,
      domain: "mozilla." + networkInterface.name + ".doman",
      dnses: dnses
    };
    this.controlMessage(options, function(result) {
      callback.setDnsResult(result.success ? null : result.reason);
    });
  },

  setDefaultRoute: function(network, oldInterface, callback) {
    if (DEBUG) debug("Going to change default route to " + network.name);
    let gateways = network.getGateways();
    let options = {
      cmd: "setDefaultRoute",
      ifname: network.name,
      oldIfname: (oldInterface && oldInterface !== network) ? oldInterface.name : null,
      gateways: gateways
    };
    this.controlMessage(options, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  removeDefaultRoute: function(network) {
    if(DEBUG) debug("Remove default route for " + network.name);
    let gateways = network.getGateways();
    let options = {
      cmd: "removeDefaultRoute",
      ifname: network.name,
      gateways: gateways
    };
    this.controlMessage(options);
  },

  _routeToString: function(interfaceName, host, prefixLength, gateway) {
    return host + "-" + prefixLength + "-" + gateway + "-" + interfaceName;
  },

  modifyRoute: function(action, interfaceName, host, prefixLength, gateway) {
    let command;

    switch (action) {
      case Ci.nsINetworkService.MODIFY_ROUTE_ADD:
        command = 'addHostRoute';
        break;
      case Ci.nsINetworkService.MODIFY_ROUTE_REMOVE:
        command = 'removeHostRoute';
        break;
      default:
        if (DEBUG) debug('Unknown action: ' + action);
        return Promise.reject();
    }

    let route = this._routeToString(interfaceName, host, prefixLength, gateway);
    let setupFunc = () => {
      let count = this.addedRoutes.get(route);
      if (DEBUG) debug(command + ": " + route + " -> " + count);

      // Return false if there is no need to send the command to network worker.
      if ((action == Ci.nsINetworkService.MODIFY_ROUTE_ADD && count) ||
          (action == Ci.nsINetworkService.MODIFY_ROUTE_REMOVE &&
           (!count || count > 1))) {
        return false;
      }

      return true;
    };

    if (DEBUG) debug(command + " " + host + " on " + interfaceName);
    let options = {
      cmd: command,
      ifname: interfaceName,
      gateway: gateway,
      prefixLength: prefixLength,
      ip: host
    };

    return new Promise((aResolve, aReject) => {
      this.controlMessage(options, (data) => {
        let count = this.addedRoutes.get(route);

        // Remove route from addedRoutes on success or failure.
        if (action == Ci.nsINetworkService.MODIFY_ROUTE_REMOVE) {
          if (count > 1) {
            this.addedRoutes.set(route, count - 1);
          } else {
            this.addedRoutes.delete(route);
          }
        }

        if (data.error) {
          aReject(data.reason);
          return;
        }

        if (action == Ci.nsINetworkService.MODIFY_ROUTE_ADD) {
          this.addedRoutes.set(route, count ? count + 1 : 1);
        }

        aResolve();
      }, setupFunc);
    });
  },

  addSecondaryRoute: function(ifname, route) {
    if(DEBUG) debug("Going to add route to secondary table on " + ifname);
    let options = {
      cmd: "addSecondaryRoute",
      ifname: ifname,
      ip: route.ip,
      prefix: route.prefix,
      gateway: route.gateway
    };
    this.controlMessage(options);
  },

  removeSecondaryRoute: function(ifname, route) {
    if(DEBUG) debug("Going to remove route from secondary table on " + ifname);
    let options = {
      cmd: "removeSecondaryRoute",
      ifname: ifname,
      ip: route.ip,
      prefix: route.prefix,
      gateway: route.gateway
    };
    this.controlMessage(options);
  },

  setNetworkProxy: function(network) {
    try {
      if (!network.httpProxyHost || network.httpProxyHost === "") {
        // Sets direct connection to internet.
        this.clearNetworkProxy();

        if (DEBUG) debug("No proxy support for " + network.name + " network interface.");
        return;
      }

      if (DEBUG) debug("Going to set proxy settings for " + network.name + " network interface.");
      // Sets manual proxy configuration.
      Services.prefs.setIntPref("network.proxy.type", MANUAL_PROXY_CONFIGURATION);
      // Do not use this proxy server for all protocols.
      Services.prefs.setBoolPref("network.proxy.share_proxy_settings", false);
      Services.prefs.setCharPref("network.proxy.http", network.httpProxyHost);
      Services.prefs.setCharPref("network.proxy.ssl", network.httpProxyHost);
      let port = network.httpProxyPort === 0 ? 8080 : network.httpProxyPort;
      Services.prefs.setIntPref("network.proxy.http_port", port);
      Services.prefs.setIntPref("network.proxy.ssl_port", port);
    } catch(ex) {
        if (DEBUG) debug("Exception " + ex + ". Unable to set proxy setting for " +
                         network.name + " network interface.");
    }
  },

  clearNetworkProxy: function() {
    if (DEBUG) debug("Going to clear all network proxy.");

    Services.prefs.clearUserPref("network.proxy.type");
    Services.prefs.clearUserPref("network.proxy.share_proxy_settings");
    Services.prefs.clearUserPref("network.proxy.http");
    Services.prefs.clearUserPref("network.proxy.http_port");
    Services.prefs.clearUserPref("network.proxy.ssl");
    Services.prefs.clearUserPref("network.proxy.ssl_port");
  },

  // Enable/Disable DHCP server.
  setDhcpServer: function(enabled, config, callback) {
    if (null === config) {
      config = {};
    }

    config.cmd = "setDhcpServer";
    config.enabled = enabled;

    this.controlMessage(config, function setDhcpServerResult(response) {
      if (!response.success) {
        callback.dhcpServerResult('Set DHCP server error');
        return;
      }
      callback.dhcpServerResult(null);
    });
  },

  // Enable/disable WiFi tethering by sending commands to netd.
  setWifiTethering: function(enable, config, callback) {
    // config should've already contained:
    //   .ifname
    //   .internalIfname
    //   .externalIfname
    config.wifictrlinterfacename = WIFI_CTRL_INTERFACE;
    config.cmd = "setWifiTethering";

    // The callback function in controlMessage may not be fired immediately.
    this.controlMessage(config, function setWifiTetheringResult(data) {
      let code = data.resultCode;
      let reason = data.resultReason;
      let enable = data.enable;
      let enableString = enable ? "Enable" : "Disable";

      if(DEBUG) debug(enableString + " Wifi tethering result: Code " + code + " reason " + reason);

      if (isError(code)) {
        callback.wifiTetheringEnabledChange("netd command error");
      } else {
        callback.wifiTetheringEnabledChange(null);
      }
    });
  },

  // Enable/disable USB tethering by sending commands to netd.
  setUSBTethering: function(enable, config, callback) {
    config.cmd = "setUSBTethering";
    // The callback function in controlMessage may not be fired immediately.
    this.controlMessage(config, function setUsbTetheringResult(data) {
      let code = data.resultCode;
      let reason = data.resultReason;
      let enable = data.enable;
      let enableString = enable ? "Enable" : "Disable";

      if(DEBUG) debug(enableString + " USB tethering result: Code " + code + " reason " + reason);

      if (isError(code)) {
        callback.usbTetheringEnabledChange("netd command error");
      } else {
        callback.usbTetheringEnabledChange(null);
      }
    });
  },

  // Switch usb function by modifying property of persist.sys.usb.config.
  enableUsbRndis: function(enable, callback) {
    if(DEBUG) debug("enableUsbRndis: " + enable);

    let params = {
      cmd: "enableUsbRndis",
      enable: enable
    };
    // Ask net work to report the result when this value is set to true.
    if (callback) {
      params.report = true;
    } else {
      params.report = false;
    }

    // The callback function in controlMessage may not be fired immediately.
    //this._usbTetheringAction = TETHERING_STATE_ONGOING;
    this.controlMessage(params, function(data) {
      callback.enableUsbRndisResult(data.result, data.enable);
    });
  },

  updateUpStream: function(previous, current, callback) {
    let params = {
      cmd: "updateUpStream",
      preInternalIfname: previous.internalIfname,
      preExternalIfname: previous.externalIfname,
      curInternalIfname: current.internalIfname,
      curExternalIfname: current.externalIfname
    };

    this.controlMessage(params, function(data) {
      let code = data.resultCode;
      let reason = data.resultReason;
      if(DEBUG) debug("updateUpStream result: Code " + code + " reason " + reason);
      callback.updateUpStreamResult(!isError(code), data.curExternalIfname);
    });
  },

  configureInterface: function(config, callback) {
    let params = {
      cmd: "configureInterface",
      ifname: config.ifname,
      ipaddr: config.ipaddr,
      mask: config.mask,
      gateway_long: config.gateway,
      dns1_long: config.dns1,
      dns2_long: config.dns2,
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  dhcpRequest: function(interfaceName, callback) {
    let params = {
      cmd: "dhcpRequest",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.dhcpRequestResult(!result.error, result.error ? null : result);
    });
  },

  enableInterface: function(interfaceName, callback) {
    let params = {
      cmd: "enableInterface",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  disableInterface: function(interfaceName, callback) {
    let params = {
      cmd: "disableInterface",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  resetConnections: function(interfaceName, callback) {
    let params = {
      cmd: "resetConnections",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  createNetwork: function(interfaceName, callback) {
    let params = {
      cmd: "createNetwork",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  destroyNetwork: function(interfaceName, callback) {
    let params = {
      cmd: "destroyNetwork",
      ifname: interfaceName
    };

    this.controlMessage(params, function(result) {
      callback.nativeCommandResult(!result.error);
    });
  },

  shutdown: false,

  observe: function observe(aSubject, aTopic, aData) {
    switch (aTopic) {
      case "xpcom-shutdown":
        debug("NetworkService shutdown");
        this.shutdown = true;
        Services.obs.removeObserver(this, "xpcom-shutdown");
        if (gNetworkWorker) {
          gNetworkWorker.shutdown();
          gNetworkWorker = null;
        }
        break;
    }
  },
};

this.NSGetFactory = XPCOMUtils.generateNSGetFactory([NetworkService]);
