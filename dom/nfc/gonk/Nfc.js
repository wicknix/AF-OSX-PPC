/* Copyright 2012 Mozilla Foundation and Mozilla contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright © 2013, Deutsche Telekom, Inc. */

"use strict";

const {classes: Cc, interfaces: Ci, utils: Cu, results: Cr} = Components;

Cu.import("resource://gre/modules/XPCOMUtils.jsm");
Cu.import("resource://gre/modules/Services.jsm");

XPCOMUtils.defineLazyServiceGetter(this, "gSettingsService",
                                   "@mozilla.org/settingsService;1",
                                   "nsISettingsService");

XPCOMUtils.defineLazyGetter(this, "NFC", function () {
  let obj = {};
  Cu.import("resource://gre/modules/nfc_consts.js", obj);
  return obj;
});

Cu.import("resource://gre/modules/systemlibs.js");
const NFC_ENABLED = libcutils.property_get("ro.moz.nfc.enabled", "false") === "true";

// set to true in nfc_consts.js to see debug messages
let DEBUG = NFC.DEBUG_NFC;

let debug;
function updateDebug() {
  if (DEBUG || NFC.DEBUG_NFC) {
    debug = function (s) {
      dump("-*- Nfc: " + s + "\n");
    };
  } else {
    debug = function (s) {};
  }
};
updateDebug();

const NFC_CONTRACTID = "@mozilla.org/nfc;1";
const NFC_CID =
  Components.ID("{2ff24790-5e74-11e1-b86c-0800200c9a66}");

const NFC_IPC_MSG_ENTRIES = [
  { permission: null,
    messages: ["NFC:AddEventListener",
               "NFC:QueryInfo",
               "NFC:CallDefaultFoundHandler",
               "NFC:CallDefaultLostHandler"] },

  { permission: "nfc",
    messages: ["NFC:ReadNDEF",
               "NFC:WriteNDEF",
               "NFC:MakeReadOnly",
               "NFC:Format",
               "NFC:Transceive"] },

  { permission: "nfc-share",
    messages: ["NFC:SendFile",
               "NFC:RegisterPeerReadyTarget",
               "NFC:UnregisterPeerReadyTarget"] },

  { permission: "nfc-manager",
    messages: ["NFC:CheckP2PRegistration",
               "NFC:NotifyUserAcceptedP2P",
               "NFC:NotifySendFileStatus",
               "NFC:ChangeRFState",
               "NFC:SetFocusApp"] }
];

XPCOMUtils.defineLazyServiceGetter(this, "ppmm",
                                   "@mozilla.org/parentprocessmessagemanager;1",
                                   "nsIMessageBroadcaster");
XPCOMUtils.defineLazyServiceGetter(this, "gSystemMessenger",
                                   "@mozilla.org/system-message-internal;1",
                                   "nsISystemMessagesInternal");
XPCOMUtils.defineLazyServiceGetter(this, "UUIDGenerator",
                                    "@mozilla.org/uuid-generator;1",
                                    "nsIUUIDGenerator");
XPCOMUtils.defineLazyGetter(this, "gMessageManager", function () {
  return {
    QueryInterface: XPCOMUtils.generateQI([Ci.nsIMessageListener,
                                           Ci.nsIObserver]),

    nfc: null,

    // Manage registered Peer Targets
    peerTargets: {},

    eventListeners: {},

    focusApp: NFC.SYSTEM_APP_ID,

    init: function init(nfc) {
      this.nfc = nfc;

      if (!NFC.DEBUG_NFC) {
        let lock = gSettingsService.createLock();
        lock.get(NFC.SETTING_NFC_DEBUG, this.nfc);
        Services.obs.addObserver(this, NFC.TOPIC_MOZSETTINGS_CHANGED, false);
      }

      Services.obs.addObserver(this, NFC.TOPIC_XPCOM_SHUTDOWN, false);
      this._registerMessageListeners();
    },

    _shutdown: function _shutdown() {
      this.nfc.shutdown();
      this.nfc = null;

      Services.obs.removeObserver(this, NFC.TOPIC_MOZSETTINGS_CHANGED);
      Services.obs.removeObserver(this, NFC.TOPIC_XPCOM_SHUTDOWN);
      this._unregisterMessageListeners();
    },

    _registerMessageListeners: function _registerMessageListeners() {
      ppmm.addMessageListener("child-process-shutdown", this);

      for (let entry of NFC_IPC_MSG_ENTRIES) {
        for (let message of entry.messages) {
          ppmm.addMessageListener(message, this);
        }
      }
    },

    _unregisterMessageListeners: function _unregisterMessageListeners() {
      ppmm.removeMessageListener("child-process-shutdown", this);

      for (let entry of NFC_IPC_MSG_ENTRIES) {
        for (let message of entry.messages) {
          ppmm.removeMessageListener(message, this);
        }
      }

      ppmm = null;
    },

    registerPeerReadyTarget: function registerPeerReadyTarget(target, appId) {
      if (!this.peerTargets[appId]) {
        this.peerTargets[appId] = target;
      }
    },

    unregisterPeerReadyTarget: function unregisterPeerReadyTarget(appId) {
      if (this.peerTargets[appId]) {
        delete this.peerTargets[appId];
      }
    },

    removePeerTarget: function removePeerTarget(target) {
      Object.keys(this.peerTargets).forEach((appId) => {
        if (this.peerTargets[appId] === target) {
          delete this.peerTargets[appId];
        }
      });
    },

    notifyDOMEvent: function notifyDOMEvent(target, options) {
      if (!target) {
        dump("invalid target");
        return;
      }

      target.sendAsyncMessage("NFC:DOMEvent", options);
    },

    setFocusApp: function setFocusApp(id, isFocus) {
      // if calling setNFCFocus(true) on the browser-element which is already
      // focused, or calling setNFCFocus(false) on the browser-element which has
      // lost focus already, ignore.
      if (isFocus == (id == this.focusApp)) {
        return;
      }

      if (this.focusApp != NFC.SYSTEM_APP_ID) {
        this.onFocusChanged(this.focusApp, false);
      }

      if (isFocus) {
        // Now we only support one focus app.
        this.focusApp = id;
        this.onFocusChanged(this.focusApp, true);
      } else if (this.focusApp == id){
        // Set focusApp to SystemApp means currently there is no foreground app.
        this.focusApp = NFC.SYSTEM_APP_ID;
      }
    },

    addEventListener: function addEventListener(target, id) {
      if (this.eventListeners[id] !== undefined) {
        return;
      }

      this.eventListeners[id] = target;
    },

    removeEventListener: function removeEventListener(target) {
      for (let id in this.eventListeners) {
        if (target == this.eventListeners[id]) {
          delete this.eventListeners[id];
          break;
        }
      }
    },

    checkP2PRegistration: function checkP2PRegistration(message) {
      let target = this.peerTargets[message.data.appId];
      let sessionToken = SessionHelper.getCurrentP2PToken();
      let isValid = (sessionToken != null) && (target != null);
      let respMsg = { requestId: message.data.requestId };
      if (!isValid) {
        respMsg.errorMsg = this.nfc.getErrorMessage(NFC.NFC_GECKO_ERROR_P2P_REG_INVALID);
      }
      // Notify the content process immediately of the status
      message.target.sendAsyncMessage(message.name + "Response", respMsg);
    },

    notifyUserAcceptedP2P: function notifyUserAcceptedP2P(appId) {
      let target = this.peerTargets[appId];
      let sessionToken = SessionHelper.getCurrentP2PToken();
      let isValid = (sessionToken != null) && (target != null);
      if (!isValid) {
        debug("Peer already lost or " + appId + " is not a registered PeerReadytarget");
        return;
      }

      this.notifyDOMEvent(target, {event: NFC.PEER_EVENT_READY,
                                   sessionToken: sessionToken});
    },

    callDefaultFoundHandler: function callDefaultFoundHandler(message) {
      let sysMsg = new NfcTechDiscoveredSysMsg(message.sessionToken,
                                               message.isP2P,
                                               message.records || null);
      gSystemMessenger.broadcastMessage("nfc-manager-tech-discovered", sysMsg);
    },

    callDefaultLostHandler: function callDefaultLostHandler(message) {
      // message.isP2P is not used.
      gSystemMessenger.broadcastMessage("nfc-manager-tech-lost", message.sessionToken);
    },

    onTagFound: function onTagFound(message) {
      let target = this.eventListeners[this.focusApp] ||
                   this.eventListeners[NFC.SYSTEM_APP_ID];

      message.event = NFC.TAG_EVENT_FOUND;

      this.notifyDOMEvent(target, message);

      delete message.event;
    },

    onTagLost: function onTagLost(sessionToken) {
      let target = this.eventListeners[this.focusApp] ||
                   this.eventListeners[NFC.SYSTEM_APP_ID];

      this.notifyDOMEvent(target, { event: NFC.TAG_EVENT_LOST,
                                    sessionToken: sessionToken });
    },

    onPeerEvent: function onPeerEvent(eventType, sessionToken) {
      let target = this.eventListeners[this.focusApp] ||
                   this.eventListeners[NFC.SYSTEM_APP_ID];

      this.notifyDOMEvent(target, { event: eventType,
                                    sessionToken: sessionToken });
    },

    onRFStateChanged: function onRFStateChanged(rfState) {
      for (let id in this.eventListeners) {
        this.notifyDOMEvent(this.eventListeners[id],
                            { event: NFC.RF_EVENT_STATE_CHANGED,
                              rfState: rfState });
      }
    },

    onFocusChanged: function onFocusChanged(focusApp, focus) {
      let target = this.eventListeners[focusApp];
      if (!target) {
        return;
      }

      this.notifyDOMEvent(target, { event: NFC.FOCUS_CHANGED,
                                    focus: focus });
    },

    /**
     * nsIMessageListener interface methods.
     */

    receiveMessage: function receiveMessage(message) {
      DEBUG && debug("Received message from content process: " + JSON.stringify(message));

      if (message.name == "child-process-shutdown") {
        this.removePeerTarget(message.target);
        this.nfc.removeTarget(message.target);
        this.removeEventListener(message.target);
        return null;
      }

      for (let entry of NFC_IPC_MSG_ENTRIES) {
        if (entry.messages.indexOf(message.name) != -1) {
          if (entry.permission &&
              !message.target.assertPermission(entry.permission)) {
            debug("Nfc message " + message.name + "doesn't have " +
                  entry.permission + " permission.");
            return null;
          }
          break;
        }
      }

      switch (message.name) {
        case "NFC:SetFocusApp":
          this.setFocusApp(message.data.tabId, message.data.isFocus);
          return null;
        case "NFC:AddEventListener":
          this.addEventListener(message.target, message.data.tabId);
          return null;
        case "NFC:RegisterPeerReadyTarget":
          this.registerPeerReadyTarget(message.target, message.data.appId);
          return null;
        case "NFC:UnregisterPeerReadyTarget":
          this.unregisterPeerReadyTarget(message.data.appId);
          return null;
        case "NFC:CheckP2PRegistration":
          this.checkP2PRegistration(message);
          return null;
        case "NFC:NotifyUserAcceptedP2P":
          this.notifyUserAcceptedP2P(message.data.appId);
          return null;
        case "NFC:NotifySendFileStatus":
          // Upon receiving the status of sendFile operation, send the response
          // to appropriate content process.
          message.data.type = "NotifySendFileStatusResponse";
          if (message.data.status) {
            message.data.errorMsg =
              this.nfc.getErrorMessage(NFC.NFC_GECKO_ERROR_SEND_FILE_FAILED);
          }
          this.nfc.sendNfcResponse(message.data);
          return null;
        case "NFC:CallDefaultFoundHandler":
          this.callDefaultFoundHandler(message.data);
          return null;
        case "NFC:CallDefaultLostHandler":
          this.callDefaultLostHandler(message.data);
          return null;
        default:
          return this.nfc.receiveMessage(message);
      }
    },

    /**
     * nsIObserver interface methods.
     */

    observe: function observe(subject, topic, data) {
      switch (topic) {
        case NFC.TOPIC_MOZSETTINGS_CHANGED:
          if ("wrappedJSObject" in subject) {
            subject = subject.wrappedJSObject;
          }
          if (subject) {
            this.nfc.handle(subject.key, subject.value);
          }
          break;
        case NFC.TOPIC_XPCOM_SHUTDOWN:
          this._shutdown();
          break;
      }
    },
  };
});

let SessionHelper = {
  tokenMap: {},

  registerSession: function registerSession(id, isP2P) {
    if (this.tokenMap[id]) {
      return this.tokenMap[id].token;
    }

    this.tokenMap[id] = {
      token: UUIDGenerator.generateUUID().toString(),
      isP2P: isP2P
    };

    return this.tokenMap[id].token;
  },

  unregisterSession: function unregisterSession(id) {
    if (this.tokenMap[id]) {
      delete this.tokenMap[id];
    }
  },

  getToken: function getToken(id) {
    return this.tokenMap[id] ? this.tokenMap[id].token : null;
  },

  getCurrentP2PToken: function getCurrentP2PToken() {
    for (let id in this.tokenMap) {
      if (this.tokenMap[id] && this.tokenMap[id].isP2P) {
        return this.tokenMap[id].token;
      }
    }
    return null;
  },

  getId: function getId(token) {
    for (let id in this.tokenMap) {
      if (this.tokenMap[id].token == token) {
        return id;
      }
    }

    return 0;
  },

  isP2PSession: function isP2PSession(id) {
    return (this.tokenMap[id] != null) && this.tokenMap[id].isP2P;
  }
};

function Nfc() {
  debug("Starting Nfc Service");

  let nfcService = Cc["@mozilla.org/nfc/service;1"].getService(Ci.nsINfcService);
  if (!nfcService) {
    debug("No nfc service component available!");
    return;
  }

  nfcService.start(this);
  this.nfcService = nfcService;

  gMessageManager.init(this);

  this.targetsByRequestId = {};
}

Nfc.prototype = {

  classID:   NFC_CID,
  classInfo: XPCOMUtils.generateCI({classID: NFC_CID,
                                    classDescription: "Nfc",
                                    interfaces: [Ci.nsINfcService]}),

  QueryInterface: XPCOMUtils.generateQI([Ci.nsIObserver, Ci.nsINfcGonkEventListener]),

  rfState: NFC.NFC_RF_STATE_IDLE,

  nfcService: null,

  targetsByRequestId: null,

  /**
   * Send arbitrary message to Nfc service.
   *
   * @param nfcMessageType
   *        A text message type.
   * @param message [optional]
   *        An optional message object to send.
   */
  sendToNfcService: function sendToNfcService(nfcMessageType, message) {
    message = message || {};
    message.type = nfcMessageType;
    this.nfcService.sendCommand(message);
  },

  sendNfcResponse: function sendNfcResponse(message) {
    let target = this.targetsByRequestId[message.requestId];
    if (!target) {
      debug("No target for requestId: " + message.requestId);
      return;
    }
    delete this.targetsByRequestId[message.requestId];

    target.sendAsyncMessage("NFC:" + message.type, message);
  },

  /**
   * Send Error response to content. This is used only
   * in case of discovering an error in message received from
   * content process.
   *
   * @param message
   *        An nsIMessageListener's message parameter.
   */
  sendNfcErrorResponse: function sendNfcErrorResponse(message, errorCode) {
    if (!message.target) {
      return;
    }

    let nfcMsgType = message.name + "Response";
    message.data.errorMsg = this.getErrorMessage(errorCode);
    message.target.sendAsyncMessage(nfcMsgType, message.data);
  },

  getErrorMessage: function getErrorMessage(errorCode) {
    return NFC.NFC_ERROR_MSG[errorCode];
  },

  /**
   * Process the incoming message from the NFC Service.
   */
  onEvent: function onEvent(event) {
    let message = Cu.cloneInto(event, this);
    DEBUG && debug("Received message from NFC Service: " + JSON.stringify(message));

    switch (message.type) {
      case "InitializedNotification":
        // Do nothing.
        break;
      case "TechDiscoveredNotification":
        message.type = "techDiscovered";
        // Update the upper layers with a session token (alias)
        message.sessionToken =
          SessionHelper.registerSession(message.sessionId, message.isP2P);
        // Do not expose the actual session to the content
        let sessionId = message.sessionId;
        delete message.sessionId;

        if (SessionHelper.isP2PSession(sessionId)) {
          if (message.records) {
            // TODO: Bug 1082493.
            // This event should be sent to the focus app, but before Bug 1082493
            // is landed we forward this to System app.
            gMessageManager.callDefaultFoundHandler(message);
          } else {
            gMessageManager.onPeerEvent(NFC.PEER_EVENT_FOUND, message.sessionToken);
          }
        } else {
          gMessageManager.onTagFound(message);
        }
        break;
      case "TechLostNotification":
        message.type = "techLost";

        // Update the upper layers with a session token (alias)
        message.sessionToken = SessionHelper.getToken(message.sessionId);
        if (SessionHelper.isP2PSession(message.sessionId)) {
          gMessageManager.onPeerEvent(NFC.PEER_EVENT_LOST, message.sessionToken);
        } else {
          gMessageManager.onTagLost(message.sessionToken);
        }

        SessionHelper.unregisterSession(message.sessionId);
        break;
     case "HCIEventTransactionNotification":
        this.notifyHCIEventTransaction(message);
        break;
     case "ChangeRFStateResponse":
        this.sendNfcResponse(message);

        if (!message.errorMsg) {
          this.rfState = message.rfState;
          gMessageManager.onRFStateChanged(this.rfState);
        }
        break;
      case "ReadNDEFResponse": // Fall through.
      case "MakeReadOnlyResponse":
      case "FormatResponse":
      case "TransceiveResponse":
      case "WriteNDEFResponse":
        this.sendNfcResponse(message);
        break;
      default:
        throw new Error("Don't know about this message type: " + message.type);
    }
  },

  // HCI Event Transaction
  notifyHCIEventTransaction: function notifyHCIEventTransaction(message) {
    delete message.type;
    /**
     * FIXME:
     * GSMA 6.0 7.4 UI Application triggering requirements
     * This specifies the need for the following parameters to be derived and
     * sent. One unclear spec is what the URI format "secure:0" refers to, given
     * SEName can be something like "SIM1" or "SIM2".
     *
     * 1) Mime-type - Secure Element application dependent
     * 2) URI,  of the format:  nfc://secure:0/<SEName>/<AID>
     *     - SEName reflects the originating SE. It must be compliant with
     *       SIMAlliance Open Mobile APIs
     *     - AID reflects the originating UICC applet identifier
     * 3) Data - Data payload of the transaction notification, if any.
     */
    gSystemMessenger.broadcastMessage("nfc-hci-event-transaction", message);
  },

  /**
   * Process a message from the gMessageManager.
   */
  receiveMessage: function receiveMessage(message) {
      if (["NFC:ChangeRFState",
           "NFC:SendFile",
           "NFC:QueryInfo"].indexOf(message.name) == -1) {
      // Update the current sessionId before sending to the NFC service.
      message.data.sessionId = SessionHelper.getId(message.data.sessionToken);
    }

    switch (message.name) {
      case "NFC:ChangeRFState":
        this.sendToNfcService("changeRFState", message.data);
        break;
      case "NFC:ReadNDEF":
        this.sendToNfcService("readNDEF", message.data);
        break;
      case "NFC:WriteNDEF":
        message.data.isP2P = SessionHelper.isP2PSession(message.data.sessionId);
        this.sendToNfcService("writeNDEF", message.data);
        break;
      case "NFC:MakeReadOnly":
        this.sendToNfcService("makeReadOnly", message.data);
        break;
      case "NFC:Format":
        this.sendToNfcService("format", message.data);
        break;
      case "NFC:Transceive":
        this.sendToNfcService("transceive", message.data);
        break;
      case "NFC:SendFile":
        // Chrome process is the arbitrator / mediator between
        // system app (content process) that issued nfc 'sendFile' operation
        // and system app that handles the system message :
        // 'nfc-manager-send-file'. System app subsequently handover's
        // the data to alternate carrier's (BT / WiFi) 'sendFile' interface.

        // Notify system app to initiate BT send file operation
        let sysMsg = new NfcSendFileSysMsg(message.data.requestId,
                                           message.data.sessionToken,
                                           message.data.blob);
        gSystemMessenger.broadcastMessage("nfc-manager-send-file",
                                          sysMsg);
        break;
      case "NFC:QueryInfo":
        return {rfState: this.rfState};
      default:
        debug("UnSupported : Message Name " + message.name);
        return null;
    }
    this.targetsByRequestId[message.data.requestId] = message.target;

    return null;
  },

  removeTarget: function removeTarget(target) {
    Object.keys(this.targetsByRequestId).forEach((requestId) => {
      if (this.targetsByRequestId[requestId] === target) {
        delete this.targetsByRequestId[requestId];
      }
    });
  },

  /**
   * nsISettingsServiceCallback
   */
  handle: function handle(name, result) {
    switch (name) {
      case NFC.SETTING_NFC_DEBUG:
        DEBUG = result;
        updateDebug();
        break;
    }
  },

  /**
   * nsIObserver interface methods.
   */
  observe: function(subject, topic, data) {
    if (topic != "profile-after-change") {
      debug("Should receive 'profile-after-change' only, received " + topic);
    }
  },

  shutdown: function shutdown() {
    this.nfcService.shutdown();
    this.nfcService = null;
  }
};

function NfcTechDiscoveredSysMsg(sessionToken, isP2P, records) {
  this.sessionToken = sessionToken;
  this.isP2P = isP2P;
  this.records = records;
}
NfcTechDiscoveredSysMsg.prototype = {
  QueryInterface: XPCOMUtils.generateQI([Ci.nsINfcTechDiscoveredSysMsg]),

  sessionToken: null,
  isP2P: null,
  records: null
};

function NfcSendFileSysMsg(requestId, sessionToken, blob) {
  this.requestId = requestId;
  this.sessionToken = sessionToken;
  this.blob = blob;
}
NfcSendFileSysMsg.prototype = {
  QueryInterface: XPCOMUtils.generateQI([Ci.nsINfcSendFileSysMsg]),

  requestId: null,
  sessionToken: null,
  blob: null
};

if (NFC_ENABLED) {
  this.NSGetFactory = XPCOMUtils.generateNSGetFactory([Nfc]);
}
