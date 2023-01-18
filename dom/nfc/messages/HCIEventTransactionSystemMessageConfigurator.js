/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

const {classes: Cc, interfaces: Ci, utils: Cu, results: Cr} = Components;

Cu.import("resource://gre/modules/XPCOMUtils.jsm");
Cu.import("resource://gre/modules/Promise.jsm");

XPCOMUtils.defineLazyServiceGetter(this, "appsService",
                                   "@mozilla.org/AppsService;1",
                                   "nsIAppsService");

let DEBUG = false;
function debug(aMsg) {
  if (DEBUG) {
    dump("-- HCIEventTransactionSystemMessageConfigurator.js " + Date.now() + " : " + aMsg + "\n");
  }
}

/**
  * nsISystemMessagesConfigurator implementation.
  */
function HCIEventTransactionSystemMessageConfigurator() {
  debug("HCIEventTransactionSystemMessageConfigurator");
}

HCIEventTransactionSystemMessageConfigurator.prototype = {
  get mustShowRunningApp() {
    debug("mustShowRunningApp returning true");
    return true;
  },

  shouldDispatch: function shouldDispatch(aManifestURL, aPageURL, aType, aMessage, aExtra) {
    DEBUG && debug("message to dispatch: " + JSON.stringify(aMessage));
    debug("aManifest url: " + aManifestURL);

    if (!aMessage) {
      return Promise.resolve(false);
    }

    return new Promise((resolve, reject) => {
      appsService.getManifestFor(aManifestURL)
      .then((aManifest) => this._checkAppManifest(aMessage.origin, aMessage.aid, aManifest))
      .then(() => {
        // FIXME: Bug 884594: Access Control Enforcer
        // Here we will call ace.isAllowed function which will also return
        // a Promise, for now we're just resolving shouldDispatch promise
        debug("dispatching message");
        resolve(true);
      })
      .catch(() => {
        // if the Promise chain was broken we don't dispatch the message
        debug("not dispatching");
        resolve(false);
      });
    });
  },

  // we might be doing some async hash computations here, returning
  // a resolved/rejected promise for now so we can easily fit the method
  // into a Promise chain
  _checkAppManifest: function _checkAppManifest(aOrigin, aAid, aManifest) {
    DEBUG && debug("aManifest " + JSON.stringify(aManifest));

    // convert AID and Secure Element name to uppercased string for comparison
    // with manifest secure_element_access rules
    let aid = this._byteAIDToHex(aAid);
    let seName = (aOrigin) ? aOrigin.toUpperCase() : "";

    let hciRules = aManifest["secure_element_access"] || [];
    let matchingRule = hciRules.find((rule) => {
      rule = rule.toUpperCase();
      if(rule === "*" || rule === (seName + "/" + aid)) {
        return true;
      }

      let isMatching = (match, element) => {
        if(match === "*") {
          return true;
        }
        if(match.charAt(match.length - 1) === '*') {
          return element.indexOf(match.substr(0,match.length - 1)) === 0;
        }

        return match === element;
      };

      return isMatching(rule.split('/')[0], seName) &&
             isMatching(rule.split('/')[1], aid);
    });

    return (matchingRule) ? Promise.resolve() : Promise.reject();
  },

  // FIXME: there is probably something which does this
  _byteAIDToHex: function _byteAIDToHex(uint8arr) {
    if (!uint8arr) {
      return "";
    }

    var hexStr = "";
    for (var i = 0; i < uint8arr.length; i++) {
      var hex = (uint8arr[i] & 0xff).toString(16);
      hex = (hex.length === 1) ? '0' + hex : hex;
      hexStr += hex;
    }
    return hexStr.toUpperCase();
  },

  classID: Components.ID("{b501edd0-28bd-11e4-8c21-0800200c9a66}"),
  QueryInterface: XPCOMUtils.generateQI([Ci.nsISystemMessagesConfigurator])
}

this.NSGetFactory = XPCOMUtils.generateNSGetFactory([HCIEventTransactionSystemMessageConfigurator]);
