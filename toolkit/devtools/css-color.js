/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

const {Cc, Ci, Cu} = require("chrome");
const {Services} = Cu.import("resource://gre/modules/Services.jsm", {});

const COLOR_UNIT_PREF = "devtools.defaultColorUnit";

const REGEX_JUST_QUOTES  = /^""$/;
const REGEX_HSL_3_TUPLE  = /^\bhsl\(([\d.]+),\s*([\d.]+%),\s*([\d.]+%)\)$/i;

/**
 * This regex matches:
 *  - #F00
 *  - #FF0000
 *  - hsl()
 *  - hsla()
 *  - rgb()
 *  - rgba()
 *  - red
 *
 *  It also matches css keywords e.g. "background-color" otherwise
 *  "background" would be replaced with #6363CE ("background" is a platform
 *  color).
 */
const REGEX_ALL_COLORS = /#[0-9a-fA-F]{3}\b|#[0-9a-fA-F]{6}\b|hsl\(.*?\)|hsla\(.*?\)|rgba?\(.*?\)|\b[a-zA-Z-]+\b/g;

const SPECIALVALUES = new Set([
  "currentcolor",
  "initial",
  "inherit",
  "transparent",
  "unset"
]);

/**
 * This module is used to convert between various color types.
 *
 * Usage:
 *   let {devtools} = Cu.import("resource://gre/modules/devtools/Loader.jsm", {});
 *   let {colorUtils} = devtools.require("devtools/css-color");
 *   let color = new colorUtils.CssColor("red");
 *
 *   color.authored === "red"
 *   color.hasAlpha === false
 *   color.valid === true
 *   color.transparent === false // transparent has a special status.
 *   color.name === "red"        // returns hex or rgba when no name available.
 *   color.hex === "#F00"        // returns shortHex when available else returns
 *                                  longHex. If alpha channel is present then we
 *                                  return this.rgba.
 *   color.longHex === "#FF0000" // If alpha channel is present then we return
 *                                  this.rgba.
 *   color.rgb === "rgb(255, 0, 0)" // If alpha channel is present then we return
 *                                     this.rgba.
 *   color.rgba === "rgba(255, 0, 0, 1)"
 *   color.hsl === "hsl(0, 100%, 50%)"
 *   color.hsla === "hsla(0, 100%, 50%, 1)" // If alpha channel is present
 *                                             then we return this.rgba.
 *
 *   color.toString() === "#F00"; // Outputs the color type determined in the
 *                                   COLOR_UNIT_PREF constant (above).
 *   // Color objects can be reused
 *   color.newColor("green") === "#0F0"; // true
 *
 *   let processed = colorUtils.processCSSString("color:red; background-color:green;");
 *   // Returns "color:#F00; background-color:#0F0;"
 *
 *   Valid values for COLOR_UNIT_PREF are contained in CssColor.COLORUNIT.
 */

function CssColor(colorValue) {
  this.newColor(colorValue);
}

module.exports.colorUtils = {
  CssColor: CssColor,
  processCSSString: processCSSString,
  rgbToHsl: rgbToHsl,
  setAlpha: setAlpha
};

/**
 * Values used in COLOR_UNIT_PREF
 */
CssColor.COLORUNIT = {
  "authored": "authored",
  "hex": "hex",
  "name": "name",
  "rgb": "rgb",
  "hsl": "hsl"
};

CssColor.prototype = {
  authored: null,

  get hasAlpha() {
    if (!this.valid) {
      return false;
    }
    return this._getRGBATuple().a !== 1;
  },

  get valid() {
    return DOMUtils.isValidCSSColor(this.authored);
  },

  /**
   * Return true for all transparent values e.g. rgba(0, 0, 0, 0).
   */
  get transparent() {
    try {
      let tuple = this._getRGBATuple();
      return !(tuple.r || tuple.g || tuple.b || tuple.a);
    } catch(e) {
      return false;
    }
  },

  get specialValue() {
    return SPECIALVALUES.has(this.authored) ? this.authored : null;
  },

  get name() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }

    try {
      let tuple = this._getRGBATuple();

      if (tuple.a !== 1) {
        return this.rgb;
      }
      let {r, g, b} = tuple;
      return DOMUtils.rgbToColorName(r, g, b);
    } catch(e) {
      return this.hex;
    }
  },

  get hex() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (this.hasAlpha) {
      return this.rgba;
    }

    let hex = this.longHex;
    if (hex.charAt(1) == hex.charAt(2) &&
        hex.charAt(3) == hex.charAt(4) &&
        hex.charAt(5) == hex.charAt(6)) {
      hex = "#" + hex.charAt(1) + hex.charAt(3) + hex.charAt(5);
    }
    return hex;
  },

  get longHex() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (this.hasAlpha) {
      return this.rgba;
    }
    return this.rgb.replace(/\brgb\((\d{1,3}),\s*(\d{1,3}),\s*(\d{1,3})\)/gi, function(_, r, g, b) {
      return "#" + ((1 << 24) + (r << 16) + (g << 8) + (b << 0)).toString(16).substr(-6).toUpperCase();
    });
  },

  get rgb() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (!this.hasAlpha) {
      if (this.authored.startsWith("rgb(")) {
        // The color is valid and begins with rgb(. Return the authored value.
        return this.authored;
      }
      let tuple = this._getRGBATuple();
      return "rgb(" + tuple.r + ", " + tuple.g + ", " + tuple.b + ")";
    }
    return this.rgba;
  },

  get rgba() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (this.authored.startsWith("rgba(")) {
      // The color is valid and begins with rgba(. Return the authored value.
        return this.authored;
    }
    let components = this._getRGBATuple();
    return "rgba(" + components.r + ", " +
                     components.g + ", " +
                     components.b + ", " +
                     components.a + ")";
  },

  get hsl() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (this.authored.startsWith("hsl(")) {
      // The color is valid and begins with hsl(. Return the authored value.
      return this.authored;
    }
    if (this.hasAlpha) {
      return this.hsla;
    }
    return this._hslNoAlpha();
  },

  get hsla() {
    let invalidOrSpecialValue = this._getInvalidOrSpecialValue();
    if (invalidOrSpecialValue !== false) {
      return invalidOrSpecialValue;
    }
    if (this.authored.startsWith("hsla(")) {
      // The color is valid and begins with hsla(. Return the authored value.
      return this.authored;
    }
    if (this.hasAlpha) {
      let a = this._getRGBATuple().a;
      return this._hslNoAlpha().replace("hsl", "hsla").replace(")", ", " + a + ")");
    }
    return this._hslNoAlpha().replace("hsl", "hsla").replace(")", ", 1)");
  },

  /**
   * Check whether the current color value is in the special list e.g.
   * transparent or invalid.
   *
   * @return {String|Boolean}
   *         - If the current color is a special value e.g. "transparent" then
   *           return the color.
   *         - If the color is invalid return an empty string.
   *         - If the color is a regular color e.g. #F06 so we return false
   *           to indicate that the color is neither invalid or special.
   */
  _getInvalidOrSpecialValue: function() {
    if (this.specialValue) {
      return this.specialValue;
    }
    if (!this.valid) {
      return "";
    }
    return false;
  },

  /**
   * Change color
   *
   * @param  {String} color
   *         Any valid color string
   */
  newColor: function(color) {
    this.authored = color.toLowerCase();
    return this;
  },

  /**
   * Return a string representing a color of type defined in COLOR_UNIT_PREF.
   */
  toString: function() {
    let color;
    let defaultUnit = Services.prefs.getCharPref(COLOR_UNIT_PREF);
    let unit = CssColor.COLORUNIT[defaultUnit];

    switch(unit) {
      case CssColor.COLORUNIT.authored:
        color = this.authored;
        break;
      case CssColor.COLORUNIT.hex:
        color = this.hex;
        break;
      case CssColor.COLORUNIT.hsl:
        color = this.hsl;
        break;
      case CssColor.COLORUNIT.name:
        color = this.name;
        break;
      case CssColor.COLORUNIT.rgb:
        color = this.rgb;
        break;
      default:
        color = this.rgb;
    }
    return color;
  },

  /**
   * Returns a RGBA 4-Tuple representation of a color or transparent as
   * appropriate.
   */
  _getRGBATuple: function() {
    let tuple = DOMUtils.colorToRGBA(this.authored);

    tuple.a = parseFloat(tuple.a.toFixed(1));

    return tuple;
  },

  _hslNoAlpha: function() {
    let {r, g, b} = this._getRGBATuple();

    if (this.authored.startsWith("hsl(")) {
      // We perform string manipulations on our output so let's ensure that it
      // is formatted as we expect.
      let [, h, s, l] = this.authored.match(REGEX_HSL_3_TUPLE);
      return "hsl(" + h + ", " + s + ", " + l + ")";
    }

    let [h,s,l] = rgbToHsl([r,g,b]);

    return "hsl(" + h + ", " + s + "%, " + l + "%)";
  },

  /**
   * This method allows comparison of CssColor objects using ===.
   */
  valueOf: function() {
    return this.rgba;
  },
};

/**
 * Process a CSS string
 *
 * @param  {String} value
 *         CSS string e.g. "color:red; background-color:green;"
 * @return {String}
 *         Converted CSS String e.g. "color:#F00; background-color:#0F0;"
 */
function processCSSString(value) {
  if (value && REGEX_JUST_QUOTES.test(value)) {
    return value;
  }

  let colorPattern = REGEX_ALL_COLORS;

  value = value.replace(colorPattern, function(match) {
    let color = new CssColor(match);
    if (color.valid) {
      return color;
    }
    return match;
  });
  return value;
}

/**
 * Convert rgb value to hsl
 *
 * @param {array} rgb
 *         Array of rgb values
 * @return {array}
 *         Array of hsl values.
 */
function rgbToHsl([r,g,b]) {
  r = r / 255;
  g = g / 255;
  b = b / 255;

  let max = Math.max(r, g, b);
  let min = Math.min(r, g, b);
  let h;
  let s;
  let l = (max + min) / 2;

  if(max == min){
    h = s = 0;
  } else {
    let d = max - min;
    s = l > 0.5 ? d / (2 - max - min) : d / (max + min);

    switch(max) {
      case r:
        h = ((g - b) / d) % 6;
        break;
      case g:
        h = (b - r) / d + 2;
        break;
      case b:
        h = (r - g) / d + 4;
        break;
    }
    h *= 60;
    if (h < 0) {
      h += 360;
    }
  }

  return [Math.round(h), Math.round(s * 100), Math.round(l * 100)];
}

/**
 * Takes a color value of any type (hex, hsl, hsla, rgb, rgba)
 * and an alpha value to generate an rgba string with the correct
 * alpha value.
 *
 * @param  {String} colorValue
 *         Color in the form of hex, hsl, hsla, rgb, rgba.
 * @param  {Number} alpha
 *         Alpha value for the color, between 0 and 1.
 * @return {String}
 *         Converted color with `alpha` value in rgba form.
 */
function setAlpha(colorValue, alpha) {
  let color = new CssColor(colorValue);

  // Throw if the color supplied is not valid.
  if (!color.valid) {
    throw new Error("Invalid color.");
  }

  // If an invalid alpha valid, just set to 1.
  if (!(alpha >= 0 && alpha <= 1)) {
    alpha = 1;
  }

  let { r, g, b } = color._getRGBATuple();
  return "rgba(" + r + ", " + g + ", " + b + ", " + alpha + ")";
}

loader.lazyGetter(this, "DOMUtils", function () {
  return Cc["@mozilla.org/inspector/dom-utils;1"].getService(Ci.inIDOMUtils);
});
