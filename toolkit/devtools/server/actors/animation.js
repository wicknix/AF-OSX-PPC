/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

"use strict";

/**
 * Set of actors that expose the Web Animations API to devtools protocol clients.
 *
 * The |Animations| actor is the main entry point. It is used to discover
 * animation players on given nodes.
 * There should only be one instance per debugger server.
 *
 * The |AnimationPlayer| actor provides attributes and methods to inspect an
 * animation as well as pause/resume/seek it.
 *
 * The Web Animation spec implementation is ongoing in Gecko, and so this set
 * of actors should evolve when the implementation progresses.
 *
 * References:
 * - WebAnimation spec:
 *   http://w3c.github.io/web-animations/
 * - WebAnimation WebIDL files:
 *   /dom/webidl/Animation*.webidl
 */

const {Cu} = require("chrome");
const {Promise: promise} = Cu.import("resource://gre/modules/Promise.jsm", {});
const {Task} = Cu.import("resource://gre/modules/Task.jsm", {});
const {setInterval, clearInterval} = require("sdk/timers");
const protocol = require("devtools/server/protocol");
const {ActorClass, Actor, FrontClass, Front, Arg, method, RetVal} = protocol;
const {NodeActor} = require("devtools/server/actors/inspector");
const EventEmitter = require("devtools/toolkit/event-emitter");
const events = require("sdk/event/core");

const PLAYER_DEFAULT_AUTO_REFRESH_TIMEOUT = 500; // ms

/**
 * The AnimationPlayerActor provides information about a given animation: its
 * startTime, currentTime, current state, etc.
 *
 * Since the state of a player changes as the animation progresses it is often
 * useful to call getCurrentState at regular intervals to get the current state.
 *
 * This actor also allows playing and pausing the animation.
 */
let AnimationPlayerActor = ActorClass({
  typeName: "animationplayer",

  /**
   * @param {AnimationsActor} The main AnimationsActor instance
   * @param {AnimationPlayer} The player object returned by getAnimationPlayers
   * @param {DOMNode} The node targeted by this player
   * @param {Number} Temporary work-around used to retrieve duration and
   * iteration count from computed-style rather than from waapi. This is needed
   * to know which duration to get, in case there are multiple css animations
   * applied to the same node.
   */
  initialize: function(animationsActor, player, node, playerIndex) {
    Actor.prototype.initialize.call(this, animationsActor.conn);

    this.player = player;
    this.node = node;
    this.playerIndex = playerIndex;
    this.styles = node.ownerDocument.defaultView.getComputedStyle(node);
  },

  destroy: function() {
    this.player = this.node = this.styles = null;
    Actor.prototype.destroy.call(this);
  },

  /**
   * Release the actor, when it isn't needed anymore.
   * Protocol.js uses this release method to call the destroy method.
   */
  release: method(function() {}, {release: true}),

  form: function(detail) {
    if (detail === "actorid") {
      return this.actorID;
    }

    let data = this.getCurrentState();
    data.actor = this.actorID;

    return data;
  },

  /**
   * Some of the player's properties are retrieved from the node's
   * computed-styles because the Web Animations API does not provide them yet.
   * But the computed-styles may contain multiple animations for a node and so
   * we need to know which is the index of the current animation in the style.
   * @return {Number}
   */
  getPlayerIndex: function() {
    let names = this.styles.animationName;

    // If no names are found, then it's probably a transition, in which case we
    // can't find the actual index, so just trust the playerIndex passed by
    // the AnimationsActor at initialization time.
    // Note that this may be incorrect if by the time the AnimationPlayerActor
    // is initialized, one of the transitions has ended, but it's the best we
    // can do for now.
    if (!names) {
      return this.playerIndex;
    }

    // If there's only one name.
    if (names.contains(",") === -1) {
      return 0;
    }

    // If there are several names, retrieve the index of the animation name in
    // the list.
    names = names.split(",").map(n => n.trim());
    for (let i = 0; i < names.length; i ++) {
      if (names[i] === this.player.source.effect.name) {
        return i;
      }
    }
  },

  /**
   * Get the animation duration from this player, in milliseconds.
   * Note that the Web Animations API doesn't yet offer a way to retrieve this
   * directly from the AnimationPlayer object, so for now, a duration is only
   * returned if found in the node's computed styles.
   * @return {Number}
   */
  getDuration: function() {
    let durationText;
    if (this.styles.animationDuration !== "0s") {
      durationText = this.styles.animationDuration;
    } else if (this.styles.transitionDuration !== "0s") {
      durationText = this.styles.transitionDuration;
    } else {
      return null;
    }

    // If the computed duration has multiple entries, we need to find the right
    // one.
    if (durationText.indexOf(",") !== -1) {
      durationText = durationText.split(",")[this.getPlayerIndex()];
    }

    return parseFloat(durationText) * 1000;
  },

  /**
   * Get the animation delay from this player, in milliseconds.
   * Note that the Web Animations API doesn't yet offer a way to retrieve this
   * directly from the AnimationPlayer object, so for now, a delay is only
   * returned if found in the node's computed styles.
   * @return {Number}
   */
  getDelay: function() {
    let delayText;
    if (this.styles.animationDelay !== "0s") {
      delayText = this.styles.animationDelay;
    } else if (this.styles.transitionDelay !== "0s") {
      delayText = this.styles.transitionDelay;
    } else {
      return 0;
    }

    if (delayText.indexOf(",") !== -1) {
      delayText = delayText.split(",")[this.getPlayerIndex()];
    }

    return parseFloat(delayText) * 1000;
  },

  /**
   * Get the animation iteration count for this player. That is, how many times
   * is the animation scheduled to run.
   * Note that the Web Animations API doesn't yet offer a way to retrieve this
   * directly from the AnimationPlayer object, so for now, check for
   * animationIterationCount in the node's computed styles, and return that.
   * This style property defaults to 1 anyway.
   * @return {Number}
   */
  getIterationCount: function() {
    let iterationText = this.styles.animationIterationCount;
    if (iterationText.indexOf(",") !== -1) {
      iterationText = iterationText.split(",")[this.getPlayerIndex()];
    }

    return iterationText === "infinite"
           ? null
           : parseInt(iterationText, 10);
  },

  /**
   * Get the current state of the AnimationPlayer (currentTime, playState, ...).
   * Note that the initial state is returned as the form of this actor when it
   * is initialized.
   * @return {Object}
   */
  getCurrentState: method(function() {
    // Note that if you add a new property to the state object, make sure you
    // add the corresponding property in the AnimationPlayerFront' initialState
    // getter.
    let newState = {
      // startTime is null whenever the animation is paused or waiting to start.
      startTime: this.player.startTime,
      currentTime: this.player.currentTime,
      playState: this.player.playState,
      name: this.player.source.effect.name,
      duration: this.getDuration(),
      delay: this.getDelay(),
      iterationCount: this.getIterationCount(),
      // isRunningOnCompositor is important for developers to know if their
      // animation is hitting the fast path or not. Currently only true for
      // Firefox OS (where we have compositor animations enabled).
      // Returns false whenever the animation is paused as it is taken off the
      // compositor then.
      isRunningOnCompositor: this.player.isRunningOnCompositor
    };

    // If we've saved a state before, compare and only send what has changed.
    // It's expected of the front to also save old states to re-construct the
    // full state when an incomplete one is received.
    // This is to minimize protocol traffic.
    let sentState = {};
    if (this.currentState) {
      for (let key in newState) {
        if (typeof this.currentState[key] === "undefined" ||
            this.currentState[key] !== newState[key]) {
          sentState[key] = newState[key];
        }
      }
    } else {
      sentState = newState;
    }
    this.currentState = newState;

    return sentState;
  }, {
    request: {},
    response: {
      data: RetVal("json")
    }
  }),

  /**
   * Pause the player.
   */
  pause: method(function() {
    this.player.pause();
  }, {
    request: {},
    response: {}
  }),

  /**
   * Play the player.
   * This method only returns when the animation has left its pending state.
   */
  play: method(function() {
    this.player.play();
    return this.player.ready;
  }, {
    request: {},
    response: {}
  }),

  /**
   * Simply exposes the player ready promise.
   *
   * When an animation is created/paused then played, there's a short time
   * during which its playState is pending, before being set to running.
   *
   * If you either created a new animation using the Web Animations API or
   * paused/played an existing one, and then want to access the playState, you
   * might be interested to call this method.
   * This is especially important for tests.
   */
  ready: method(function() {
    return this.player.ready;
  }, {
    request: {},
    response: {}
  })
});

let AnimationPlayerFront = FrontClass(AnimationPlayerActor, {
  AUTO_REFRESH_EVENT: "updated-state",

  initialize: function(conn, form, detail, ctx) {
    EventEmitter.decorate(this);
    Front.prototype.initialize.call(this, conn, form, detail, ctx);

    this.state = {};
  },

  form: function(form, detail) {
    if (detail === "actorid") {
      this.actorID = form;
      return;
    }
    this._form = form;
    this.state = this.initialState;
  },

  destroy: function() {
    this.stopAutoRefresh();
    Front.prototype.destroy.call(this);
  },

  /**
   * Getter for the initial state of the player. Up to date states can be
   * retrieved by calling the getCurrentState method.
   */
  get initialState() {
    return {
      startTime: this._form.startTime,
      currentTime: this._form.currentTime,
      playState: this._form.playState,
      name: this._form.name,
      duration: this._form.duration,
      delay: this._form.delay,
      iterationCount: this._form.iterationCount,
      isRunningOnCompositor: this._form.isRunningOnCompositor
    }
  },

  // About auto-refresh:
  //
  // The AnimationPlayerFront is capable of automatically refreshing its state
  // by calling the getCurrentState method at regular intervals. This allows
  // consumers to update their knowledge of the player's currentTime, playState,
  // ... dynamically.
  //
  // Calling startAutoRefresh will start the automatic refreshing of the state,
  // and calling stopAutoRefresh will stop it.
  // Once the automatic refresh has been started, the AnimationPlayerFront emits
  // "updated-state" events everytime the state changes.
  //
  // Note that given the time-related nature of animations, the actual state
  // changes a lot more often than "updated-state" events are emitted. This is
  // to avoid making many protocol requests.

  /**
   * Start auto-refreshing this player's state.
   * @param {Number} interval Optional auto-refresh timer interval to override
   * the default value.
   */
  startAutoRefresh: function(interval=PLAYER_DEFAULT_AUTO_REFRESH_TIMEOUT) {
    if (this.autoRefreshTimer) {
      return;
    }

    this.autoRefreshTimer = setInterval(this.refreshState.bind(this), interval);
  },

  /**
   * Stop auto-refreshing this player's state.
   */
  stopAutoRefresh: function() {
    if (!this.autoRefreshTimer) {
      return;
    }

    clearInterval(this.autoRefreshTimer);
    this.autoRefreshTimer = null;
  },

  /**
   * Called automatically when auto-refresh is on. Doesn't return anything, but
   * emits the "updated-state" event.
   */
  refreshState: Task.async(function*() {
    let data = yield this.getCurrentState();

    // By the time the new state is received, auto-refresh might be stopped.
    if (!this.autoRefreshTimer) {
      return;
    }

    // If the animationplayer is now finished, stop auto-refreshing.
    if (data.playState === "finished") {
      this.stopAutoRefresh();
    }

    if (this.currentStateHasChanged) {
      this.state = data;
      this.emit(this.AUTO_REFRESH_EVENT, this.state);
    }
  }),

  /**
   * getCurrentState interceptor re-constructs incomplete states since the actor
   * only sends the values that have changed.
   */
  getCurrentState: protocol.custom(function() {
    this.currentStateHasChanged = false;
    return this._getCurrentState().then(data => {
      for (let key in this.state) {
        if (typeof data[key] === "undefined") {
          data[key] = this.state[key];
        } else if (data[key] !== this.state[key]) {
          this.currentStateHasChanged = true;
        }
      }
      return data;
    });
  }, {
    impl: "_getCurrentState"
  }),
});

/**
 * The Animations actor lists animation players for a given node.
 */
let AnimationsActor = exports.AnimationsActor = ActorClass({
  typeName: "animations",

  initialize: function(conn, tabActor) {
    Actor.prototype.initialize.call(this, conn);
    this.tabActor = tabActor;

    this.allAnimationsPaused = false;
    this.onNavigate = this.onNavigate.bind(this);
    events.on(this.tabActor, "navigate", this.onNavigate);
  },

  destroy: function() {
    Actor.prototype.destroy.call(this);
    events.off(this.tabActor, "navigate", this.onNavigate);
    this.tabActor = null;
  },

  /**
   * Since AnimationsActor doesn't have a protocol.js parent actor that takes
   * care of its lifetime, implementing disconnect is required to cleanup.
   */
  disconnect: function() {
    this.destroy();
  },

  /**
   * Retrieve the list of AnimationPlayerActor actors corresponding to
   * currently running animations for a given node.
   * @param {NodeActor} nodeActor The NodeActor type is defined in
   * /toolkit/devtools/server/actors/inspector
   */
  getAnimationPlayersForNode: method(function(nodeActor) {
    let players = nodeActor.rawNode.getAnimationPlayers();

    let actors = [];
    for (let i = 0; i < players.length; i ++) {
      // XXX: for now the index is passed along as the AnimationPlayerActor uses
      // it to retrieve animation information from CSS.
      actors.push(AnimationPlayerActor(this, players[i], nodeActor.rawNode, i));
    }

    return actors;
  }, {
    request: {
      actorID: Arg(0, "domnode")
    },
    response: {
      players: RetVal("array:animationplayer")
    }
  }),

  /**
   * Iterates through all nodes in all of the tabActor's window documents and
   * finds all existing animation players.
   * This is currently used to allow playing/pausing all animations at once
   * until the WebAnimations API provides a way to play/pause via the document
   * timeline (alternatively, when bug 1123524 is fixed, we will be able to
   * only iterate once and then listen for changes).
   */
  getAllAnimationPlayers: function() {
    let players = [];

    // These loops shouldn't be as bad as they look.
    // Typically, there will be very few windows, and getElementsByTagName is
    // really fast even on large DOM trees.
    for (let window of this.tabActor.windows) {
      let root = window.document.body || window.document;
      for (let element of root.getElementsByTagNameNS("*", "*")) {
        players = [...players, ...element.getAnimationPlayers()];
      }
    }

    return players;
  },

  onNavigate: function({isTopLevel}) {
    if (isTopLevel) {
      this.allAnimationsPaused = false;
    }
  },

  /**
   * Pause all animations in the current tabActor's frames.
   */
  pauseAll: method(function() {
    for (let player of this.getAllAnimationPlayers()) {
      player.pause();
    }
    this.allAnimationsPaused = true;
  }, {
    request: {},
    response: {}
  }),

  /**
   * Play all animations in the current tabActor's frames.
   * This method only returns when the animations have left their pending states.
   */
  playAll: method(function() {
    let readyPromises = [];
    for (let player of this.getAllAnimationPlayers()) {
      player.play();
      readyPromises.push(player.ready);
    }
    this.allAnimationsPaused = false;
    return promise.all(readyPromises);
  }, {
    request: {},
    response: {}
  }),

  toggleAll: method(function() {
    if (this.allAnimationsPaused) {
      return this.playAll();
    } else {
      return this.pauseAll();
    }
  }, {
    request: {},
    response: {}
  })
});

let AnimationsFront = exports.AnimationsFront = FrontClass(AnimationsActor, {
  initialize: function(client, {animationsActor}) {
    Front.prototype.initialize.call(this, client, {actor: animationsActor});
    this.manage(this);
  },

  destroy: function() {
    Front.prototype.destroy.call(this);
  }
});
