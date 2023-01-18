/* Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/ */

/**
 * Appends a div to the document body.
 *
 * @param t  The testharness.js Test object. If provided, this will be used
 *           to register a cleanup callback to remove the div when the test
 *           finishes.
 */
function addDiv(t) {
  var div = document.createElement('div');
  document.body.appendChild(div);
  if (t && typeof t.add_cleanup === 'function') {
    t.add_cleanup(function() { div.remove(); });
  }
  return div;
}

/**
 * Promise wrapper for requestAnimationFrame.
 */
function waitForFrame() {
  return new Promise(function(resolve, reject) {
    window.requestAnimationFrame(resolve);
  });
}

/**
 * Wrapper that takes a sequence of N players and returns:
 *
 *   Promise.all([players[0].ready, players[1].ready, ... players[N-1].ready]);
 */
function waitForAllPlayers(players) {
  return Promise.all(players.map(function(player) { return player.ready; }));
}
