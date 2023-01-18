// For now this test only calls update to verify that our registration
// job queueing works properly when called from the worker thread. We should
// test actual update scenarios with a SJS test.
onmessage = function(e) {
  self.update();
  clients.getServiced().then(function(c) {
    if (c.length == 0) {
      // We cannot proceed.
      return;
    }

    c[0].postMessage('FINISH');
  });
}
