/* Any copyright is dedicated to the Public Domain.
   http://creativecommons.org/publicdomain/zero/1.0/ */

var gClient;
var gMgr;
var gRefMemory;

function run_test() {
  gMgr = Cc["@mozilla.org/memory-reporter-manager;1"].getService(Ci.nsIMemoryReporterManager);
  Cu.forceGC();
  gRefMemory = gMgr.residentUnique;

  add_test(init_server);
  add_test(add_browser_actors);
  add_test(connect_client);
  add_test(list_tabs);
  add_test(close_client);
  run_next_test();
}

function check_footprint(step, max) {
  var footprint = (gMgr.residentUnique - gRefMemory) / 1024;
  ok(footprint < max, "Footprint after " + step + " is " + footprint + " kB (should be less than " + max + " kB).");
  run_next_test();
}

function init_server() {
  DebuggerServer.init();
  check_footprint("DebuggerServer.init()", 500);
}

function add_browser_actors() {
  DebuggerServer.addBrowserActors();
  check_footprint("DebuggerServer.addBrowserActors()", 12000);
}

function connect_client() {
  gClient = new DebuggerClient(DebuggerServer.connectPipe());
  gClient.connect(function onConnect() {
    check_footprint("DebuggerClient.connect()", 12500);
  });
}

function list_tabs() {
  gClient.listTabs(function onListTabs(aResponse) {
    check_footprint("DebuggerClient.listTabs()", 13000);
  });
}

function close_client() {
  gClient.close(run_next_test);
}
