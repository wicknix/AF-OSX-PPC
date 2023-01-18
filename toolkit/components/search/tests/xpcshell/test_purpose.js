/* Any copyright is dedicated to the Public Domain.
 *    http://creativecommons.org/publicdomain/zero/1.0/ */

/*
 * Test that a search purpose can be specified and that query parameters for
 * that purpose are included in the search URL.
 */

"use strict";

function run_test() {
  removeMetadata();
  updateAppInfo();
  do_load_manifest("data/chrome.manifest");
  useHttpServer();

  run_next_test();
}

add_task(function* test_purpose() {
  let [engine] = yield addTestEngines([
    { name: "Test search engine", xmlFileName: "engine.xml" },
  ]);

  function check_submission(aExpected, aSearchTerm, aType, aPurpose) {
    do_check_eq(engine.getSubmission(aSearchTerm, aType, aPurpose).uri.spec,
                base + aExpected);
  }

  let base = "http://www.google.com/search?q=foo&ie=utf-8&oe=utf-8&aq=t&client=firefox";
  check_submission("",              "foo");
  check_submission("",              "foo", null);
  check_submission("",              "foo", "text/html");
  check_submission("&channel=rcs",  "foo", null,        "contextmenu");
  check_submission("&channel=rcs",  "foo", "text/html", "contextmenu");
  check_submission("&channel=fflb", "foo", null,        "keyword");
  check_submission("&channel=fflb", "foo", "text/html", "keyword");
  check_submission("",              "foo", "text/html", "invalid");

  // Tests for a param that varies with a purpose but has a default value.
  base = "http://www.google.com/search?q=foo&client=firefox";
  check_submission("&channel=none", "foo", "application/x-moz-default-purpose");
  check_submission("&channel=none", "foo", "application/x-moz-default-purpose", null);
  check_submission("&channel=none", "foo", "application/x-moz-default-purpose", "");
  check_submission("&channel=rcs",  "foo", "application/x-moz-default-purpose", "contextmenu");
  check_submission("&channel=fflb", "foo", "application/x-moz-default-purpose", "keyword");
  check_submission("",              "foo", "application/x-moz-default-purpose", "invalid");

  do_test_finished();
});
