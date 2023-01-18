/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

package org.mozilla.gecko.tests;


public class testCheck extends PixelTest {
    private void pause(int length) {
        try {
            Thread.sleep(length);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected Type getTestType() {
        return Type.TALOS;
    }

    public void testCheck() {
        String url = getAbsoluteUrl("/startup_test/fennecmark/timecube.html");

        blockForGeckoReady();

        loadAndPaint(url);

        mDriver.setupScrollHandling();

        // Setup scrolling coordinates.
        MotionEventHelper meh = new MotionEventHelper(getInstrumentation(), mDriver.getGeckoLeft(), mDriver.getGeckoTop());
        int midX = mDriver.getGeckoWidth() / 2;
        int height = mDriver.getGeckoHeight();
        int topY = height / 8;

        mDriver.startCheckerboardRecording();

        // Scroll repeatedly downwards, then upwards. On each iteration of i,
        // increase the scroll distance to test different scroll amounts.
        for (int i = 2; i < 7; i++) {
            int botY = (height * i / 8);
            for (int j = 0; j < 3; j++) {
                meh.dragSync(midX, botY, midX, topY, 200);
                pause(1000);
            }
            for (int j = 0; j < 3; j++) {
                meh.dragSync(midX, topY, midX, botY, 200);
                pause(1000);
            }
        }

        float completeness = mDriver.stopCheckerboardRecording();
        mAsserter.dumpLog("__start_report" + completeness + "__end_report");
        long msecs = System.currentTimeMillis();
        mAsserter.dumpLog("__startTimestamp" + msecs + "__endTimestamp");
    }
}
