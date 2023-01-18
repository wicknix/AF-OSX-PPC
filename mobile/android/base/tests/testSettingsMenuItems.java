package org.mozilla.gecko.tests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.mozilla.gecko.Actions;
import org.mozilla.gecko.AppConstants;
import org.mozilla.gecko.util.HardwareUtils;

/** This patch tests the Sections present in the Settings Menu and the
 *  default values for them
 */
public class testSettingsMenuItems extends PixelTest {
    /**
     * The following String[][] (arrays) match the menu hierarchy for each section.
     * Each String[] (array) represents the menu items/choices in the following order:
     *
     * itemTitle { defaultValue [options] }
     *
     * where defaultValue is optional, and there can be multiple options.
     *
     * These menu items are the ones that are always present - to test menu items that differ
     * based on build (e.g., release vs. nightly), add the items in <code>addConditionalSettings</code>.
     */

    // Customize menu items.
    String[] PATH_CUSTOMIZE = { StringHelper.CUSTOMIZE_SECTION_LABEL };
    String[][] OPTIONS_CUSTOMIZE = {
        { "Home" },
        { "Search", "", "Show search suggestions", "Installed search engines"},
        { StringHelper.TABS_LABEL, "Don't restore after quitting " + StringHelper.BRAND_NAME, "Always restore", "Don't restore after quitting " + StringHelper.BRAND_NAME },
        { StringHelper.IMPORT_FROM_ANDROID_LABEL, "", "Bookmarks", "History", "Import" },
    };

    // Home panel menu items.
    String[] PATH_HOME = { StringHelper.CUSTOMIZE_SECTION_LABEL, "Home" };
    String[][] OPTIONS_HOME = {
      { "Panels" },
      { "Automatic updates", "Enabled", "Enabled", "Only over Wi-Fi" },
    };

    // Display menu items.
    String[] PATH_DISPLAY = { StringHelper.DISPLAY_SECTION_LABEL };
    final String[] TITLE_BAR_LABEL_ARR = { StringHelper.TITLE_BAR_LABEL, StringHelper.SHOW_PAGE_ADDRESS_LABEL,
        StringHelper.SHOW_PAGE_TITLE_LABEL, StringHelper.SHOW_PAGE_ADDRESS_LABEL };
    String[][] OPTIONS_DISPLAY = {
        { StringHelper.TEXT_SIZE_LABEL },
        TITLE_BAR_LABEL_ARR,
        { StringHelper.SCROLL_TITLE_BAR_LABEL, "Hide the " + StringHelper.BRAND_NAME + " title bar when scrolling down a page" },
        { "Advanced" },
        { StringHelper.CHARACTER_ENCODING_LABEL, "Don't show menu", "Show menu", "Don't show menu" },
        { StringHelper.PLUGINS_LABEL, "Tap to play", "Enabled", "Tap to play", "Disabled" },
    };

    // Privacy menu items.
    String[] PATH_PRIVACY = { StringHelper.PRIVACY_SECTION_LABEL };
    final String[] TRACKING_PROTECTION_LABEL_ARR = { StringHelper.TRACKING_PROTECTION_LABEL };
    String[][] OPTIONS_PRIVACY = {
        TRACKING_PROTECTION_LABEL_ARR,
        { StringHelper.DNT_LABEL },
        { StringHelper.COOKIES_LABEL, "Enabled", "Enabled, excluding 3rd party", "Disabled" },
        { StringHelper.REMEMBER_PASSWORDS_LABEL },
        { StringHelper.MASTER_PASSWORD_LABEL },
        { StringHelper.CLEAR_PRIVATE_DATA_LABEL, "", "Browsing history", "Downloads", "Form & search history", "Cookies & active logins", "Saved passwords", "Cache", "Offline website data", "Site settings", "Clear data" },
    };

    // Mozilla/vendor menu items.
    String[] PATH_MOZILLA = { StringHelper.MOZILLA_SECTION_LABEL };
    String[][] OPTIONS_MOZILLA = {
        { StringHelper.ABOUT_LABEL },
        { StringHelper.FAQS_LABEL },
        { StringHelper.FEEDBACK_LABEL },
        { "Data choices" },
        { StringHelper.HEALTH_REPORT_LABEL, "Shares data with Mozilla about your browser health and helps you understand your browser performance" },
        { StringHelper.MY_HEALTH_REPORT_LABEL },
    };

    // Developer menu items.
    String[] PATH_DEVELOPER = { StringHelper.DEVELOPER_TOOLS_SECTION_LABEL };
    String[][] OPTIONS_DEVELOPER = {
        { StringHelper.PAINT_FLASHING_LABEL },
        { StringHelper.REMOTE_DEBUGGING_LABEL },
        { StringHelper.LEARN_MORE_LABEL },
    };

    /*
     * This sets up a hierarchy of settings to test.
     *
     * The keys are String arrays representing the path through menu items
     * (the single-item arrays being top-level categories), and each value
     * is a List of menu items contained within each category.
     *
     * Each menu item is itself an array as follows:
     *  - item title
     *  - default string value of item (optional)
     *  - string values of options that are displayed once clicked (optional).
     */
    public void setupSettingsMap(Map<String[], List<String[]>> settingsMap) {
        settingsMap.put(PATH_CUSTOMIZE, new ArrayList<String[]>(Arrays.asList(OPTIONS_CUSTOMIZE)));
        settingsMap.put(PATH_HOME, new ArrayList<String[]>(Arrays.asList(OPTIONS_HOME)));
        settingsMap.put(PATH_DISPLAY, new ArrayList<String[]>(Arrays.asList(OPTIONS_DISPLAY)));
        settingsMap.put(PATH_PRIVACY, new ArrayList<String[]>(Arrays.asList(OPTIONS_PRIVACY)));
        settingsMap.put(PATH_MOZILLA, new ArrayList<String[]>(Arrays.asList(OPTIONS_MOZILLA)));
        settingsMap.put(PATH_DEVELOPER, new ArrayList<String[]>(Arrays.asList(OPTIONS_DEVELOPER)));
    }

    public void testSettingsMenuItems() {
        blockForGeckoReady();

        Map<String[], List<String[]>> settingsMenuItems = new HashMap<String[], List<String[]>>();
        setupSettingsMap(settingsMenuItems);

        // Set special handling for Settings items that are conditionally built.
        updateConditionalSettings(settingsMenuItems);

        selectMenuItem(StringHelper.SETTINGS_LABEL);
        mAsserter.ok(mSolo.waitForText(StringHelper.SETTINGS_LABEL),
                "The Settings menu did not load", StringHelper.SETTINGS_LABEL);

        // Dismiss the Settings screen and verify that the view is returned to about:home page
        mActions.sendSpecialKey(Actions.SpecialKey.BACK);

        // Waiting for page title to appear to be sure that is fully loaded before opening the menu
        mAsserter.ok(mSolo.waitForText(StringHelper.TITLE_PLACE_HOLDER), "about:home did not load",
                StringHelper.TITLE_PLACE_HOLDER);
        verifyUrl(StringHelper.ABOUT_HOME_URL);

        selectMenuItem(StringHelper.SETTINGS_LABEL);
        mAsserter.ok(mSolo.waitForText(StringHelper.SETTINGS_LABEL),
                "The Settings menu did not load", StringHelper.SETTINGS_LABEL);

        checkForSync(mDevice);

        checkMenuHierarchy(settingsMenuItems);
    }

    /**
     * Check for Sync in settings.
     *
     * Sync location is a top level menu item on phones and small tablets,
     * but is under "Customize" on large tablets.
     */
    public void checkForSync(Device device) {
        mAsserter.ok(mSolo.waitForText(StringHelper.SYNC_LABEL), "Waiting for Sync option",
                StringHelper.SYNC_LABEL);
    }

    /**
     * Check for conditions for building certain settings, and add them to be tested
     * if they are present.
     */
    public void updateConditionalSettings(Map<String[], List<String[]>> settingsMap) {
        // Preferences dependent on RELEASE_BUILD
        if (!AppConstants.RELEASE_BUILD) {
            // Text reflow - only built if *not* release build
            String[] textReflowUi = { StringHelper.TEXT_REFLOW_LABEL };
            settingsMap.get(PATH_DISPLAY).add(textReflowUi);

            if (AppConstants.MOZ_STUMBLER_BUILD_TIME_ENABLED) {
                // Anonymous cell tower/wifi collection
                String[] networkReportingUi = { "Mozilla Location Service", "Help Mozilla map the world! Share approximate Wi-Fi and cellular location of your device to improve our geolocation service" };
                settingsMap.get(PATH_MOZILLA).add(networkReportingUi);

                String[] learnMoreUi = { "Learn more" };
                settingsMap.get(PATH_MOZILLA).add(learnMoreUi);
            }
        }

        if (!AppConstants.NIGHTLY_BUILD) {
            settingsMap.get(PATH_PRIVACY).remove(TRACKING_PROTECTION_LABEL_ARR);
        }

        // Automatic updates
        if (AppConstants.MOZ_UPDATER) {
            String[] autoUpdateUi = { "Download updates automatically", "Only over Wi-Fi", "Always", "Only over Wi-Fi", "Never" };
            settingsMap.get(PATH_CUSTOMIZE).add(autoUpdateUi);
        }

        // Crash reporter
        if (AppConstants.MOZ_CRASHREPORTER) {
            String[] crashReporterUi = { "Crash Reporter", StringHelper.BRAND_NAME + " submits crash reports to help Mozilla make your browser more stable and secure" };
            settingsMap.get(PATH_MOZILLA).add(crashReporterUi);
        }

        // Telemetry
        if (AppConstants.MOZ_TELEMETRY_REPORTING) {
            String[] telemetryUi = { "Telemetry", "Shares performance, usage, hardware and customization data about your browser with Mozilla to help us make " + StringHelper.BRAND_NAME + " better" };
            settingsMap.get(PATH_MOZILLA).add(telemetryUi);
        }

        // Tablet: we don't allow a page title option.
        if (HardwareUtils.isTablet()) {
            settingsMap.get(PATH_DISPLAY).remove(TITLE_BAR_LABEL_ARR);
        }
    }

    public void checkMenuHierarchy(Map<String[], List<String[]>> settingsMap) {
        // Check the items within each category.
        String section = null;
        for (Entry<String[], List<String[]>> e : settingsMap.entrySet()) {
            final String[] menuPath = e.getKey();

            for (String menuItem : menuPath) {
                section = "^" + menuItem + "$";

                waitForEnabledText(section);
                mSolo.clickOnText(section);
            }

            List<String[]> sectionItems = e.getValue();

            // Check each item of the section.
            for (String[] item : sectionItems) {
                int itemLen = item.length;

                // Each item must at least have a title.
                mAsserter.ok(item.length > 0, "Section-item", "Each item must at least have a title");

                // Check item title.
                String itemTitle = "^" + item[0] + "$";
                boolean foundText = waitForPreferencesText(itemTitle);

                mAsserter.ok(foundText, "Waiting for settings item " + itemTitle + " in section " + section,
                             "The " + itemTitle + " option is present in section " + section);
                // Check item default, if it exists.
                if (itemLen > 1) {
                    String itemDefault = "^" + item[1] + "$";
                    foundText = waitForPreferencesText(itemDefault);
                    mAsserter.ok(foundText, "Waiting for settings item default " + itemDefault
                                 + " in section " + section,
                                 "The " + itemDefault + " default is present in section " + section);
                }
                // Check item choices, if they exist.
                if (itemLen > 2) {
                    waitForEnabledText(itemTitle);
                    mSolo.clickOnText(itemTitle);
                    for (int i = 2; i < itemLen; i++) {
                        String itemChoice = "^" + item[i] + "$";
                        foundText = waitForPreferencesText(itemChoice);
                        mAsserter.ok(foundText, "Waiting for settings item choice " + itemChoice
                                     + " in section " + section,
                                     "The " + itemChoice + " choice is present in section " + section);
                    }

                    // Leave submenu after checking.
                    if (waitForText("^Cancel$")) {
                        mSolo.clickOnText("^Cancel$");
                    } else {
                        // Some submenus aren't dialogs, but are nested screens; exit using "back".
                        mActions.sendSpecialKey(Actions.SpecialKey.BACK);
                    }
                }
            }

            // Navigate back if on a phone. Tablets shouldn't do this because they use headers and fragments.
            if (mDevice.type.equals("phone")) {
                int menuDepth = menuPath.length;
                while (menuDepth > 0) {
                    mActions.sendSpecialKey(Actions.SpecialKey.BACK);
                    menuDepth--;
                    // Sleep so subsequent back actions aren't lost.
                    mSolo.sleep(150);
                }
            }
        }
    }
}
