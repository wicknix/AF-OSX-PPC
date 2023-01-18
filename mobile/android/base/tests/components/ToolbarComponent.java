/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

package org.mozilla.gecko.tests.components;

import static org.mozilla.gecko.tests.helpers.AssertionHelper.fAssertEquals;
import static org.mozilla.gecko.tests.helpers.AssertionHelper.fAssertFalse;
import static org.mozilla.gecko.tests.helpers.AssertionHelper.fAssertNotNull;
import static org.mozilla.gecko.tests.helpers.AssertionHelper.fAssertTrue;

import org.mozilla.gecko.NewTabletUI;
import org.mozilla.gecko.R;
import org.mozilla.gecko.tests.StringHelper;
import org.mozilla.gecko.tests.UITestContext;
import org.mozilla.gecko.tests.helpers.DeviceHelper;
import org.mozilla.gecko.tests.helpers.NavigationHelper;
import org.mozilla.gecko.tests.helpers.WaitHelper;

import android.view.View;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.TextView;

import com.jayway.android.robotium.solo.Condition;
import com.jayway.android.robotium.solo.Solo;

/**
 * A class representing any interactions that take place on the Toolbar.
 */
public class ToolbarComponent extends BaseComponent {

    private static final String URL_HTTP_PREFIX = "http://";

    public ToolbarComponent(final UITestContext testContext) {
        super(testContext);
    }

    public ToolbarComponent assertIsEditing() {
        fAssertTrue("The toolbar is in the editing state", isEditing());
        return this;
    }

    public ToolbarComponent assertIsNotEditing() {
        fAssertFalse("The toolbar is not in the editing state", isEditing());
        return this;
    }

    public ToolbarComponent assertTitle(final String url) {
        fAssertNotNull("The url argument is not null", url);

        final String expected;
        final String absoluteURL = NavigationHelper.adjustUrl(url);
        if (StringHelper.ABOUT_HOME_URL.equals(absoluteURL)) {
            expected = StringHelper.ABOUT_HOME_TITLE;
        } else if (absoluteURL.startsWith(URL_HTTP_PREFIX)) {
            expected = absoluteURL.substring(URL_HTTP_PREFIX.length());
        } else {
            expected = absoluteURL;
        }

        fAssertEquals("The Toolbar title is " + expected, expected, getTitle());
        return this;
    }

    public ToolbarComponent assertUrl(final String expected) {
        assertIsEditing();
        fAssertEquals("The Toolbar url is " + expected, expected, getUrlEditText().getText());
        return this;
    }

    public ToolbarComponent assertIsUrlEditTextSelected() {
        fAssertTrue("The edit text is selected", isUrlEditTextSelected());
        return this;
    }

    public ToolbarComponent assertIsUrlEditTextNotSelected() {
        fAssertFalse("The edit text is not selected", isUrlEditTextSelected());
        return this;
    }

    /**
     * Returns the root View for the browser toolbar.
     */
    private View getToolbarView() {
        mSolo.waitForView(R.id.browser_toolbar);
        return mSolo.getView(R.id.browser_toolbar);
    }

    private EditText getUrlEditText() {
        return (EditText) getToolbarView().findViewById(R.id.url_edit_text);
    }

    private View getUrlDisplayLayout() {
        return getToolbarView().findViewById(R.id.display_layout);
    }

    private TextView getUrlTitleText() {
        return (TextView) getToolbarView().findViewById(R.id.url_bar_title);
    }

    private ImageButton getBackButton() {
        DeviceHelper.assertIsTablet();
        return (ImageButton) getToolbarView().findViewById(R.id.back);
    }

    private ImageButton getForwardButton() {
        DeviceHelper.assertIsTablet();
        return (ImageButton) getToolbarView().findViewById(R.id.forward);
    }

    private ImageButton getReloadButton() {
        DeviceHelper.assertIsTablet();
        return (ImageButton) getToolbarView().findViewById(R.id.reload);
    }
    /**
     * Returns the View for the edit cancel button in the browser toolbar.
     */
    private View getEditCancelButton() {
        return getToolbarView().findViewById(R.id.edit_cancel);
    }

    private String getTitle() {
        return getTitleHelper(true);
    }

    /**
     * Returns the title of the page. Note that this makes no assertions to Toolbar state and
     * may return a value that may never be visible to the user. Callers likely want to use
     * {@link assertTitle} instead.
     */
    public String getPotentiallyInconsistentTitle() {
        return getTitleHelper(false);
    }

    private String getTitleHelper(final boolean shouldAssertNotEditing) {
        if (shouldAssertNotEditing) {
            assertIsNotEditing();
        }

        return getUrlTitleText().getText().toString();
    }

    private boolean isEditing() {
        return getUrlDisplayLayout().getVisibility() != View.VISIBLE &&
                getUrlEditText().getVisibility() == View.VISIBLE;
    }

    public ToolbarComponent enterEditingMode() {
        assertIsNotEditing();

        mSolo.clickOnView(getUrlTitleText(), true);

        waitForEditing();
        WaitHelper.waitFor("UrlEditText to be input method target", new Condition() {
            @Override
            public boolean isSatisfied() {
                return getUrlEditText().isInputMethodTarget();
            }
        });

        return this;
    }

    public ToolbarComponent commitEditingMode() {
        assertIsEditing();

        WaitHelper.waitForPageLoad(new Runnable() {
            @Override
            public void run() {
                mSolo.sendKey(Solo.ENTER);
            }
        });
        waitForNotEditing();

        return this;
    }

    public ToolbarComponent dismissEditingMode() {
        assertIsEditing();

        if (DeviceHelper.isTablet()) {
            final EditText urlEditText = getUrlEditText();
            if (urlEditText.isFocused()) {
                mSolo.goBack();
            }
            mSolo.goBack();
        } else {
            mSolo.clickOnView(getEditCancelButton());
        }

        waitForNotEditing();

        return this;
    }

    public ToolbarComponent enterUrl(final String url) {
        fAssertNotNull("url is not null", url);

        assertIsEditing();

        final EditText urlEditText = getUrlEditText();
        fAssertTrue("The UrlEditText is the input method target",
                urlEditText.isInputMethodTarget());

        mSolo.clearEditText(urlEditText);
        mSolo.typeText(urlEditText, url);

        return this;
    }

    public ToolbarComponent pressBackButton() {
        final ImageButton backButton = getBackButton();
        return pressButton(backButton, "back");
    }

    public ToolbarComponent pressForwardButton() {
        final ImageButton forwardButton = getForwardButton();
        return pressButton(forwardButton, "forward");
    }

    public ToolbarComponent pressReloadButton() {
        final ImageButton reloadButton = getReloadButton();
        return pressButton(reloadButton, "reload");
    }

    private ToolbarComponent pressButton(final View view, final String buttonName) {
        fAssertNotNull("The " + buttonName + " button View is not null", view);
        fAssertTrue("The " + buttonName + " button is enabled", view.isEnabled());
        fAssertEquals("The " + buttonName + " button is visible",
                View.VISIBLE, view.getVisibility());
        assertIsNotEditing();

        WaitHelper.waitForPageLoad(new Runnable() {
            @Override
            public void run() {
                mSolo.clickOnView(view);
            }
        });

        return this;
    }

    private void waitForEditing() {
        WaitHelper.waitFor("Toolbar to enter editing mode", new Condition() {
            @Override
            public boolean isSatisfied() {
                return isEditing();
            }
        });
    }

    private void waitForNotEditing() {
        WaitHelper.waitFor("Toolbar to exit editing mode", new Condition() {
            @Override
            public boolean isSatisfied() {
                return !isEditing();
            }
        });
    }

    private boolean isUrlEditTextSelected() {
        return getUrlEditText().isSelected();
    }
}
