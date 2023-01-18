/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef nsButtonBoxFrame_h___
#define nsButtonBoxFrame_h___

#include "mozilla/Attributes.h"
#include "nsBoxFrame.h"

class nsButtonBoxFrame : public nsBoxFrame
{
public:
  NS_DECL_FRAMEARENA_HELPERS

  friend nsIFrame* NS_NewButtonBoxFrame(nsIPresShell* aPresShell);

  explicit nsButtonBoxFrame(nsStyleContext* aContext)
    :nsBoxFrame(aContext, false) {
    UpdateMouseThrough();
  }

  virtual void BuildDisplayListForChildren(nsDisplayListBuilder*   aBuilder,
                                           const nsRect&           aDirtyRect,
                                           const nsDisplayListSet& aLists) override;

  virtual nsresult HandleEvent(nsPresContext* aPresContext, 
                               mozilla::WidgetGUIEvent* aEvent,
                               nsEventStatus* aEventStatus) override;

  virtual void MouseClicked(nsPresContext* aPresContext,
                            mozilla::WidgetGUIEvent* aEvent)
  { DoMouseClick(aEvent, false); }

#ifdef DEBUG_FRAME_DUMP
  virtual nsresult GetFrameName(nsAString& aResult) const override {
    return MakeFrameName(NS_LITERAL_STRING("ButtonBoxFrame"), aResult);
  }
#endif

  /**
   * Our implementation of MouseClicked. 
   * @param aTrustEvent if true and aEvent as null, then assume the event was trusted
   */
  void DoMouseClick(mozilla::WidgetGUIEvent* aEvent, bool aTrustEvent);
  void UpdateMouseThrough() override { AddStateBits(NS_FRAME_MOUSE_THROUGH_NEVER); }
}; // class nsButtonBoxFrame

#endif /* nsButtonBoxFrame_h___ */
