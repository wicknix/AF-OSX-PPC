/* vim: set shiftwidth=2 tabstop=8 autoindent cindent expandtab: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "nsAnimationManager.h"
#include "nsTransitionManager.h"

#include "mozilla/EventDispatcher.h"
#include "mozilla/MemoryReporting.h"
#include "mozilla/StyleAnimationValue.h"

#include "nsPresContext.h"
#include "nsStyleSet.h"
#include "nsStyleChangeList.h"
#include "nsCSSRules.h"
#include "RestyleManager.h"
#include "nsLayoutUtils.h"
#include "nsIFrame.h"
#include "nsIDocument.h"
#include <math.h>

using namespace mozilla;
using namespace mozilla::css;
using mozilla::dom::Animation;
using mozilla::dom::AnimationPlayer;
using mozilla::CSSAnimationPlayer;

mozilla::dom::Promise*
CSSAnimationPlayer::GetReady(ErrorResult& aRv)
{
  FlushStyle();
  return AnimationPlayer::GetReady(aRv);
}

void
CSSAnimationPlayer::Play()
{
  mPauseShouldStick = false;
  AnimationPlayer::Play();
}

void
CSSAnimationPlayer::Pause()
{
  mPauseShouldStick = true;
  AnimationPlayer::Pause();
}

mozilla::dom::AnimationPlayState
CSSAnimationPlayer::PlayStateFromJS() const
{
  // Flush style to ensure that any properties controlling animation state
  // (e.g. animation-play-state) are fully updated.
  FlushStyle();
  return AnimationPlayer::PlayStateFromJS();
}

void
CSSAnimationPlayer::PlayFromJS()
{
  // Note that flushing style below might trigger calls to
  // PlayFromStyle()/PauseFromStyle() on this object.
  FlushStyle();
  AnimationPlayer::PlayFromJS();
}

void
CSSAnimationPlayer::PlayFromStyle()
{
  mIsStylePaused = false;
  if (!mPauseShouldStick) {
    DoPlay();
  }
}

void
CSSAnimationPlayer::PauseFromStyle()
{
  // Check if the pause state is being overridden
  if (mIsStylePaused) {
    return;
  }

  mIsStylePaused = true;
  DoPause();
}

void
CSSAnimationPlayer::QueueEvents(EventArray& aEventsToDispatch)
{
  if (!mSource) {
    return;
  }

  ComputedTiming computedTiming = mSource->GetComputedTiming();

  dom::Element* target;
  nsCSSPseudoElements::Type targetPseudoType;
  mSource->GetTarget(target, targetPseudoType);

  switch (computedTiming.mPhase) {
    case ComputedTiming::AnimationPhase_Null:
    case ComputedTiming::AnimationPhase_Before:
      // Do nothing
      break;

    case ComputedTiming::AnimationPhase_Active:
      // Dispatch 'animationstart' or 'animationiteration' when needed.
      if (computedTiming.mCurrentIteration != mLastNotification) {
        // Notify 'animationstart' even if a negative delay puts us
        // past the first iteration.
        // Note that when somebody changes the animation-duration
        // dynamically, this will fire an extra iteration event
        // immediately in many cases.  It's not clear to me if that's the
        // right thing to do.
        uint32_t message = mLastNotification == LAST_NOTIFICATION_NONE
                           ? NS_ANIMATION_START
                           : NS_ANIMATION_ITERATION;
        mLastNotification = computedTiming.mCurrentIteration;
        TimeDuration iterationStart =
          mSource->Timing().mIterationDuration *
          computedTiming.mCurrentIteration;
        TimeDuration elapsedTime =
          std::max(iterationStart, mSource->InitialAdvance());
        AnimationEventInfo ei(target, Name(), message,
                              StickyTimeDuration(elapsedTime),
                              PseudoTypeAsString(targetPseudoType));
        aEventsToDispatch.AppendElement(ei);
      }
      break;

    case ComputedTiming::AnimationPhase_After:
      // If we skipped the animation interval entirely, dispatch
      // 'animationstart' first
      if (mLastNotification == LAST_NOTIFICATION_NONE) {
        // Notifying for start of 0th iteration.
        // (This is overwritten below but we set it here to maintain
        // internal consistency.)
        mLastNotification = 0;
        StickyTimeDuration elapsedTime =
          std::min(StickyTimeDuration(mSource->InitialAdvance()),
                   computedTiming.mActiveDuration);
        AnimationEventInfo ei(target, Name(), NS_ANIMATION_START,
                              elapsedTime,
                              PseudoTypeAsString(targetPseudoType));
        aEventsToDispatch.AppendElement(ei);
      }
      // Dispatch 'animationend' when needed.
      if (mLastNotification != LAST_NOTIFICATION_END) {
        mLastNotification = LAST_NOTIFICATION_END;
        AnimationEventInfo ei(target, Name(), NS_ANIMATION_END,
                              computedTiming.mActiveDuration,
                              PseudoTypeAsString(targetPseudoType));
        aEventsToDispatch.AppendElement(ei);
      }
      break;
  }
}

CommonAnimationManager*
CSSAnimationPlayer::GetAnimationManager() const
{
  nsPresContext* context = GetPresContext();
  if (!context) {
    return nullptr;
  }

  return context->AnimationManager();
}

/* static */ nsString
CSSAnimationPlayer::PseudoTypeAsString(nsCSSPseudoElements::Type aPseudoType)
{
  switch (aPseudoType) {
    case nsCSSPseudoElements::ePseudo_before:
      return NS_LITERAL_STRING("::before");
    case nsCSSPseudoElements::ePseudo_after:
      return NS_LITERAL_STRING("::after");
    default:
      return EmptyString();
  }
}

void
nsAnimationManager::UpdateStyleAndEvents(AnimationPlayerCollection*
                                           aCollection,
                                         TimeStamp aRefreshTime,
                                         EnsureStyleRuleFlags aFlags)
{
  aCollection->EnsureStyleRuleFor(aRefreshTime, aFlags);
  QueueEvents(aCollection, mPendingEvents);
}

void
nsAnimationManager::QueueEvents(AnimationPlayerCollection* aCollection,
                                EventArray& aEventsToDispatch)
{
  for (size_t playerIdx = aCollection->mPlayers.Length(); playerIdx-- != 0; ) {
    CSSAnimationPlayer* player =
      aCollection->mPlayers[playerIdx]->AsCSSAnimationPlayer();
    MOZ_ASSERT(player, "Expected a collection of CSS Animation players");
    player->QueueEvents(aEventsToDispatch);
  }
}

/* virtual */ size_t
nsAnimationManager::SizeOfExcludingThis(MallocSizeOf aMallocSizeOf) const
{
  return CommonAnimationManager::SizeOfExcludingThis(aMallocSizeOf);

  // Measurement of the following members may be added later if DMD finds it is
  // worthwhile:
  // - mPendingEvents
}

/* virtual */ size_t
nsAnimationManager::SizeOfIncludingThis(MallocSizeOf aMallocSizeOf) const
{
  return aMallocSizeOf(this) + SizeOfExcludingThis(aMallocSizeOf);
}

nsIStyleRule*
nsAnimationManager::CheckAnimationRule(nsStyleContext* aStyleContext,
                                       mozilla::dom::Element* aElement)
{
  if (!mPresContext->IsDynamic()) {
    // For print or print preview, ignore animations.
    return nullptr;
  }

  // Everything that causes our animation data to change triggers a
  // style change, which in turn triggers a non-animation restyle.
  // Likewise, when we initially construct frames, we're not in a
  // style change, but also not in an animation restyle.

  const nsStyleDisplay* disp = aStyleContext->StyleDisplay();
  AnimationPlayerCollection* collection =
    GetAnimationPlayers(aElement, aStyleContext->GetPseudoType(), false);
  if (!collection &&
      disp->mAnimationNameCount == 1 &&
      disp->mAnimations[0].GetName().IsEmpty()) {
    return nullptr;
  }

  // build the animations list
  dom::AnimationTimeline* timeline = aElement->OwnerDoc()->Timeline();
  AnimationPlayerPtrArray newPlayers;
  BuildAnimations(aStyleContext, aElement, timeline, newPlayers);

  if (newPlayers.IsEmpty()) {
    if (collection) {
      collection->Destroy();
    }
    return nullptr;
  }

  if (collection) {
    collection->mStyleRule = nullptr;
    collection->mStyleRuleRefreshTime = TimeStamp();
    collection->UpdateAnimationGeneration(mPresContext);

    // Copy over the start times and (if still paused) pause starts
    // for each animation (matching on name only) that was also in the
    // old list of animations.
    // This means that we honor dynamic changes, which isn't what the
    // spec says to do, but WebKit seems to honor at least some of
    // them.  See
    // http://lists.w3.org/Archives/Public/www-style/2011Apr/0079.html
    // In order to honor what the spec said, we'd copy more data over
    // (or potentially optimize BuildAnimations to avoid rebuilding it
    // in the first place).
    if (!collection->mPlayers.IsEmpty()) {

      for (size_t newIdx = newPlayers.Length(); newIdx-- != 0;) {
        AnimationPlayer* newPlayer = newPlayers[newIdx];

        // Find the matching animation with this name in the old list
        // of animations.  We iterate through both lists in a backwards
        // direction which means that if there are more animations in
        // the new list of animations with a given name than in the old
        // list, it will be the animations towards the of the beginning of
        // the list that do not match and are treated as new animations.
        nsRefPtr<CSSAnimationPlayer> oldPlayer;
        size_t oldIdx = collection->mPlayers.Length();
        while (oldIdx-- != 0) {
          CSSAnimationPlayer* a =
            collection->mPlayers[oldIdx]->AsCSSAnimationPlayer();
          MOZ_ASSERT(a, "All players in the CSS Animation collection should"
                        " be CSSAnimationPlayer objects");
          if (a->Name() == newPlayer->Name()) {
            oldPlayer = a;
            break;
          }
        }
        if (!oldPlayer) {
          continue;
        }

        // Update the old from the new so we can keep the original object
        // identity (and any expando properties attached to it).
        if (oldPlayer->GetSource() && newPlayer->GetSource()) {
          Animation* oldAnim = oldPlayer->GetSource();
          Animation* newAnim = newPlayer->GetSource();
          oldAnim->Timing() = newAnim->Timing();
          oldAnim->Properties() = newAnim->Properties();
        }

        // Reset compositor state so animation will be re-synchronized.
        oldPlayer->ClearIsRunningOnCompositor();

        // Handle changes in play state.
        // CSSAnimationPlayer takes care of override behavior so that,
        // for example, if the author has called pause(), that will
        // override the animation-play-state.
        // (We should check newPlayer->IsStylePaused() but that requires
        //  downcasting to CSSAnimationPlayer and we happen to know that
        //  newPlayer will only ever be paused by calling PauseFromStyle
        //  making IsPaused synonymous in this case.)
        if (!oldPlayer->IsStylePaused() && newPlayer->IsPaused()) {
          oldPlayer->PauseFromStyle();
        } else if (oldPlayer->IsStylePaused() && !newPlayer->IsPaused()) {
          oldPlayer->PlayFromStyle();
        }

        // Replace new animation with the (updated) old one and remove the
        // old one from the array so we don't try to match it any more.
        //
        // Although we're doing this while iterating this is safe because
        // we're not changing the length of newPlayers and we've finished
        // iterating over the list of old iterations.
        newPlayer->Cancel();
        newPlayer = nullptr;
        newPlayers.ReplaceElementAt(newIdx, oldPlayer);
        collection->mPlayers.RemoveElementAt(oldIdx);
      }
    }
  } else {
    collection =
      GetAnimationPlayers(aElement, aStyleContext->GetPseudoType(), true);
  }
  collection->mPlayers.SwapElements(newPlayers);
  collection->mNeedsRefreshes = true;
  collection->Tick();

  // Cancel removed animations
  for (size_t newPlayerIdx = newPlayers.Length(); newPlayerIdx-- != 0; ) {
    newPlayers[newPlayerIdx]->Cancel();
  }

  TimeStamp refreshTime = mPresContext->RefreshDriver()->MostRecentRefresh();
  UpdateStyleAndEvents(collection, refreshTime,
                       EnsureStyleRule_IsNotThrottled);
  // We don't actually dispatch the mPendingEvents now.  We'll either
  // dispatch them the next time we get a refresh driver notification
  // or the next time somebody calls
  // nsPresShell::FlushPendingNotifications.
  if (!mPendingEvents.IsEmpty()) {
    mPresContext->Document()->SetNeedStyleFlush();
  }

  return GetAnimationRule(aElement, aStyleContext->GetPseudoType());
}

struct KeyframeData {
  float mKey;
  uint32_t mIndex; // store original order since sort algorithm is not stable
  nsCSSKeyframeRule *mRule;
};

struct KeyframeDataComparator {
  bool Equals(const KeyframeData& A, const KeyframeData& B) const {
    return A.mKey == B.mKey && A.mIndex == B.mIndex;
  }
  bool LessThan(const KeyframeData& A, const KeyframeData& B) const {
    return A.mKey < B.mKey || (A.mKey == B.mKey && A.mIndex < B.mIndex);
  }
};

class ResolvedStyleCache {
public:
  ResolvedStyleCache() : mCache() {}
  nsStyleContext* Get(nsPresContext *aPresContext,
                      nsStyleContext *aParentStyleContext,
                      nsCSSKeyframeRule *aKeyframe);

private:
  nsRefPtrHashtable<nsPtrHashKey<nsCSSKeyframeRule>, nsStyleContext> mCache;
};

nsStyleContext*
ResolvedStyleCache::Get(nsPresContext *aPresContext,
                        nsStyleContext *aParentStyleContext,
                        nsCSSKeyframeRule *aKeyframe)
{
  // FIXME (spec):  The css3-animations spec isn't very clear about how
  // properties are resolved when they have values that depend on other
  // properties (e.g., values in 'em').  I presume that they're resolved
  // relative to the other styles of the element.  The question is
  // whether they are resolved relative to other animations:  I assume
  // that they're not, since that would prevent us from caching a lot of
  // data that we'd really like to cache (in particular, the
  // StyleAnimationValue values in AnimationPropertySegment).
  nsStyleContext *result = mCache.GetWeak(aKeyframe);
  if (!result) {
    nsCOMArray<nsIStyleRule> rules;
    rules.AppendObject(aKeyframe);
    nsRefPtr<nsStyleContext> resultStrong = aPresContext->StyleSet()->
      ResolveStyleByAddingRules(aParentStyleContext, rules);
    mCache.Put(aKeyframe, resultStrong);
    result = resultStrong;
  }
  return result;
}

void
nsAnimationManager::BuildAnimations(nsStyleContext* aStyleContext,
                                    dom::Element* aTarget,
                                    dom::AnimationTimeline* aTimeline,
                                    AnimationPlayerPtrArray& aPlayers)
{
  MOZ_ASSERT(aPlayers.IsEmpty(), "expect empty array");

  ResolvedStyleCache resolvedStyles;

  const nsStyleDisplay *disp = aStyleContext->StyleDisplay();

  nsRefPtr<nsStyleContext> styleWithoutAnimation;

  for (size_t animIdx = 0, animEnd = disp->mAnimationNameCount;
       animIdx != animEnd; ++animIdx) {
    const StyleAnimation& src = disp->mAnimations[animIdx];

    // CSS Animations whose animation-name does not match a @keyframes rule do
    // not generate animation events. This includes when the animation-name is
    // "none" which is represented by an empty name in the StyleAnimation.
    // Since such animations neither affect style nor dispatch events, we do
    // not generate a corresponding AnimationPlayer for them.
    nsCSSKeyframesRule* rule =
      src.GetName().IsEmpty()
      ? nullptr
      : mPresContext->StyleSet()->KeyframesRuleForName(mPresContext,
                                                       src.GetName());
    if (!rule) {
      continue;
    }

    nsRefPtr<CSSAnimationPlayer> dest = new CSSAnimationPlayer(aTimeline);
    aPlayers.AppendElement(dest);

    AnimationTiming timing;
    timing.mIterationDuration =
      TimeDuration::FromMilliseconds(src.GetDuration());
    timing.mDelay = TimeDuration::FromMilliseconds(src.GetDelay());
    timing.mIterationCount = src.GetIterationCount();
    timing.mDirection = src.GetDirection();
    timing.mFillMode = src.GetFillMode();

    nsRefPtr<Animation> destAnim =
      new Animation(mPresContext->Document(), aTarget,
                    aStyleContext->GetPseudoType(), timing, src.GetName());
    dest->SetSource(destAnim);

    // Even in the case where we call PauseFromStyle below, we still need to
    // call PlayFromStyle first. This is because a newly-created player is idle
    // and has no effect until it is played (or otherwise given a start time).
    dest->PlayFromStyle();

    if (src.GetPlayState() == NS_STYLE_ANIMATION_PLAY_STATE_PAUSED) {
      dest->PauseFromStyle();
    }

    // While current drafts of css3-animations say that later keyframes
    // with the same key entirely replace earlier ones (no cascading),
    // this is a bad idea and contradictory to the rest of CSS.  So
    // we're going to keep all the keyframes for each key and then do
    // the replacement on a per-property basis rather than a per-rule
    // basis, just like everything else in CSS.

    AutoInfallibleTArray<KeyframeData, 16> sortedKeyframes;

    for (uint32_t ruleIdx = 0, ruleEnd = rule->StyleRuleCount();
         ruleIdx != ruleEnd; ++ruleIdx) {
      css::Rule* cssRule = rule->GetStyleRuleAt(ruleIdx);
      MOZ_ASSERT(cssRule, "must have rule");
      MOZ_ASSERT(cssRule->GetType() == css::Rule::KEYFRAME_RULE,
                 "must be keyframe rule");
      nsCSSKeyframeRule *kfRule = static_cast<nsCSSKeyframeRule*>(cssRule);

      const nsTArray<float> &keys = kfRule->GetKeys();
      for (uint32_t keyIdx = 0, keyEnd = keys.Length();
           keyIdx != keyEnd; ++keyIdx) {
        float key = keys[keyIdx];
        // FIXME (spec):  The spec doesn't say what to do with
        // out-of-range keyframes.  We'll ignore them.
        if (0.0f <= key && key <= 1.0f) {
          KeyframeData *data = sortedKeyframes.AppendElement();
          data->mKey = key;
          data->mIndex = ruleIdx;
          data->mRule = kfRule;
        }
      }
    }

    sortedKeyframes.Sort(KeyframeDataComparator());

    if (sortedKeyframes.Length() == 0) {
      // no segments
      continue;
    }

    // Record the properties that are present in any keyframe rules we
    // are using.
    nsCSSPropertySet properties;

    for (uint32_t kfIdx = 0, kfEnd = sortedKeyframes.Length();
         kfIdx != kfEnd; ++kfIdx) {
      css::Declaration *decl = sortedKeyframes[kfIdx].mRule->Declaration();
      for (uint32_t propIdx = 0, propEnd = decl->Count();
           propIdx != propEnd; ++propIdx) {
        nsCSSProperty prop = decl->GetPropertyAt(propIdx);
        if (prop != eCSSPropertyExtra_variable) {
          // CSS Variables are not animatable
          properties.AddProperty(prop);
        }
      }
    }

    for (nsCSSProperty prop = nsCSSProperty(0);
         prop < eCSSProperty_COUNT_no_shorthands;
         prop = nsCSSProperty(prop + 1)) {
      if (!properties.HasProperty(prop) ||
          nsCSSProps::kAnimTypeTable[prop] == eStyleAnimType_None) {
        continue;
      }

      // Build a list of the keyframes to use for this property.  This
      // means we need every keyframe with the property in it, except
      // for those keyframes where a later keyframe with the *same key*
      // also has the property.
      AutoInfallibleTArray<uint32_t, 16> keyframesWithProperty;
      float lastKey = 100.0f; // an invalid key
      for (uint32_t kfIdx = 0, kfEnd = sortedKeyframes.Length();
           kfIdx != kfEnd; ++kfIdx) {
        KeyframeData &kf = sortedKeyframes[kfIdx];
        if (!kf.mRule->Declaration()->HasProperty(prop)) {
          continue;
        }
        if (kf.mKey == lastKey) {
          // Replace previous occurrence of same key.
          keyframesWithProperty[keyframesWithProperty.Length() - 1] = kfIdx;
        } else {
          keyframesWithProperty.AppendElement(kfIdx);
        }
        lastKey = kf.mKey;
      }

      AnimationProperty &propData = *destAnim->Properties().AppendElement();
      propData.mProperty = prop;

      KeyframeData *fromKeyframe = nullptr;
      nsRefPtr<nsStyleContext> fromContext;
      bool interpolated = true;
      for (uint32_t wpIdx = 0, wpEnd = keyframesWithProperty.Length();
           wpIdx != wpEnd; ++wpIdx) {
        uint32_t kfIdx = keyframesWithProperty[wpIdx];
        KeyframeData &toKeyframe = sortedKeyframes[kfIdx];

        nsRefPtr<nsStyleContext> toContext =
          resolvedStyles.Get(mPresContext, aStyleContext, toKeyframe.mRule);

        if (fromKeyframe) {
          interpolated = interpolated &&
            BuildSegment(propData.mSegments, prop, src,
                         fromKeyframe->mKey, fromContext,
                         fromKeyframe->mRule->Declaration(),
                         toKeyframe.mKey, toContext);
        } else {
          if (toKeyframe.mKey != 0.0f) {
            // There's no data for this property at 0%, so use the
            // cascaded value above us.
            if (!styleWithoutAnimation) {
              styleWithoutAnimation = mPresContext->StyleSet()->
                ResolveStyleWithoutAnimation(aTarget, aStyleContext,
                                             eRestyle_AllHintsWithAnimations);
            }
            interpolated = interpolated &&
              BuildSegment(propData.mSegments, prop, src,
                           0.0f, styleWithoutAnimation, nullptr,
                           toKeyframe.mKey, toContext);
          }
        }

        fromContext = toContext;
        fromKeyframe = &toKeyframe;
      }

      if (fromKeyframe->mKey != 1.0f) {
        // There's no data for this property at 100%, so use the
        // cascaded value above us.
        if (!styleWithoutAnimation) {
          styleWithoutAnimation = mPresContext->StyleSet()->
            ResolveStyleWithoutAnimation(aTarget, aStyleContext,
                                         eRestyle_AllHintsWithAnimations);
        }
        interpolated = interpolated &&
          BuildSegment(propData.mSegments, prop, src,
                       fromKeyframe->mKey, fromContext,
                       fromKeyframe->mRule->Declaration(),
                       1.0f, styleWithoutAnimation);
      }

      // If we failed to build any segments due to inability to
      // interpolate, remove the property from the animation.  (It's not
      // clear if this is the right thing to do -- we could run some of
      // the segments, but it's really not clear whether we should skip
      // values (which?) or skip segments, so best to skip the whole
      // thing for now.)
      if (!interpolated) {
        destAnim->Properties().RemoveElementAt(
          destAnim->Properties().Length() - 1);
      }
    }
  }
}

bool
nsAnimationManager::BuildSegment(InfallibleTArray<AnimationPropertySegment>&
                                   aSegments,
                                 nsCSSProperty aProperty,
                                 const StyleAnimation& aAnimation,
                                 float aFromKey, nsStyleContext* aFromContext,
                                 mozilla::css::Declaration* aFromDeclaration,
                                 float aToKey, nsStyleContext* aToContext)
{
  StyleAnimationValue fromValue, toValue, dummyValue;
  if (!ExtractComputedValueForTransition(aProperty, aFromContext, fromValue) ||
      !ExtractComputedValueForTransition(aProperty, aToContext, toValue) ||
      // Check that we can interpolate between these values
      // (If this is ever a performance problem, we could add a
      // CanInterpolate method, but it seems fine for now.)
      !StyleAnimationValue::Interpolate(aProperty, fromValue, toValue,
                                        0.5, dummyValue)) {
    return false;
  }

  AnimationPropertySegment &segment = *aSegments.AppendElement();

  segment.mFromValue = fromValue;
  segment.mToValue = toValue;
  segment.mFromKey = aFromKey;
  segment.mToKey = aToKey;
  const nsTimingFunction *tf;
  if (aFromDeclaration &&
      aFromDeclaration->HasProperty(eCSSProperty_animation_timing_function)) {
    tf = &aFromContext->StyleDisplay()->mAnimations[0].GetTimingFunction();
  } else {
    tf = &aAnimation.GetTimingFunction();
  }
  segment.mTimingFunction.Init(*tf);

  return true;
}

/* virtual */ void
nsAnimationManager::WillRefresh(mozilla::TimeStamp aTime)
{
  MOZ_ASSERT(mPresContext,
             "refresh driver should not notify additional observers "
             "after pres context has been destroyed");
  if (!mPresContext->GetPresShell()) {
    // Someone might be keeping mPresContext alive past the point
    // where it has been torn down; don't bother doing anything in
    // this case.  But do get rid of all our transitions so we stop
    // triggering refreshes.
    RemoveAllElementCollections();
    return;
  }

  FlushAnimations(Can_Throttle);
}

void
nsAnimationManager::FlushAnimations(FlushFlags aFlags)
{
  // FIXME: check that there's at least one style rule that's not
  // in its "done" state, and if there isn't, remove ourselves from
  // the refresh driver (but leave the animations!).
  TimeStamp now = mPresContext->RefreshDriver()->MostRecentRefresh();
  bool didThrottle = false;
  for (PRCList *l = PR_LIST_HEAD(&mElementCollections);
       l != &mElementCollections;
       l = PR_NEXT_LINK(l)) {
    AnimationPlayerCollection* collection =
      static_cast<AnimationPlayerCollection*>(l);
    collection->Tick();
    bool canThrottleTick = aFlags == Can_Throttle &&
      collection->CanPerformOnCompositorThread(
        AnimationPlayerCollection::CanAnimateFlags(0)) &&
      collection->CanThrottleAnimation(now);

    nsRefPtr<css::AnimValuesStyleRule> oldStyleRule = collection->mStyleRule;
    UpdateStyleAndEvents(collection, now, canThrottleTick
                                          ? EnsureStyleRule_IsThrottled
                                          : EnsureStyleRule_IsNotThrottled);
    if (oldStyleRule != collection->mStyleRule) {
      collection->PostRestyleForAnimation(mPresContext);
    } else {
      didThrottle = true;
    }
  }

  if (didThrottle) {
    mPresContext->Document()->SetNeedStyleFlush();
  }

  DispatchEvents(); // may destroy us
}

void
nsAnimationManager::DoDispatchEvents()
{
  nsRefPtr<nsAnimationManager> kungFuDeathGrip(this);
  EventArray events;
  mPendingEvents.SwapElements(events);
  for (uint32_t i = 0, i_end = events.Length(); i < i_end; ++i) {
    AnimationEventInfo &info = events[i];
    EventDispatcher::Dispatch(info.mElement, mPresContext, &info.mEvent);

    if (!mPresContext) {
      break;
    }
  }
}
