/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */


#ifndef nsRegion_h__
#define nsRegion_h__

#include <stddef.h>                     // for size_t
#include <stdint.h>                     // for uint32_t, uint64_t
#include <sys/types.h>                  // for int32_t
#include "gfxCore.h"                    // for NS_GFX
#include "mozilla/ToString.h"           // for mozilla::ToString
#include "nsCoord.h"                    // for nscoord
#include "nsError.h"                    // for nsresult
#include "nsPoint.h"                    // for nsIntPoint, nsPoint
#include "nsRect.h"                     // for nsIntRect, nsRect
#include "nsMargin.h"                   // for nsIntMargin
#include "nsStringGlue.h"               // for nsCString
#include "xpcom-config.h"               // for CPP_THROW_NEW
#include "mozilla/Move.h"               // for mozilla::Move

class nsIntRegion;
class gfx3DMatrix;

#include "pixman.h"

/* For information on the internal representation look at pixman-region.c
 *
 * This replaces an older homebrew implementation of nsRegion. The
 * representation used here may use more rectangles than nsRegion however, the
 * representation is canonical.  This means that there's no need for an
 * Optimize() method because for a paticular region there is only one
 * representation. This means that nsIntRegion will have more predictable
 * performance characteristics than the old nsRegion and should not become
 * degenerate.
 *
 * The pixman region code originates from X11 which has spread to a variety of
 * projects including Qt, Gtk, Wine. It should perform reasonably well.
 */

enum class VisitSide {
	TOP,
	BOTTOM,
	LEFT,
	RIGHT
};

class nsRegionRectIterator;

class nsRegion
{

  friend class nsRegionRectIterator;

public:
  nsRegion () { pixman_region32_init(&mImpl); }
  MOZ_IMPLICIT nsRegion (const nsRect& aRect) { pixman_region32_init_rect(&mImpl,
                                                                          aRect.x,
                                                                          aRect.y,
                                                                          aRect.width,
                                                                          aRect.height); }
  nsRegion (const nsRegion& aRegion) { pixman_region32_init(&mImpl); pixman_region32_copy(&mImpl,aRegion.Impl()); }
  nsRegion (nsRegion&& aRegion) { mImpl = aRegion.mImpl; pixman_region32_init(&aRegion.mImpl); }
  nsRegion& operator = (nsRegion&& aRegion) {
      pixman_region32_fini(&mImpl);
      mImpl = aRegion.mImpl;
      pixman_region32_init(&aRegion.mImpl);
      return *this;
  }
 ~nsRegion () { pixman_region32_fini(&mImpl); }
  nsRegion& operator = (const nsRect& aRect) { Copy (aRect); return *this; }
  nsRegion& operator = (const nsRegion& aRegion) { Copy (aRegion); return *this; }
  bool operator==(const nsRegion& aRgn) const
  {
    return IsEqual(aRgn);
  }
  bool operator!=(const nsRegion& aRgn) const
  {
    return !(*this == aRgn);
  }

  friend std::ostream& operator<<(std::ostream& stream, const nsRegion& m);

  void Swap(nsRegion* aOther)
  {
    pixman_region32_t tmp = mImpl;
    mImpl = aOther->mImpl;
    aOther->mImpl = tmp;
  }

  static
  nsresult InitStatic()
  {
    return NS_OK;
  }

  static
  void ShutdownStatic() {}

  void AndWith(const nsRegion& aOther)
  {
    And(*this, aOther);
  }
  void AndWith(const nsRect& aOther)
  {
    And(*this, aOther);
  }
  nsRegion& And(const nsRegion& aRgn1,   const nsRegion& aRgn2)
  {
    pixman_region32_intersect(&mImpl, aRgn1.Impl(), aRgn2.Impl());
    return *this;
  }
  nsRegion& And(const nsRect& aRect, const nsRegion& aRegion)
  {
    return And(aRegion, aRect);
  }
  nsRegion& And(const nsRegion& aRegion, const nsRect& aRect)
  {
    pixman_region32_intersect_rect(&mImpl, aRegion.Impl(), aRect.x, aRect.y, aRect.width, aRect.height);
    return *this;
  }
  nsRegion& And(const nsRect& aRect1, const nsRect& aRect2)
  {
    nsRect TmpRect;

    TmpRect.IntersectRect(aRect1, aRect2);
    return Copy(TmpRect);
  }

  nsRegion& OrWith(const nsRegion& aOther)
  {
    return Or(*this, aOther);
  }
  nsRegion& OrWith(const nsRect& aOther)
  {
    return Or(*this, aOther);
  }
  nsRegion& Or(const nsRegion& aRgn1, const nsRegion& aRgn2)
  {
    pixman_region32_union(&mImpl, aRgn1.Impl(), aRgn2.Impl());
    return *this;
  }
  nsRegion& Or(const nsRegion& aRegion, const nsRect& aRect)
  {
    pixman_region32_union_rect(&mImpl, aRegion.Impl(), aRect.x, aRect.y, aRect.width, aRect.height);
    return *this;
  }
  nsRegion& Or(const nsRect& aRect, const nsRegion& aRegion)
  {
    return  Or(aRegion, aRect);
  }
  nsRegion& Or(const nsRect& aRect1, const nsRect& aRect2)
  {
    Copy (aRect1);
    return Or (*this, aRect2);
  }

  nsRegion& XorWith(const nsRegion& aOther)
  {
    return Xor(*this, aOther);
  }
  nsRegion& XorWith(const nsRect& aOther)
  {
    return Xor(*this, aOther);
  }
  nsRegion& Xor(const nsRegion& aRgn1,   const nsRegion& aRgn2)
  {
    // this could be implemented better if pixman had direct
    // support for xoring regions.
    nsRegion p;
    p.Sub(aRgn1, aRgn2);
    nsRegion q;
    q.Sub(aRgn2, aRgn1);
    return Or(p, q);
  }
  nsRegion& Xor(const nsRegion& aRegion, const nsRect& aRect)
  {
    return Xor(aRegion, nsRegion(aRect));
  }
  nsRegion& Xor(const nsRect& aRect, const nsRegion& aRegion)
  {
    return Xor(nsRegion(aRect), aRegion);
  }
  nsRegion& Xor(const nsRect& aRect1, const nsRect& aRect2)
  {
    return Xor(nsRegion(aRect1), nsRegion(aRect2));
  }

  nsRegion ToAppUnits (nscoord aAppUnitsPerPixel) const;

  nsRegion& SubOut(const nsRegion& aOther)
  {
    return Sub(*this, aOther);
  }
  nsRegion& SubOut(const nsRect& aOther)
  {
    return Sub(*this, aOther);
  }
  nsRegion& Sub(const nsRegion& aRgn1, const nsRegion& aRgn2)
  {
    pixman_region32_subtract(&mImpl, aRgn1.Impl(), aRgn2.Impl());
    return *this;
  }
  nsRegion& Sub(const nsRegion& aRegion, const nsRect& aRect)
  {
    return Sub(aRegion, nsRegion(aRect));
  }
  nsRegion& Sub(const nsRect& aRect, const nsRegion& aRegion)
  {
    return Sub(nsRegion(aRect), aRegion);
  }
  nsRegion& Sub(const nsRect& aRect1, const nsRect& aRect2)
  {
    Copy(aRect1);
    return Sub(*this, aRect2);
  }

  /**
   * Returns true iff the given point is inside the region. A region
   * created from a rect (x=0, y=0, w=100, h=100) will NOT contain
   * the point x=100, y=100.
   */
  bool Contains (int aX, int aY) const
  {
    return pixman_region32_contains_point(Impl(), aX, aY, nullptr);
  }
  bool Contains (const nsRect& aRect) const
  {
    pixman_box32_t box = RectToBox(aRect);
    return pixman_region32_contains_rectangle(Impl(), &box) == PIXMAN_REGION_IN;
  }
  bool Contains (const nsRegion& aRgn) const;
  bool Intersects (const nsRect& aRect) const;

  void MoveBy (int32_t aXOffset, int32_t aYOffset)
  {
    MoveBy (nsPoint (aXOffset, aYOffset));
  }
  void MoveBy (nsPoint aPt) { pixman_region32_translate(&mImpl, aPt.x, aPt.y); }
  void SetEmpty ()
  {
    pixman_region32_clear(&mImpl);
  }

  nsRegion MovedBy(int32_t aXOffset, int32_t aYOffset) const
  {
    return MovedBy(nsPoint(aXOffset, aYOffset));
  }
  nsRegion MovedBy(const nsPoint& aPt) const
  {
    nsRegion copy(*this);
    copy.MoveBy(aPt);
    return copy;
  }

  nsRegion Intersect(const nsRegion& aOther) const
  {
    nsRegion intersection;
    intersection.And(*this, aOther);
    return intersection;
  }

  void Inflate(const nsMargin& aMargin);

  nsRegion Inflated(const nsMargin& aMargin) const
  {
    nsRegion copy(*this);
    copy.Inflate(aMargin);
    return copy;
  }

  bool IsEmpty () const { return !pixman_region32_not_empty(Impl()); }
  bool IsComplex () const { return GetNumRects() > 1; }
  bool IsEqual (const nsRegion& aRegion) const
  {
    return pixman_region32_equal(Impl(), aRegion.Impl());
  }
  uint32_t GetNumRects () const
  {
    // Work around pixman bug. Sometimes pixman creates regions with 1 rect
    // that's empty.
    uint32_t result = pixman_region32_n_rects(Impl());
    return (result == 1 && GetBounds().IsEmpty()) ? 0 : result;
  }
  const nsRect GetBounds () const { return BoxToRect(mImpl.extents); }
  uint64_t Area () const;
  // Converts this region from aFromAPP, an appunits per pixel ratio, to
  // aToAPP. This applies nsRect::ConvertAppUnitsRoundOut/In to each rect of
  // the region.
  nsRegion ConvertAppUnitsRoundOut (int32_t aFromAPP, int32_t aToAPP) const;
  nsRegion ConvertAppUnitsRoundIn (int32_t aFromAPP, int32_t aToAPP) const;
  nsRegion& ScaleRoundOut(float aXScale, float aYScale);
  nsRegion& ScaleInverseRoundOut(float aXScale, float aYScale);
  nsRegion& Transform (const gfx3DMatrix &aTransform);
  nsIntRegion ScaleToOutsidePixels (float aXScale, float aYScale, nscoord aAppUnitsPerPixel) const;
  nsIntRegion ScaleToInsidePixels (float aXScale, float aYScale, nscoord aAppUnitsPerPixel) const;
  nsIntRegion ScaleToNearestPixels (float aXScale, float aYScale, nscoord aAppUnitsPerPixel) const;
  nsIntRegion ToOutsidePixels (nscoord aAppUnitsPerPixel) const;
  nsIntRegion ToNearestPixels (nscoord aAppUnitsPerPixel) const;

  /**
   * Gets the largest rectangle contained in the region.
   * @param aContainingRect if non-empty, we choose a rectangle that
   * maximizes the area intersecting with aContainingRect (and break ties by
   * then choosing the largest rectangle overall)
   */
  nsRect GetLargestRectangle (const nsRect& aContainingRect = nsRect()) const;

  /**
   * Make sure the region has at most aMaxRects by adding area to it
   * if necessary. The simplified region will be a superset of the
   * original region. The simplified region's bounding box will be
   * the same as for the current region.
   */
  void SimplifyOutward (uint32_t aMaxRects);
  /**
   * Simplify the region by adding at most aThreshold area between spans of
   * rects.  The simplified region will be a superset of the original region.
   * The simplified region's bounding box will be the same as for the current
   * region.
   */
  void SimplifyOutwardByArea(uint32_t aThreshold);
  /**
   * Make sure the region has at most aMaxRects by removing area from
   * it if necessary. The simplified region will be a subset of the
   * original region.
   */
  void SimplifyInward (uint32_t aMaxRects);

  /**
   * VisitEdges is a weird kind of function that we use for padding
   * out surfaces to prevent texture filtering artifacts.
   * It calls the visitFn callback for each of the exterior edges of
   * the regions. The top and bottom edges will be expanded 1 pixel
   * to the left and right if there's an outside corner. The order
   * the edges are visited is not guaranteed.
   *
   * visitFn has a side parameter that can be TOP,BOTTOM,LEFT,RIGHT
   * and specifies which kind of edge is being visited. x1, y1, x2, y2
   * are the coordinates of the line. (x1 == x2) || (y1 == y2)
   */
  typedef void (*visitFn)(void *closure, VisitSide side, int x1, int y1, int x2, int y2);
  void VisitEdges(visitFn, void *closure);

  nsCString ToString() const;
private:
  pixman_region32_t mImpl;

#ifndef MOZ_TREE_PIXMAN
  // For compatibility with pixman versions older than 0.25.2.
  static inline void
  pixman_region32_clear(pixman_region32_t *region)
  {
    pixman_region32_fini(region);
    pixman_region32_init(region);
  }
#endif

  nsIntRegion ToPixels(nscoord aAppUnitsPerPixel, bool aOutsidePixels) const;

  nsRegion& Copy (const nsRegion& aRegion)
  {
    pixman_region32_copy(&mImpl, aRegion.Impl());
    return *this;
  }

  nsRegion& Copy (const nsRect& aRect)
  {
    // pixman needs to distinguish between an empty region and a region
    // with one rect so that it can return a different number of rectangles.
    // Empty rect: data = empty_box
    //     1 rect: data = null
    //    >1 rect: data = rects
    if (aRect.IsEmpty()) {
      pixman_region32_clear(&mImpl);
    } else {
      pixman_box32_t box = RectToBox(aRect);
      pixman_region32_reset(&mImpl, &box);
    }
    return *this;
  }

  static inline pixman_box32_t RectToBox(const nsRect &aRect)
  {
    pixman_box32_t box = { aRect.x, aRect.y, aRect.XMost(), aRect.YMost() };
    return box;
  }

  static inline pixman_box32_t RectToBox(const nsIntRect &aRect)
  {
    pixman_box32_t box = { aRect.x, aRect.y, aRect.XMost(), aRect.YMost() };
    return box;
  }


  static inline nsRect BoxToRect(const pixman_box32_t &aBox)
  {
    return nsRect(aBox.x1, aBox.y1,
                  aBox.x2 - aBox.x1,
                  aBox.y2 - aBox.y1);
  }

  pixman_region32_t* Impl() const
  {
    return const_cast<pixman_region32_t*>(&mImpl);
  }

};


class NS_GFX nsRegionRectIterator
{
  const nsRegion*  mRegion;
  int i;
  int n;
  nsRect rect;
  pixman_box32_t *boxes;

public:
  explicit nsRegionRectIterator (const nsRegion& aRegion)
  {
    mRegion = &aRegion;
    i = 0;
    boxes = pixman_region32_rectangles(aRegion.Impl(), &n);
    // Work around pixman bug. Sometimes pixman creates regions with 1 rect
    // that's empty.
    if (n == 1 && nsRegion::BoxToRect(boxes[0]).IsEmpty()) {
      n = 0;
    }
  }

  const nsRect* Next ()
  {
    if (i == n)
      return nullptr;
    rect = nsRegion::BoxToRect(boxes[i]);
    NS_ASSERTION(!rect.IsEmpty(), "Shouldn't return empty rect");
    i++;
    return &rect;
  }

  const nsRect* Prev ()
  {
    if (i == -1)
      return nullptr;
    rect = nsRegion::BoxToRect(boxes[i]);
    NS_ASSERTION(!rect.IsEmpty(), "Shouldn't return empty rect");
    i--;
    return &rect;
  }

  void Reset ()
  {
    i = 0;
  }
};

/**
 * nsIntRegions use int32_t coordinates and nsIntRects.
 */
class NS_GFX nsIntRegion
{
  friend class nsIntRegionRectIterator;
  friend class nsRegion;

public:
  nsIntRegion () {}
  MOZ_IMPLICIT nsIntRegion (const nsIntRect& aRect) : mImpl (ToRect(aRect)) {}
  nsIntRegion (const nsIntRegion& aRegion) : mImpl (aRegion.mImpl) {}
  nsIntRegion (nsIntRegion&& aRegion) : mImpl (mozilla::Move(aRegion.mImpl)) {}
  nsIntRegion& operator = (const nsIntRect& aRect) { mImpl = ToRect (aRect); return *this; }
  nsIntRegion& operator = (const nsIntRegion& aRegion) { mImpl = aRegion.mImpl; return *this; }
  nsIntRegion& operator = (nsIntRegion&& aRegion) { mImpl = mozilla::Move(aRegion.mImpl); return *this; }

  bool operator==(const nsIntRegion& aRgn) const
  {
    return IsEqual(aRgn);
  }
  bool operator!=(const nsIntRegion& aRgn) const
  {
    return !(*this == aRgn);
  }

  friend std::ostream& operator<<(std::ostream& stream, const nsIntRegion& m) {
    return stream << m.mImpl;
  }

  void Swap(nsIntRegion* aOther)
  {
    mImpl.Swap(&aOther->mImpl);
  }

  void AndWith(const nsIntRegion& aOther)
  {
    And(*this, aOther);
  }
  void AndWith(const nsIntRect& aOther)
  {
    And(*this, aOther);
  }
  nsIntRegion& And  (const nsIntRegion& aRgn1,   const nsIntRegion& aRgn2)
  {
    mImpl.And (aRgn1.mImpl, aRgn2.mImpl);
    return *this;
  }
  nsIntRegion& And  (const nsIntRegion& aRegion, const nsIntRect& aRect)
  {
    mImpl.And (aRegion.mImpl, ToRect (aRect));
    return *this;
  }
  nsIntRegion& And  (const nsIntRect& aRect, const nsIntRegion& aRegion)
  {
    return  And  (aRegion, aRect);
  }
  nsIntRegion& And  (const nsIntRect& aRect1, const nsIntRect& aRect2)
  {
    nsIntRect TmpRect;

    TmpRect.IntersectRect (aRect1, aRect2);
    mImpl = ToRect (TmpRect);
    return *this;
  }

  nsIntRegion& OrWith(const nsIntRegion& aOther)
  {
    return Or(*this, aOther);
  }
  nsIntRegion& OrWith(const nsIntRect& aOther)
  {
    return Or(*this, aOther);
  }
  nsIntRegion& Or   (const nsIntRegion& aRgn1,   const nsIntRegion& aRgn2)
  {
    mImpl.Or (aRgn1.mImpl, aRgn2.mImpl);
    return *this;
  }
  nsIntRegion& Or   (const nsIntRegion& aRegion, const nsIntRect& aRect)
  {
    mImpl.Or (aRegion.mImpl, ToRect (aRect));
    return *this;
  }
  nsIntRegion& Or   (const nsIntRect& aRect, const nsIntRegion& aRegion)
  {
    return  Or   (aRegion, aRect);
  }
  nsIntRegion& Or   (const nsIntRect& aRect1, const nsIntRect& aRect2)
  {
    mImpl = ToRect (aRect1);
    return Or (*this, aRect2);
  }

  nsIntRegion& XorWith(const nsIntRegion& aOther)
  {
    return Xor(*this, aOther);
  }
  nsIntRegion& XorWith(const nsIntRect& aOther)
  {
    return Xor(*this, aOther);
  }
  nsIntRegion& Xor  (const nsIntRegion& aRgn1,   const nsIntRegion& aRgn2)
  {
    mImpl.Xor (aRgn1.mImpl, aRgn2.mImpl);
    return *this;
  }
  nsIntRegion& Xor  (const nsIntRegion& aRegion, const nsIntRect& aRect)
  {
    mImpl.Xor (aRegion.mImpl, ToRect (aRect));
    return *this;
  }
  nsIntRegion& Xor  (const nsIntRect& aRect, const nsIntRegion& aRegion)
  {
    return  Xor  (aRegion, aRect);
  }
  nsIntRegion& Xor  (const nsIntRect& aRect1, const nsIntRect& aRect2)
  {
    mImpl = ToRect (aRect1);
    return Xor (*this, aRect2);
  }

  nsIntRegion& SubOut(const nsIntRegion& aOther)
  {
    return Sub(*this, aOther);
  }
  nsIntRegion& SubOut(const nsIntRect& aOther)
  {
    return Sub(*this, aOther);
  }
  nsIntRegion& Sub  (const nsIntRegion& aRgn1,   const nsIntRegion& aRgn2)
  {
    mImpl.Sub (aRgn1.mImpl, aRgn2.mImpl);
    return *this;
  }
  nsIntRegion& Sub  (const nsIntRegion& aRegion, const nsIntRect& aRect)
  {
    mImpl.Sub (aRegion.mImpl, ToRect (aRect));
    return *this;
  }
  nsIntRegion& Sub  (const nsIntRect& aRect, const nsIntRegion& aRegion)
  {
    return Sub (nsIntRegion (aRect), aRegion);
  }
  nsIntRegion& Sub  (const nsIntRect& aRect1, const nsIntRect& aRect2)
  {
    mImpl = ToRect (aRect1);
    return Sub (*this, aRect2);
  }

  /**
   * Returns true iff the given point is inside the region. A region
   * created from a rect (x=0, y=0, w=100, h=100) will NOT contain
   * the point x=100, y=100.
   */
  bool Contains (int aX, int aY) const
  {
    return mImpl.Contains(aX, aY);
  }
  bool Contains (const nsIntRect& aRect) const
  {
    return mImpl.Contains (ToRect (aRect));
  }
  bool Contains (const nsIntRegion& aRgn) const
  {
    return mImpl.Contains (aRgn.mImpl);
  }
  bool Intersects (const nsIntRect& aRect) const
  {
    return mImpl.Intersects (ToRect (aRect));
  }

  void MoveBy (int32_t aXOffset, int32_t aYOffset)
  {
    MoveBy (nsIntPoint (aXOffset, aYOffset));
  }
  void MoveBy (nsIntPoint aPt)
  {
    mImpl.MoveBy (aPt.x, aPt.y);
  }
  nsIntRegion MovedBy(int32_t aXOffset, int32_t aYOffset) const
  {
    return MovedBy(nsIntPoint(aXOffset, aYOffset));
  }
  nsIntRegion MovedBy(const nsIntPoint& aPt) const
  {
    nsIntRegion copy(*this);
    copy.MoveBy(aPt);
    return copy;
  }

  nsIntRegion Intersect(const nsIntRegion& aOther) const
  {
    nsIntRegion intersection;
    intersection.And(*this, aOther);
    return intersection;
  }

  void Inflate(const nsIntMargin& aMargin)
  {
    mImpl.Inflate(nsMargin(aMargin.top, aMargin.right, aMargin.bottom, aMargin.left));
  }
  nsIntRegion Inflated(const nsIntMargin& aMargin) const
  {
    nsIntRegion copy(*this);
    copy.Inflate(aMargin);
    return copy;
  }

  void SetEmpty ()
  {
    mImpl.SetEmpty  ();
  }

  bool IsEmpty () const { return mImpl.IsEmpty (); }
  bool IsComplex () const { return mImpl.IsComplex (); }
  bool IsEqual (const nsIntRegion& aRegion) const
  {
    return mImpl.IsEqual (aRegion.mImpl);
  }
  uint32_t GetNumRects () const { return mImpl.GetNumRects (); }
  nsIntRect GetBounds () const { return FromRect (mImpl.GetBounds ()); }
  uint64_t Area () const { return mImpl.Area(); }
  nsRegion ToAppUnits (nscoord aAppUnitsPerPixel) const;
  nsIntRect GetLargestRectangle (const nsIntRect& aContainingRect = nsIntRect()) const
  {
    return FromRect (mImpl.GetLargestRectangle( ToRect(aContainingRect) ));
  }

  nsIntRegion& ScaleRoundOut (float aXScale, float aYScale)
  {
    mImpl.ScaleRoundOut(aXScale, aYScale);
    return *this;
  }

  nsIntRegion& Transform (const gfx3DMatrix &aTransform)
  {
    mImpl.Transform(aTransform);
    return *this;
  }

  /**
   * Make sure the region has at most aMaxRects by adding area to it
   * if necessary. The simplified region will be a superset of the
   * original region. The simplified region's bounding box will be
   * the same as for the current region.
   */
  void SimplifyOutward (uint32_t aMaxRects)
  {
    mImpl.SimplifyOutward (aMaxRects);
  }
  void SimplifyOutwardByArea (uint32_t aThreshold)
  {
    mImpl.SimplifyOutwardByArea (aThreshold);
  }
  /**
   * Make sure the region has at most aMaxRects by removing area from
   * it if necessary. The simplified region will be a subset of the
   * original region.
   */
  void SimplifyInward (uint32_t aMaxRects)
  {
    mImpl.SimplifyInward (aMaxRects);
  }

  typedef void (*visitFn)(void *closure, VisitSide side, int x1, int y1, int x2, int y2);
  void VisitEdges (visitFn visit, void *closure)
  {
    mImpl.VisitEdges (visit, closure);
  }

  nsCString ToString() const { return mImpl.ToString(); }

private:
  nsRegion mImpl;

  static nsRect ToRect(const nsIntRect& aRect)
  {
    return nsRect (aRect.x, aRect.y, aRect.width, aRect.height);
  }
  static nsIntRect FromRect(const nsRect& aRect)
  {
    return nsIntRect (aRect.x, aRect.y, aRect.width, aRect.height);
  }
};

class NS_GFX nsIntRegionRectIterator
{
  nsRegionRectIterator mImpl;
  nsIntRect mTmp;

public:
  explicit nsIntRegionRectIterator (const nsIntRegion& aRegion) : mImpl (aRegion.mImpl) {}

  const nsIntRect* Next ()
  {
    const nsRect* r = mImpl.Next();
    if (!r)
      return nullptr;
    mTmp = nsIntRegion::FromRect (*r);
    return &mTmp;
  }

  const nsIntRect* Prev ()
  {
    const nsRect* r = mImpl.Prev();
    if (!r)
      return nullptr;
    mTmp = nsIntRegion::FromRect (*r);
    return &mTmp;
  }

  void Reset ()
  {
    mImpl.Reset ();
  }
};
#endif
