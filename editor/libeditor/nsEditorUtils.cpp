/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "nsEditorUtils.h"

#include "mozilla/dom/Selection.h"
#include "nsCOMArray.h"
#include "nsComponentManagerUtils.h"
#include "nsError.h"
#include "nsIClipboardDragDropHookList.h"
// hooks
#include "nsIClipboardDragDropHooks.h"
#include "nsIContent.h"
#include "nsIContentIterator.h"
#include "nsIDOMDocument.h"
#include "nsIDocShell.h"
#include "nsIDocument.h"
#include "nsIInterfaceRequestorUtils.h"
#include "nsINode.h"
#include "nsISimpleEnumerator.h"

class nsISupports;
class nsRange;

using namespace mozilla;
using namespace mozilla::dom;

/******************************************************************************
 * nsAutoSelectionReset
 *****************************************************************************/

nsAutoSelectionReset::nsAutoSelectionReset(Selection* aSel, nsEditor* aEd)
  : mSel(nullptr), mEd(nullptr)
{ 
  if (!aSel || !aEd) return;    // not much we can do, bail.
  if (aEd->ArePreservingSelection()) return;   // we already have initted mSavedSel, so this must be nested call.
  mSel = aSel;
  mEd = aEd;
  if (mSel)
  {
    mEd->PreserveSelectionAcrossActions(mSel);
  }
}

nsAutoSelectionReset::~nsAutoSelectionReset()
{
  NS_ASSERTION(!mSel || mEd, "mEd should be non-null when mSel is");
  if (mSel && mEd->ArePreservingSelection())   // mSel will be null if this was nested call
  {
    mEd->RestorePreservedSelection(mSel);
  }
}

void
nsAutoSelectionReset::Abort()
{
  NS_ASSERTION(!mSel || mEd, "mEd should be non-null when mSel is");
  if (mSel)
    mEd->StopPreservingSelection();
}


/******************************************************************************
 * some helper classes for iterating the dom tree
 *****************************************************************************/

nsDOMIterator::nsDOMIterator() :
mIter(nullptr)
{
}
    
nsDOMIterator::~nsDOMIterator()
{
}
    
nsresult
nsDOMIterator::Init(nsRange* aRange)
{
  nsresult res;
  mIter = do_CreateInstance("@mozilla.org/content/post-content-iterator;1", &res);
  NS_ENSURE_SUCCESS(res, res);
  NS_ENSURE_TRUE(mIter, NS_ERROR_FAILURE);
  return mIter->Init(aRange);
}

nsresult
nsDOMIterator::Init(nsIDOMNode* aNode)
{
  nsresult res;
  mIter = do_CreateInstance("@mozilla.org/content/post-content-iterator;1", &res);
  NS_ENSURE_SUCCESS(res, res);
  NS_ENSURE_TRUE(mIter, NS_ERROR_FAILURE);
  nsCOMPtr<nsIContent> content = do_QueryInterface(aNode);
  return mIter->Init(content);
}

nsresult
nsDOMIterator::AppendList(nsBoolDomIterFunctor& functor,
                          nsTArray<nsCOMPtr<nsINode>>& arrayOfNodes) const
{
  // Iterate through dom and build list
  while (!mIter->IsDone()) {
    nsCOMPtr<nsINode> node = mIter->GetCurrentNode();
    NS_ENSURE_TRUE(node, NS_ERROR_NULL_POINTER);

    if (functor(node)) {
      arrayOfNodes.AppendElement(node);
    }
    mIter->Next();
  }
  return NS_OK;
}

nsresult
nsDOMIterator::AppendList(nsBoolDomIterFunctor& functor,
                          nsCOMArray<nsIDOMNode>& arrayOfNodes) const
{
  nsCOMPtr<nsIDOMNode> node;
  
  // iterate through dom and build list
  while (!mIter->IsDone())
  {
    node = do_QueryInterface(mIter->GetCurrentNode());
    NS_ENSURE_TRUE(node, NS_ERROR_NULL_POINTER);

    if (functor(node))
    {
      arrayOfNodes.AppendObject(node);
    }
    mIter->Next();
  }
  return NS_OK;
}

nsDOMSubtreeIterator::nsDOMSubtreeIterator()
{
}
    
nsDOMSubtreeIterator::~nsDOMSubtreeIterator()
{
}
    
nsresult
nsDOMSubtreeIterator::Init(nsRange* aRange)
{
  nsresult res;
  mIter = do_CreateInstance("@mozilla.org/content/subtree-content-iterator;1", &res);
  NS_ENSURE_SUCCESS(res, res);
  NS_ENSURE_TRUE(mIter, NS_ERROR_FAILURE);
  return mIter->Init(aRange);
}

/******************************************************************************
 * some general purpose editor utils
 *****************************************************************************/

bool
nsEditorUtils::IsDescendantOf(nsINode* aNode, nsINode* aParent, int32_t* aOffset)
{
  MOZ_ASSERT(aNode && aParent);
  if (aNode == aParent) {
    return false;
  }

  for (nsCOMPtr<nsINode> node = aNode; node; node = node->GetParentNode()) {
    if (node->GetParentNode() == aParent) {
      if (aOffset) {
        *aOffset = aParent->IndexOf(node);
      }
      return true;
    }
  }

  return false;
}

bool
nsEditorUtils::IsDescendantOf(nsIDOMNode* aNode, nsIDOMNode* aParent, int32_t* aOffset)
{
  nsCOMPtr<nsINode> node = do_QueryInterface(aNode);
  nsCOMPtr<nsINode> parent = do_QueryInterface(aParent);
  NS_ENSURE_TRUE(node && parent, false);
  return IsDescendantOf(node, parent, aOffset);
}

bool
nsEditorUtils::IsLeafNode(nsIDOMNode *aNode)
{
  bool hasChildren = false;
  if (aNode)
    aNode->HasChildNodes(&hasChildren);
  return !hasChildren;
}

/******************************************************************************
 * utility methods for drag/drop/copy/paste hooks
 *****************************************************************************/

nsresult
nsEditorHookUtils::GetHookEnumeratorFromDocument(nsIDOMDocument *aDoc,
                                                 nsISimpleEnumerator **aResult)
{
  nsCOMPtr<nsIDocument> doc = do_QueryInterface(aDoc);
  NS_ENSURE_TRUE(doc, NS_ERROR_FAILURE);

  nsCOMPtr<nsIDocShell> docShell = doc->GetDocShell();
  nsCOMPtr<nsIClipboardDragDropHookList> hookObj = do_GetInterface(docShell);
  NS_ENSURE_TRUE(hookObj, NS_ERROR_FAILURE);

  return hookObj->GetHookEnumerator(aResult);
}

bool
nsEditorHookUtils::DoInsertionHook(nsIDOMDocument *aDoc, nsIDOMEvent *aDropEvent,  
                                   nsITransferable *aTrans)
{
  nsCOMPtr<nsISimpleEnumerator> enumerator;
  GetHookEnumeratorFromDocument(aDoc, getter_AddRefs(enumerator));
  NS_ENSURE_TRUE(enumerator, true);

  bool hasMoreHooks = false;
  while (NS_SUCCEEDED(enumerator->HasMoreElements(&hasMoreHooks)) && hasMoreHooks)
  {
    nsCOMPtr<nsISupports> isupp;
    if (NS_FAILED(enumerator->GetNext(getter_AddRefs(isupp))))
      break;

    nsCOMPtr<nsIClipboardDragDropHooks> _override = do_QueryInterface(isupp);
    if (_override)
    {
      bool doInsert = true;
#ifdef DEBUG
      nsresult hookResult =
#endif
      _override->OnPasteOrDrop(aDropEvent, aTrans, &doInsert);
      NS_ASSERTION(NS_SUCCEEDED(hookResult), "hook failure in OnPasteOrDrop");
      NS_ENSURE_TRUE(doInsert, false);
    }
  }

  return true;
}
