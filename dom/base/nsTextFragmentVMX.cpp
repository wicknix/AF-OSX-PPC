// For TenFourFox AltiVec by Cameron Kaiser and Tobias Netzel
// Modified for M682592 (issue 75, issue 109)
// Remember, PPC is normally big endian! Endian Little Hate We! -- Connectix

#include "nscore.h"
#include "nsAlgorithm.h"
#include <algorithm>
#include "nsTextFragmentImpl.h"
#include <altivec.h>
#warning compiled VMX version

namespace mozilla {
namespace VMX {

int32_t
FirstNon8Bit(const char16_t *str, const char16_t *end)
{
  const uint32_t numUnicharsPerVector = 8;
  const uint32_t numCharsPerVector = 16;
  register vector unsigned short vect;

  typedef Non8BitParameters<sizeof(size_t)> p;
  const uint32_t alignMask = p::alignMask();
  const size_t mask = p::mask();
  const uint32_t numUnicharsPerWord = p::numUnicharsPerWord();

  const int32_t len = end - str;
  int32_t i = 0;

  // Align ourselves to a 16-byte boundary, as required by AltiVec loads.
  int32_t alignLen =
    std::min(len, int32_t(((-NS_PTR_TO_UINT32(str)) & 0xf) / sizeof(char16_t)));

if ((len - alignLen) > (numUnicharsPerVector - 1)) {
  for (; i < alignLen; i++) {
    if (str[i] > 255)
      return i;
  }

  register const vector unsigned short gtcompare =
	vec_mergel( vec_splat_s8( 0 ), vec_splat_s8( -1 ) );
  // Check one VMX register (16 bytes) at a time.
  // This is simpler on AltiVec and involves no mucking about with masks,
  // since the vec_any_gt intrinsic does exactly what we want.
  const int32_t vectWalkEnd = ((len - i) / numUnicharsPerVector) * numUnicharsPerVector;
  // We use this a lot, so let's calculate it now.
  const int32_t vectFactor = (numCharsPerVector/numUnicharsPerVector);
    int32_t i2 = i * vectFactor;
    while (1) {
#define CheckForASCII						\
      vect = vec_ld(i2, (unsigned short *)str);			\
      if (vec_any_gt(vect, gtcompare))				\
        return (i2 / vectFactor);                               \
      i2 += numCharsPerVector;					\
      if (!(i2 < vectWalkEnd))					\
        break;
      CheckForASCII
      CheckForASCII
    }
    i = i2 / vectFactor;
  }
  else {
    // Align ourselves to a word boundary.
    alignLen =
      std::min(len, int32_t(((-NS_PTR_TO_UINT32(str)) & alignMask) / sizeof(char16_t)));
    for (; i < alignLen; i++) {
      if (str[i] > 255)
        return i;
    }
  }

  // Check one word at a time.
  const int32_t wordWalkEnd = ((len - i) / numUnicharsPerWord) * numUnicharsPerWord;
  for(; i < wordWalkEnd; i += numUnicharsPerWord) {
    const size_t word = *reinterpret_cast<const size_t*>(str + i);
    if (word & mask)
      return i;
  }

  // Take care of the remainder one character at a time.
  for (; i < len; i++) {
    if (str[i] > 255) {
      return i;
    }
  }

  return -1;
}

} // namespace VMX
} // namespace mozilla
