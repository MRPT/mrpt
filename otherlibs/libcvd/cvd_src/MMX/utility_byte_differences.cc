#include "cvd/utility.h"
#include "cvd_src/utility_helpers.h"
#include <mmintrin.h>

namespace CVD {

    void byte_to_short_differences(const __m64* a, const __m64* b, __m64* diff, size_t count)
    {
	__m64 z = _mm_setzero_si64();
	for (;count; --count, ++a, ++b, diff+=2) {
	    __m64 aq = *a;
	    __m64 bq = *b;
	    __m64 alo = _mm_unpacklo_pi8(aq,z);
	    __m64 ahi = _mm_unpackhi_pi8(aq,z);
	    __m64 blo = _mm_unpacklo_pi8(bq,z);
	    __m64 bhi = _mm_unpackhi_pi8(bq,z);
	    diff[0] = _mm_sub_pi16(alo,blo);
	    diff[1] = _mm_sub_pi16(ahi,bhi);
	}
	_mm_empty();
    }

    void short_differences(const __m64* a, const __m64* b, __m64* diff, size_t count)
    {
	while (count--) {
	    *(diff++) = _mm_sub_pi16(*(a++), *(b++));
	}
	_mm_empty();
    }

    
    struct MMX_funcs {
	template <class T1, class T2> static inline void unaligned_differences(const T1* a, const T1* b, T2* diff, size_t count) {
	    differences<T1,T2>(a,b,diff,count);
	}
	static inline void aligned_differences(const byte* a, const byte* b, short* diff, size_t count) {
	    if (is_aligned<8>(b))
		byte_to_short_differences((const __m64*)a,(const __m64*)b, (__m64*)diff, count>>3);
	    else
		unaligned_differences(a,b,diff,count);
	}
	
	static inline void aligned_differences(const short* a, const short* b, short* diff, size_t count) {
	    if (is_aligned<8>(b))	    
		short_differences((const __m64*)a, (const __m64*)b, (__m64*)diff, count>>2);
	    else
		unaligned_differences(a,b,diff,count);
	}
    };

    void differences(const byte* a, const byte* b, short* diff, size_t count) {
	maybe_aligned_differences<MMX_funcs, byte, short, 8, 8>(a,b,diff,count);
    }
    
    void differences(const short* a, const short* b, short* diff, size_t count) {
	maybe_aligned_differences<MMX_funcs, short, short, 8, 4>(a,b,diff,count);
    }
}
