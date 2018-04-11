#include "cvd/vision.h"

#include <emmintrin.h>

using namespace std;

namespace CVD {

#define SORT2_SIMD_UB(a,b)			\
    {						\
	const __m128i ta = a;			\
	a = _mm_min_epu8(a,b);			\
	b = _mm_max_epu8(ta,b);			\
    }
    
#define SORT3_SIMD_UB(a,b,c)			\
    {						\
	SORT2_SIMD_UB(a,b);			\
	SORT2_SIMD_UB(b,c);			\
	SORT2_SIMD_UB(a,b);			\
    }
    
    namespace median {
	inline __m128i median3(__m128i a, __m128i b, __m128i c) {
	    SORT2_SIMD_UB(a,b);
	    return _mm_max_epu8(a,_mm_min_epu8(b,c));
	}
	
	__m128i median_3x3_simd(const byte* p, const int w)
	{
	    __m128i a = _mm_loadu_si128((const __m128i*)(p-w-1));
	    __m128i b = _mm_loadu_si128((const __m128i*)(p-w));
	    __m128i c = _mm_loadu_si128((const __m128i*)(p-w+1));
	    SORT3_SIMD_UB(a,b,c);
	    __m128i d = _mm_loadu_si128((const __m128i*)(p-1));
	    __m128i e = _mm_loadu_si128((const __m128i*)(p));
	    __m128i f = _mm_loadu_si128((const __m128i*)(p+1));
	    SORT3_SIMD_UB(d,e,f);
	    __m128i g = _mm_loadu_si128((const __m128i*)(p+w-1));
	    __m128i h = _mm_loadu_si128((const __m128i*)(p+w));
	    __m128i i = _mm_loadu_si128((const __m128i*)(p+w+1));
	    SORT3_SIMD_UB(g,h,i);
	    e = median3(b,e,h);
	    g = _mm_max_epu8(_mm_max_epu8(a,d), g);
	    c = _mm_min_epu8(c, _mm_min_epu8(f,i));
	    return median3(c,e,g);
	}
	
	void median_filter_3x3_simd(const byte* p, const int stride, const int n, byte* out)
	{
	    int j = 0;
	    for (; j+15<n; j+=16, p+=16, out += 16) {
		_mm_storeu_si128((__m128i*)out, median_3x3_simd(p,stride));
	    }
	    if (j < n) {
		const int left = n - j;
		if (n >= 16 && left > 8)
		    _mm_storeu_si128((__m128i*)(out+left-16), median_3x3_simd(p+left-16,stride));
		else
		    median_filter_3x3(p, stride, left, out);
	    }
	}
	
    }

    void median_filter_3x3(const BasicImage<byte>& I, BasicImage<byte> out)
    {
	assert(out.size() == I.size());
	const int s = I.row_stride();
	const int n = I.size().x - 2;
	for (int i=1; i+1<I.size().y; ++i)
	    median::median_filter_3x3_simd(I[i]+1, s, n, out[i]+1);
    }
    

};
