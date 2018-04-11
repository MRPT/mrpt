#include <cvd/fast_corner.h>

#include <vector>
#include <list>
#include <cvd/utility.h>
using namespace CVD;
using namespace std;

#include <emmintrin.h>

#include "cvd_src/fast/prototypes.h"
#include "cvd_src/SSE2/faster_corner_utilities.h"

namespace CVD
{
    #include "cvd_src/corner_9.h"

    template <bool Aligned> void faster_corner_detect_9(const BasicImage<byte>& I, std::vector<ImageRef>& corners, const int barrier)
    {
	const int w = I.size().x;
	const int stride = 3*w;
	const int rowStride = I.row_stride();

	const __m128i barriers = _mm_set1_epi8((byte)barrier);

	int xend = I.size().x - 3;
	xend -= (I.size().x-3) % 16;

	for(int y=3; y < I.size().y - 3; y++)
	{
	    const byte* prow = I[y];
	    for(int x=3; x < 16; x++)
	    {
	    	const byte* p = prow + x;
	    	if(is_corner_9<Less>(p, rowStride, barrier) || is_corner_9<Greater>(p, rowStride, barrier))
		    corners.emplace_back(x, y);
	    }

	    for(int x=16; x < xend; x+=16)
	    {
	    	const byte* p = prow + x;
		__m128i lo, hi;
		{
		    const __m128i here = load_si128<Aligned>((const __m128i*)(p));
		    lo = _mm_subs_epu8(here, barriers);
		    hi = _mm_adds_epu8(barriers, here);
		}
		unsigned int ans_0, ans_8, possible;
		{
		    __m128i top = load_si128<Aligned>((const __m128i*)(p-stride));
		    __m128i bottom = load_si128<Aligned>((const __m128i*)(p+stride));

		    CHECK_BARRIER(lo, hi, top, ans_0);
		    CHECK_BARRIER(lo, hi, bottom, ans_8);
		    possible = ans_0 | ans_8;
		    if (!possible)
			continue;
		}

		unsigned int ans_15, ans_1;
		{
		    __m128i a = _mm_loadu_si128((const __m128i*)(p-1-stride));
		    __m128i c = _mm_insert_epi16(_mm_srli_si128(a,2), *(const unsigned short*)(p+15-stride), 7);
		    CHECK_BARRIER(lo, hi, a, ans_15);
		    CHECK_BARRIER(lo, hi, c, ans_1);
		    possible &= ans_8 | (ans_15 & ans_1);
		    if (!possible)
			continue;
		}

		unsigned int ans_9, ans_7;
		{
		    __m128i d = _mm_loadu_si128((const __m128i*)(p-1+stride));
		    __m128i f = _mm_insert_epi16(_mm_srli_si128(d,2), *(const unsigned short*)(p+15+stride), 7);
		    CHECK_BARRIER(lo, hi, d, ans_9);
		    CHECK_BARRIER(lo, hi, f, ans_7);
		    possible &= ans_9 | (ans_0 & ans_1);
		    possible &= ans_7 | (ans_15 & ans_0);
		    if (!possible)
			continue;
		}

		unsigned int ans_12, ans_4;
		{
		    __m128i left = _mm_loadu_si128((const __m128i*)(p-3));
		    __m128i right = _mm_loadu_si128((const __m128i*)(p+3));
		    CHECK_BARRIER(lo, hi, left, ans_12);
		    CHECK_BARRIER(lo, hi, right, ans_4);
		    possible &= ans_12 | (ans_4 & (ans_1 | ans_7));
		    possible &= ans_4 | (ans_12 & (ans_9 | ans_15));
		    if (!possible)
			continue;
		}

		unsigned int ans_14, ans_6;
		{
		    __m128i ul = _mm_loadu_si128((const __m128i*)(p-2-2*w));
		    __m128i lr = _mm_loadu_si128((const __m128i*)(p+2+2*w));
		    CHECK_BARRIER(lo, hi, ul, ans_14);
		    CHECK_BARRIER(lo, hi, lr, ans_6);
		    {
			const unsigned int ans_6_7 = ans_6 & ans_7;
			possible &= ans_14 | (ans_6_7 & (ans_4 | (ans_8 & ans_9)));
			possible &= ans_1 | (ans_6_7) | ans_12;
		    }
		    {
			const unsigned int ans_14_15 = ans_14 & ans_15;
			possible &= ans_6 | (ans_14_15 & (ans_12 | (ans_0 & ans_1)));
			possible &= ans_9 | (ans_14_15) | ans_4;
		    }
		    if (!possible)
			continue;
		}

		unsigned int ans_10, ans_2;
		{
		    __m128i ll = _mm_loadu_si128((const __m128i*)(p-2+2*w));
		    __m128i ur = _mm_loadu_si128((const __m128i*)(p+2-2*w));
		    CHECK_BARRIER(lo, hi, ll, ans_10);
		    CHECK_BARRIER(lo, hi, ur, ans_2);
		    {
			const unsigned int ans_1_2 = ans_1 & ans_2;
			possible &= ans_10 | (ans_1_2 & ((ans_0 & ans_15) | ans_4));
			possible &= ans_12 | (ans_1_2) | (ans_6 & ans_7);
		    }
		    {
			const unsigned int ans_9_10 = ans_9 & ans_10;
			possible &= ans_2 | (ans_9_10 & ((ans_7 & ans_8) | ans_12));
			possible &= ans_4 | (ans_9_10) | (ans_14 & ans_15);
		    }
		    possible &= ans_8 | ans_14 | ans_2;
		    possible &= ans_0 | ans_10 | ans_6;
		    if (!possible)
			continue;
		}

		unsigned int ans_13, ans_5;
		{
		    __m128i g = _mm_loadu_si128((const __m128i*)(p-3-w));
		    __m128i l = _mm_loadu_si128((const __m128i*)(p+3+w));
		    CHECK_BARRIER(lo, hi, g, ans_13);
		    CHECK_BARRIER(lo, hi, l, ans_5);
		    const unsigned int ans_15_0 = ans_15 & ans_0;
		    const unsigned int ans_7_8 = ans_7 & ans_8;
		    {
			const unsigned int ans_12_13 = ans_12 & ans_13;
			possible &= ans_5 | (ans_12_13 & ans_14 & ((ans_15_0) | ans_10));
			possible &= ans_7 | (ans_1 & ans_2) | (ans_12_13);
			possible &= ans_2 | (ans_12_13) | (ans_7_8);
		    }
		    {
			const unsigned int ans_4_5 = ans_4 & ans_5;
			const unsigned int ans_9_10 = ans_9 & ans_10;
			possible &= ans_13 | (ans_4_5 & ans_6 & ((ans_7_8) | ans_2));
			possible &= ans_15 | (ans_4_5) | (ans_9_10);
			possible &= ans_10 | (ans_4_5) | (ans_15_0);
			possible &= ans_15 | (ans_9_10) | (ans_4_5);
		    }

		    possible &= ans_8 | (ans_13 & ans_14) | ans_2;
		    possible &= ans_0 | (ans_5 & ans_6) | ans_10;
		    if (!possible)
			continue;
		}


		unsigned int ans_11, ans_3;
		{
		    __m128i ii = _mm_loadu_si128((const __m128i*)(p-3+w));
		    __m128i jj = _mm_loadu_si128((const __m128i*)(p+3-w));
		    CHECK_BARRIER(lo, hi, ii, ans_11);
		    CHECK_BARRIER(lo, hi, jj, ans_3);
		    {
			const unsigned int ans_2_3 = ans_2 & ans_3;
			possible &= ans_11 | (ans_2_3 & ans_4 & ((ans_0 & ans_1) | (ans_5 & ans_6)));
			possible &= ans_13 | (ans_7 & ans_8) | (ans_2_3);
			possible &= ans_8 | (ans_2_3) | (ans_13 & ans_14);
		    }
		    {
			const unsigned int ans_11_12 = ans_11 & ans_12;
			possible &= ans_3 | (ans_10 & ans_11_12 & ((ans_8 & ans_9) | (ans_13 & ans_14)));
			possible &= ans_1 | (ans_11_12) | (ans_6 & ans_7);
			possible &= ans_6 | (ans_0 & ans_1) | (ans_11_12);
		    }
		    {
			const unsigned int ans_3_4 = ans_3 & ans_4;
			possible &= ans_9 | (ans_3_4) | (ans_14 & ans_15);
			possible &= ans_14 | (ans_8 & ans_9) | (ans_3_4);
		    }
		    {
			const unsigned int ans_10_11 = ans_10 & ans_11;
			possible &= ans_5 | (ans_15 & ans_0) | (ans_10_11);
			possible &= ans_0 | (ans_10_11) | (ans_5 & ans_6);
		    }
		    if (!possible)
			continue;

		}

		possible |= (possible >> 16);

		//if(possible & 0x0f) //Does this make it faster?
		{
		    if(possible & (1<< 0))
		      corners.emplace_back(x + 0, y);
		    if(possible & (1<< 1))
		      corners.emplace_back(x + 1, y);
		    if(possible & (1<< 2))
		      corners.emplace_back(x + 2, y);
		    if(possible & (1<< 3))
		      corners.emplace_back(x + 3, y);
		    if(possible & (1<< 4))
		      corners.emplace_back(x + 4, y);
		    if(possible & (1<< 5))
		      corners.emplace_back(x + 5, y);
		    if(possible & (1<< 6))
		      corners.emplace_back(x + 6, y);
		    if(possible & (1<< 7))
		      corners.emplace_back(x + 7, y);
		}
		//if(possible & 0xf0) //Does this mak( ,  fast)r?
		{
		    if(possible & (1<< 8))
		      corners.emplace_back(x + 8, y);
		    if(possible & (1<< 9))
		      corners.emplace_back(x + 9, y);
		    if(possible & (1<<10))
		      corners.emplace_back(x +10, y);
		    if(possible & (1<<11))
		      corners.emplace_back(x +11, y);
		    if(possible & (1<<12))
		      corners.emplace_back(x +12, y);
		    if(possible & (1<<13))
		      corners.emplace_back(x +13, y);
		    if(possible & (1<<14))
		      corners.emplace_back(x +14, y);
		    if(possible & (1<<15))
		      corners.emplace_back(x +15, y);
		}
	    }

	    for(int x=xend; x < I.size().x - 3; x++)
	    	if(is_corner_9<Less>(&I[y][x], rowStride, barrier) || is_corner_9<Greater>(&I[y][x], rowStride, barrier))
		    corners.emplace_back(x, y);
	}
    }

    void fast_corner_detect_9(const BasicImage<byte>& I, std::vector<ImageRef>& corners, int barrier)
    {
	if (I.size().x < 22) {
	    fast_corner_detect_plain_9(I,corners,barrier);
	    return;
	} else if (I.size().x < 22 || I.size().y < 7)
	    return;

	if (is_aligned<16>(I[0]) && is_aligned<16>(I[1]))
	    faster_corner_detect_9<true>(I, corners, barrier);
	else
	    faster_corner_detect_9<false>(I, corners, barrier);
    }

}
