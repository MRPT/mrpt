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
    #include "cvd_src/corner_10.h"

    template <bool Aligned> void faster_corner_detect_10(const BasicImage<byte>& I, std::vector<ImageRef>& corners, const int barrier)
    {
	const int w = I.size().x;
	const int stride = 3*w;

	const __m128i barriers = _mm_set1_epi8((byte)barrier);

	int xend = I.size().x - 3;
	xend -= (I.size().x-3) % 16;

	for(int y=3; y < I.size().y - 3; y++)
	{
	    for(int x=3; x < 16; x++)
	    	if(is_corner_10<Less>(&I[y][x], I.row_stride(), barrier) || is_corner_10<Greater>(&I[y][x], I.row_stride(), barrier))
		    corners.emplace_back(x, y);

	    for(int x=16; x < xend; x+=16)
	    {
	    	const byte* p = &I[y][x];
		__m128i lo, hi;
		{
		    const __m128i here = load_si128<Aligned>((const __m128i*)(p));
		    lo = _mm_subs_epu8(here, barriers);
		    hi = _mm_adds_epu8(barriers, here);
		}
		unsigned int ans_b, ans_e;
		{
		    __m128i top = load_si128<Aligned>((const __m128i*)(p-stride));
		    __m128i bottom = load_si128<Aligned>((const __m128i*)(p+stride));

		    CHECK_BARRIER(lo, hi, top, ans_b);
		    CHECK_BARRIER(lo, hi, bottom, ans_e);
		    if (!(ans_b | ans_e))
			continue;
		}

		unsigned int ans_m, ans_p, possible;
		{
		    __m128i ul = _mm_loadu_si128((const __m128i*)(p-2-2*w));
		    __m128i lr = _mm_loadu_si128((const __m128i*)(p+2+2*w));
		    CHECK_BARRIER(lo, hi, ul, ans_m);
		    CHECK_BARRIER(lo, hi, lr, ans_p);
		    possible = (ans_m & ans_b) | (ans_e & ans_p);
		    if (!possible)
			continue;
		}

		unsigned int ans_o, ans_n;
		{
		    __m128i ll = _mm_loadu_si128((const __m128i*)(p-2+2*w));
		    __m128i ur = _mm_loadu_si128((const __m128i*)(p+2-2*w));
		    CHECK_BARRIER(lo, hi, ll, ans_o);
		    CHECK_BARRIER(lo, hi, ur, ans_n);
		    possible &= ans_o | (ans_b & ans_n);
		    possible &= ans_n | (ans_e & ans_o);
		    if (!possible)
			continue;
		}

		unsigned int ans_h, ans_k;
		{
		    __m128i left = _mm_loadu_si128((const __m128i*)(p-3));
		    __m128i right = _mm_loadu_si128((const __m128i*)(p+3));
		    CHECK_BARRIER(lo, hi, left, ans_h);
		    CHECK_BARRIER(lo, hi, right, ans_k);
		    possible &= ans_h | (ans_n & ans_k & ans_p);
		    possible &= ans_k | (ans_m & ans_h & ans_o);
		    if (!possible)
			continue;
		}

		unsigned int ans_a, ans_c;
		{
		    __m128i a = _mm_loadu_si128((const __m128i*)(p-1-stride));
		    __m128i c = _mm_insert_epi16(_mm_srli_si128(a,2), *(const unsigned short*)(p+15-stride), 7);
		    //__m128i c = _mm_loadu_si128((const __m128i*)(p+1-stride));
		    CHECK_BARRIER(lo, hi, a, ans_a);
		    CHECK_BARRIER(lo, hi, c, ans_c);
		    possible &= ans_a | (ans_e & ans_p);
		    possible &= ans_c | (ans_o & ans_e);
		    if (!possible)
			continue;
		}

		unsigned int ans_d, ans_f;
		{
		    __m128i d = _mm_loadu_si128((const __m128i*)(p-1+stride));
		    __m128i f = _mm_insert_epi16(_mm_srli_si128(d,2), *(const unsigned short*)(p+15+stride), 7);
		    //__m128i f = _mm_loadu_si128((const __m128i*)(p+1+stride));
		    CHECK_BARRIER(lo, hi, d, ans_d);
		    CHECK_BARRIER(lo, hi, f, ans_f);
		    const unsigned int ans_abc = ans_a & ans_b & ans_c;
		    possible &= ans_d | (ans_abc & ans_n);
		    possible &= ans_f | (ans_m & ans_abc);
		    if (!possible)
			continue;
		}

		unsigned int ans_g, ans_i;
		{
		    __m128i g = _mm_loadu_si128((const __m128i*)(p-3-w));
		    __m128i ii = _mm_loadu_si128((const __m128i*)(p-3+w));
		    CHECK_BARRIER(lo, hi, g, ans_g);
		    CHECK_BARRIER(lo, hi, ii, ans_i);
		    possible &= ans_g | (ans_f & ans_p & ans_k);
		    possible &= ans_i | (ans_c & ans_n & ans_k);
		    if (!possible)
			continue;
		}

		unsigned int ans_j, ans_l;
		{
		    __m128i jj = _mm_loadu_si128((const __m128i*)(p+3-w));
		    __m128i l = _mm_loadu_si128((const __m128i*)(p+3+w));
		    CHECK_BARRIER(lo, hi, jj, ans_j);
		    CHECK_BARRIER(lo, hi, l, ans_l);
		    const unsigned int ans_ghi = ans_g & ans_h & ans_i;
		    possible &= ans_j | (ans_d & ans_o & ans_ghi);
		    possible &= ans_l | (ans_m & ans_a & ans_ghi);
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
	    	if(is_corner_10<Less>(&I[y][x], I.row_stride(), barrier) || is_corner_10<Greater>(&I[y][x], I.row_stride(), barrier))
		    corners.emplace_back(x, y);
	}
    }

    void fast_corner_detect_10(const BasicImage<byte>& I, std::vector<ImageRef>& corners, int barrier)
    {
	if (I.size().x < 22) {
	    fast_corner_detect_plain_10(I,corners,barrier);
	    return;
	} else if (I.size().x < 22 || I.size().y < 7)
	    return;

	if (is_aligned<16>(I[0]) && is_aligned<16>(I[1]))
	    faster_corner_detect_10<true>(I, corners, barrier);
	else
	    faster_corner_detect_10<false>(I, corners, barrier);
    }
}
