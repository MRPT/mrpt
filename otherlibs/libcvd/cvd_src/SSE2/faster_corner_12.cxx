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
    #include "cvd_src/corner_12.h"

    template <int I, int N> struct BitCheck {
	template <class C> static inline void eval(unsigned int three, const byte* p, const int w, const int barrier, C& corners) {
	    const int BIT = 1<<I;
	    if (three & BIT) {
		if (three & (BIT << 16)) {
		    if (is_corner_12<Greater>(p, w, barrier))
			corners.push_back(p);
		} else {
		    if (is_corner_12<Less>(p, w, barrier))
			corners.push_back(p);
		}
	    }
	    BitCheck<I+1,N>::eval(three, p+1, w, barrier, corners);
	}
    };

    template <int N> struct BitCheck<N,N> {
	template <class C> static inline void eval(unsigned int, const byte* , const int , const int , C& ) {}
    };

    template <int CHUNKS, class C> inline void process_16(unsigned int three, const byte* p, const int w, const int barrier, C& corners)
    {
	three |= (three >> 16);
	const int BITS = 16/CHUNKS;
	const int mask = ((1<<BITS)-1);
	for (int i=0; i<CHUNKS; ++i) {
	    if (three & mask)
		BitCheck<0,BITS>::eval(three, p, w, barrier, corners);
	    p += BITS;
	    three >>= BITS;
	}
    }

    template <bool Aligned> void faster_corner_detect_12(const BasicImage<byte>& I, std::vector<ImageRef>& corners, int barrier)
    {
	const int w = I.size().x;
	const int stride = 3*w;
	typedef std::list<const byte*> Passed;
	Passed passed;

	const __m128i barriers = _mm_set1_epi8((byte)barrier);

	for (int i=3; i<I.size().y-3; ++i) {
	    const byte* p = I[i]+3;
	    //Do the edge of the row, using the old-fasioned 4 point test
	    for(int j=3; j < 16; j++, p++)
		if(is_corner_12<Greater>(p, w, barrier) || is_corner_12<Less>(p, w, barrier))
		    passed.push_back(p);

	    for (int j=1; j<(w-3)/16; ++j, p+=16) {
		__m128i lo, hi;
		{
		    const __m128i here = load_si128<Aligned>((const __m128i*)(p));
		    lo = _mm_subs_epu8(here, barriers);
		    hi = _mm_adds_epu8(barriers, here);
		}
		const __m128i above = load_si128<Aligned>((const __m128i*)(p-stride));
		const __m128i below = load_si128<Aligned>((const __m128i*)(p+stride));
		unsigned int up_flags, down_flags;
		CHECK_BARRIER(lo, hi, above, up_flags);
		CHECK_BARRIER(lo, hi, below, down_flags);
		const unsigned int either_ud = up_flags | down_flags;
		if (either_ud) {
		    unsigned int left_flags;
		    {
			const __m128i other = _mm_loadu_si128((const __m128i*)(p-3));
			CHECK_BARRIER(lo, hi, other, left_flags);
		    }
		    const unsigned int both_ud = up_flags & down_flags;
		    if (both_ud | (either_ud&left_flags)) {
			unsigned int right_flags;
			{
			    const __m128i other = _mm_loadu_si128((const __m128i*)(p+3));
			    CHECK_BARRIER(lo, hi, other, right_flags);
			}
			const unsigned int at_least_three = (either_ud & (left_flags & right_flags)) | (both_ud & (left_flags | right_flags));
			if (at_least_three) {
			    process_16<4>(at_least_three, p, w, barrier, passed);
			}
		    }
		}
	    }

	    //Do the edge of the row, using the old-fasioned 4 point test
	    for(int j=((w-3)/16) * 16; j < w-3; j++, p++)
	    {
	    	int cb = *p + barrier;
	    	int c_b = *p - barrier;
		int num_above= (p[stride] > cb) + (p[-stride] > cb);
		int num_below= (p[stride] < c_b) + (p[-stride] < c_b);

		if(!num_above && !num_below)
		    continue;

		//Look left
		num_above+= p[-3] > cb;
		num_below+= p[-3] < c_b;

		if(num_above & 2) //num_above is 2 or 3
		{
		    if(!(num_above & 1)) //num_above is 2
			num_above += p[3] > cb;


		    //Only do a complete check if num_above is 3
		    if((num_above&1) && is_corner_12<Greater>(p, w, barrier))
		    	passed.push_back(p);
		}
		else if(num_below & 2)
		{
		    if(!(num_below & 1))
			num_below += p[3] < c_b;


		    if((num_below&1) && is_corner_12<Less>(p, w, barrier))
		    	passed.push_back(p);
		}
	    }

	    passed.push_back(0);
	}
	corners.reserve(passed.size());
	int row = 3;
	const byte* row_start = I[3];
	for (Passed::iterator it = passed.begin(); it != passed.end(); ++it) {
	    if (*it == 0) {
		row_start=I[++row];
		continue;
	    }
	    int x = *it - row_start;
	    if (x > 2 && x < w-3)
		corners.emplace_back(x, row);
	}
    }


    void fast_corner_detect_12(const BasicImage<byte>& I, std::vector<ImageRef>& corners, int barrier)
    {
	if (I.size().x < 22) {
	    fast_corner_detect_plain_12(I,corners,barrier);
	    return;
	} else if (I.size().x < 22 || I.size().y < 7)
	    return;

	if (is_aligned<16>(I[0]) && is_aligned<16>(I[1]))
	    faster_corner_detect_12<true>(I, corners, barrier);
	else
	    faster_corner_detect_12<false>(I, corners, barrier);
    }
}
