/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd,
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/memory.h>
#include "faster_corner_prototypes.h"

#include <mrpt/utils/SSE_types.h>
#include "faster_corner_utilities.h"
#include "corner_12.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


template <int I, int N> struct BitCheck
{
	template <class C> static inline void eval(unsigned int three, const uint8_t* p, const int w, const int barrier, C& corners) {
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

template <int N> struct BitCheck<N,N>
{
	template <class C> static inline void eval(unsigned int, const uint8_t* , const int , const int , C& ) {}
};

template <int CHUNKS, class C> inline void process_16(unsigned int three, const uint8_t* p, const int w, const int barrier, C& corners)
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


#if MRPT_HAS_SSE2 && MRPT_HAS_OPENCV

template <bool Aligned>
void faster_corner_detect_12(const IplImage* I, mrpt::vision::TSimpleFeatureList & corners, int barrier, uint8_t octave, std::vector<size_t> * out_feats_index_by_row)
{
	corners.reserve(corners.size()+500);
	//corners.mark_kdtree_as_outdated();

	size_t *ptr_feat_index_by_row, *ptr_feat_index_end;
	if (out_feats_index_by_row)
	{
		out_feats_index_by_row->resize(I->height);
		ptr_feat_index_by_row = &(*out_feats_index_by_row)[0];
		ptr_feat_index_end = ptr_feat_index_by_row+out_feats_index_by_row->size();
	}
	else {
		ptr_feat_index_by_row = NULL;
		ptr_feat_index_end=NULL;
	}

const int w = I->width;
const int stride = 3*I->widthStep; // 3*w;
typedef std::list<const uint8_t*> Passed;
Passed passed;

// The compiler refuses to reserve a register for this,
// even though xmm6 and xmm7 go unused.
// It loads it from memory each time.  I am stymied.
const __m128i barriers = _mm_set1_epi8((uint8_t)barrier);

for (int i=3; i<I->height-3; ++i) {
	const uint8_t* p = (const uint8_t*)I->imageData + I->widthStep*i; //  I[i];
    for (int j=0; j<w/16; ++j, p+=16) {
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
    for(int j=(w/16) * 16; j < w-3; j++, p++)
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

	// 3 first rows have no features:
	if (ptr_feat_index_by_row) {
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
	}

	int row = 3;

	const uint8_t* row_start = (const uint8_t*)I->imageData + I->widthStep*3;
	for (Passed::iterator it = passed.begin(); it != passed.end(); ++it) {
		if (*it == 0)
		{
			// next row:
			if (ptr_feat_index_by_row)  // save index by row:
				*ptr_feat_index_by_row++=corners.size();

			row_start= (const uint8_t*)I->imageData + I->widthStep*(++row); //  I[++row];
			continue;
		}
		int x = *it - row_start;
		if (x > 2 && x < w-3)
		corners.push_back_fast(x<<octave, row<<octave);
	}

	// 3 last rows have no features:
	if (ptr_feat_index_by_row) {
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
		*ptr_feat_index_by_row++ = corners.size();
		ASSERT_(ptr_feat_index_by_row==ptr_feat_index_end)
	}

}


#endif // MRPT_HAS_SSE2 && MRPT_HAS_OPENCV


#if MRPT_HAS_OPENCV

void fast_corner_detect_12(const IplImage* I, mrpt::vision::TSimpleFeatureList & corners, int barrier, uint8_t octave, std::vector<size_t> * out_feats_index_by_row)
{
	if (I->width < 22)
	{
		fast_corner_detect_plain_12(I,corners,barrier, octave,out_feats_index_by_row);
		return;
	}
	else if (I->width < 22 || I->height < 7)
		return;

#if MRPT_HAS_SSE2
	if (mrpt::system::is_aligned<16>(I->imageData) && mrpt::system::is_aligned<16>(I->imageData+I->widthStep))
		faster_corner_detect_12<true>(I, corners, barrier, octave,out_feats_index_by_row);
	else
		faster_corner_detect_12<false>(I, corners, barrier, octave,out_feats_index_by_row);
#else
	fast_corner_detect_plain_12(I,corners,barrier, octave,out_feats_index_by_row);
#endif
}
#endif
