#include "cvd/vision.h"
#include <emmintrin.h>

namespace CVD{
	namespace{


		#define _mm_shuffle_32(X, Y, Z)\
				_mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(X), _mm_castsi128_ps(Y), Z))

		void shift_3(__m128i& w_01_08, __m128i& w_09_16, __m128i& w_17_24)
		{
			//This performs a shift by one word (2 bytes), over three
			//registers, spilling 0 in to pixel 1, pixel 8 in to pixel 9 and 
			//so on.
			//
			//The shift direction moves upwards in memory, so since the three registers
			//represent a row of pixels, the shift moves pixels to the right.
			__m128i w_extra = _mm_shuffle_32(_mm_srli_si128(w_09_16, 2), w_01_08, _MM_SHUFFLE(3, 3, 3, 3));
			w_17_24 = _mm_or_si128(_mm_slli_si128(w_17_24, 2), _mm_and_si128(w_extra, _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, short(0xffff))));
			w_09_16 = _mm_or_si128(_mm_slli_si128(w_09_16, 2), _mm_srli_si128(w_extra, 14));
			w_01_08 = _mm_slli_si128(w_01_08, 2);
		}
		void extract_pairs(__m128i& w_01_08, __m128i& w_09_16, __m128i& w_17_24, __m128i& w_chnk1, __m128i& w_chnk2)
		{
			//[w_01_08 w_09_16 w_17_24] is [ [A B C D E F G H] [I J K L M N O P] [Q R S T U V W X] ]
			//We want to end with a selection of pairs:
			//[ chunk1 chunk2 ] = [ [ B C E F H I K L ] [ N O Q R T W W X ] ] 

			__m128i w_23tmp = _mm_srli_si128(w_01_08, 2);
			w_01_08 = _mm_and_si128(w_01_08, _mm_set_epi16(0, 0, short(0xffff), short(0xffff), 0, 0, 0, 0));
			w_23tmp = _mm_and_si128(w_23tmp, _mm_set_epi16(short(0xffff), short(0xffff), 0, 0, 0, 0, short(0xffff), short(0xffff)));
			w_chnk1 = _mm_or_si128(w_01_08, w_23tmp);

			//This gives us:
			//w_01_08 = [ 0 0 0 0 E F 0 0 ]
			//w23_tmp = [ B C 0 0 0 0 H 0 ]
			//w_chnk1 = [ B C 0 0 E F H 0 ]
			
			w_chnk1 = _mm_shuffle_32(w_chnk1, w_chnk1, _MM_SHUFFLE(1, 3, 2, 0));
			//w_chnk1 = [ B C E F H 0 0 0 ]

			//w_09_16 = [ I J K L M N O P]
			__m128i w_4bitt = _mm_and_si128(_mm_slli_si128(w_09_16, 2),  _mm_set_epi16(short(0xffff), short(0xffff), 0, 0, 0, 0, short(0xffff), 0));
			__m128i w_5_tmp = _mm_and_si128(w_09_16, _mm_set_epi16(0, 0, 0, 0, short(0xffff), short(0xffff), 0, 0));
			//w_4bitt = [0 I 0 0 0 0 N O]
			//w_5_tmp = [0 0 K L 0 0 0 0]

			w_chnk1 = _mm_or_si128(w_chnk1,_mm_slli_si128(_mm_or_si128(w_4bitt, w_5_tmp), 8) );
			//w_chnk1 = [ B C E F H I K L ]

			//w_17_24 = [ Q R S T U V W X]
			w_chnk2 = _mm_shuffle_32(_mm_slli_si128(w_17_24, 6), _mm_setzero_si128(), _MM_SHUFFLE(0, 0, 0, 3));
			//w_chnk2 = [ T U 0 0 0 0 0 0]

			//w_17_24 = [ Q R S T U V W X]
			//w_4bitt = [ 0 I 0 0 0 0 N O]
			__m128i w_chnl2 = _mm_shuffle_32(w_4bitt, w_17_24, _MM_SHUFFLE(3, 0, 3, 2));
			//w_chnl2 = [ 0 0 n o q r w x ]

			w_chnk2 = _mm_or_si128(w_chnk2, w_chnl2);
			w_chnk2 = _mm_shuffle_32(w_chnk2, w_chnk2, _MM_SHUFFLE(3, 0, 2, 1));
			//w_chnk2 = [ n o q r t u w x ]

			//__m128i out_16b = _mm_packs_epi16(w_chnk1, w_chnk2);
			//out_16b = bcefhiklnoqrtuwx
			//return out_16b;
		}


		__m128i square_average(const __m128i& top_01_08, const __m128i& top_09_16, const __m128i& top_17_24)
		{	

			//Step 3: the shift-add stage
			//Perform the shift-add stage
			__m128i s_top_a = top_01_08, s_top_b = top_09_16, s_top_c = top_17_24;
			shift_3(s_top_a, s_top_b, s_top_c);
			s_top_a = _mm_add_epi16(top_01_08, s_top_a);
			s_top_b = _mm_add_epi16(top_09_16, s_top_b);
			s_top_c = _mm_add_epi16(top_17_24, s_top_c);

			//Step 4: extract pairs
			__m128i chunk1, chunk2;
			extract_pairs(s_top_a, s_top_b, s_top_c, chunk1, chunk2);
			
			//Finish off: divide by 9 and pack to bytes.

			//Multiply by 65536/9 and right shift by 16
			//by taking the top word of the integer.
			//Therby multiplying by 9. This provides enough 
			//precision.
			chunk1 = _mm_mulhi_epu16(chunk1, _mm_set_epi16(7282, 7282, 7282, 7282, 7282, 7282, 7282, 7282));
			chunk2 = _mm_mulhi_epu16(chunk2, _mm_set_epi16(7282, 7282, 7282, 7282, 7282, 7282, 7282, 7282));
			return _mm_packus_epi16(chunk1, chunk2);
		}

		void weight_top(__m128i& top_01_08, __m128i& top_09_16, __m128i& top_17_24)
		{
			top_01_08 = _mm_mullo_epi16(top_01_08, _mm_set_epi16(2, 4, 4, 2, 4, 4, 2, 4));
			top_09_16 = _mm_mullo_epi16(top_09_16, _mm_set_epi16(4, 4, 2, 4, 4, 2, 4, 4));
			top_17_24 = _mm_mullo_epi16(top_17_24, _mm_set_epi16(4, 2, 4, 4, 2, 4, 4, 2));
		}

		void weight_mid(__m128i& mid_01_08, __m128i& mid_09_16, __m128i& mid_17_24)
		{
			mid_01_08 = _mm_mullo_epi16(mid_01_08, _mm_set_epi16(1, 2, 2, 1, 2, 2, 1, 2));
			mid_09_16 = _mm_mullo_epi16(mid_09_16, _mm_set_epi16(2, 2, 1, 2, 2, 1, 2, 2));
			mid_17_24 = _mm_mullo_epi16(mid_17_24, _mm_set_epi16(2, 1, 2, 2, 1, 2, 2, 1));
		}

		static __m128i load(const byte* m)
		{
			return _mm_loadu_si128((__m128i*)m);
		}

		static void store(const byte* m, const __m128i& v)
		{
			_mm_storeu_si128((__m128i*)m, v);
		}

		//Reduce a 48x3 pixel strip in to a 32x2 pixel strip
		//Template on the alignedness of all 5 pointers.
		void reduce_48(const byte* data0, const byte* data1, const byte* data2, byte* out0, byte* out1)
		{
			//The first step is to extract the pixels from the image, and 
			//unpack them from 8 bit ints to 16 bit ints for sufficient
			//precision.
			//
			//Three rows of pixels looks like this:
			// A B C D E F  (top)
			// G H I J K L  (mid)
			// M N O P Q R  (bot)
			//
			// After averaging, it will look like this:
			// a b c d
			// e f g h
			//
			// Where a = (4A + 2B + 2G + H)/9
			//       b = (4C + 2B + 2I + H)/9
			//       c = (4M + 2N + 2G + H)/9
			//       d = (4O + 2N + 2I + H)/9
			// 
			// Step 1: Weighting
			// -----------------
			// The first step is to perform weighting on top and mid:
			//   
			//  top = A B C D E F 
			//   *=        *=
			//   w  = 4 2 4 4 2 4
			//   
			//  top = 4A 2B 4C 4D 2E 4F
			//
			// and mid:
			// 
			//  mid = G H I J K L
			//   *=        *=
			//   w  = 2 1 2 2 1 2
			//
			//  mid = 2G  H 2I 2J  K 2L
			//
			//  Step 2: Sum rows
			//  ----------------
			//
			//  Doing top += mid gives:
			//
			//  top  = |   4A  |  2B   |  4C   |  4D   |  2E   |  4F   |
			//   +=    |       |       |    += |       |       |       |
			//  mid  = |   2G  |   H   |  2I   |  2J   |  K    |  2L   |
			//         |       |       |       |       |       |       |
			//  top  = | 4A+2G | 2B+H  | 4C+2I | 4D+2J | 2E+K  | 4F+2L |
			//
			//  Step 3: Shift and add
			//  ---------------------
			//
			// Adding top to a shifted version of itself gives:
			//   
			//  top          = | 4A+2G | 2B+H | 4C+2I | 4D+2J | 2E+K  | 4F+2L |
			//  shifted_top  = |   0   |4A+2G | 2B+H  | 4C+2I | 4D+2J | 2E+K  | 
			//  result       = | junk  | a    |   b   |  junk |  c    |   d   |
			//
			// The fourth step extracts the pairs a b and c d and so on, ignoring the 
			// junk values. The result is then divided by 9 and packed from words
			// back in to bytes.
			//
			// The algorithm works on rows of 24 pixels, sampling them down to 16
			// pixels. Since 24 pixels does is not a multiple of 16, the complete 
			// algorithm works on chunks of 48 pixels, reducing the to 32 pixels.
			//
			// Furthermore, the third row (bot) is averaged against mid in exactly
			// the same way as top, producing a second output row. In this way,
			// 3 rows of 48 pixels are converted in to two rows of 32 pixels.
			
			//Load 48 consecutive bytes from memory	
			__m128i t_1_16 =  load(data0);
			__m128i t_17_32 = load(data0+16);
			__m128i t_33_48 = load(data0+32);

			__m128i m_1_16 =  load(data1);
			__m128i m_17_32 = load(data1+16);
			__m128i m_33_48 = load(data1+32);
			
			__m128i b_1_16 =  load(data2);
			__m128i b_17_32 = load(data2+16);
			__m128i b_33_48 = load(data2+32);

			//Unpack the first 24 bytes in to words
			__m128i top_01_08 = _mm_unpacklo_epi8(t_1_16, _mm_setzero_si128());
			__m128i top_09_16 = _mm_unpackhi_epi8(t_1_16, _mm_setzero_si128());
			__m128i top_17_24 = _mm_unpacklo_epi8(t_17_32,_mm_setzero_si128());

			__m128i mid_01_08 = _mm_unpacklo_epi8(m_1_16, _mm_setzero_si128());
			__m128i mid_09_16 = _mm_unpackhi_epi8(m_1_16, _mm_setzero_si128());
			__m128i mid_17_24 = _mm_unpacklo_epi8(m_17_32,_mm_setzero_si128());

			//Step 1: Weighting
			//Perform the weightings 4 2 4 4 2 4 4 2 4 (top and bottom) 2 1 2 2 1 2 (mid)
			
			weight_top(top_01_08, top_09_16, top_17_24);
			weight_mid(mid_01_08, mid_09_16, mid_17_24);

			//Step 2: Sum rows
			//Sum top to mid
			top_01_08 = _mm_add_epi16(top_01_08, mid_01_08);
			top_09_16 = _mm_add_epi16(top_09_16, mid_09_16);
			top_17_24 = _mm_add_epi16(top_17_24, mid_17_24);
			
			//Steps 3, 4:
			store(out0, square_average(top_01_08, top_09_16, top_17_24));

			/////////////////////////////////////////////
			//
			//Do bot, but in exactly the same way as top.
			//
			top_01_08 = _mm_unpacklo_epi8(b_1_16, _mm_setzero_si128());
			top_09_16 = _mm_unpackhi_epi8(b_1_16, _mm_setzero_si128());
			top_17_24 = _mm_unpacklo_epi8(b_17_32,_mm_setzero_si128());
			weight_top(top_01_08, top_09_16, top_17_24);
			top_01_08 = _mm_add_epi16(top_01_08, mid_01_08);
			top_09_16 = _mm_add_epi16(top_09_16, mid_09_16);
			top_17_24 = _mm_add_epi16(top_17_24, mid_17_24);
			store(out1, square_average(top_01_08, top_09_16, top_17_24));

			//////////////////////////////////////////////////////////////////////////////////////////
			//
			// Do the next 24 pixels in the same way as the first 24
			//
			top_01_08 = _mm_unpackhi_epi8(t_17_32, _mm_setzero_si128());
			top_09_16 = _mm_unpacklo_epi8(t_33_48, _mm_setzero_si128());
			top_17_24 = _mm_unpackhi_epi8(t_33_48,_mm_setzero_si128());
			mid_01_08 = _mm_unpackhi_epi8(m_17_32, _mm_setzero_si128());
			mid_09_16 = _mm_unpacklo_epi8(m_33_48, _mm_setzero_si128());
			mid_17_24 = _mm_unpackhi_epi8(m_33_48,_mm_setzero_si128());
			
			weight_top(top_01_08, top_09_16, top_17_24);
			weight_mid(mid_01_08, mid_09_16, mid_17_24);
			top_01_08 = _mm_add_epi16(top_01_08, mid_01_08);
			top_09_16 = _mm_add_epi16(top_09_16, mid_09_16);
			top_17_24 = _mm_add_epi16(top_17_24, mid_17_24);

			store(out0 + 16, square_average(top_01_08, top_09_16, top_17_24));

			top_01_08 = _mm_unpackhi_epi8(b_17_32, _mm_setzero_si128());
			top_09_16 = _mm_unpacklo_epi8(b_33_48, _mm_setzero_si128());
			top_17_24 = _mm_unpackhi_epi8(b_33_48,_mm_setzero_si128());
			weight_top(top_01_08, top_09_16, top_17_24);
			top_01_08 = _mm_add_epi16(top_01_08, mid_01_08);
			top_09_16 = _mm_add_epi16(top_09_16, mid_09_16);
			top_17_24 = _mm_add_epi16(top_17_24, mid_17_24);
			store(out1 + 16, square_average(top_01_08, top_09_16, top_17_24));
		}

	}

	void twoThirdsSample(const BasicImage<byte>& in, BasicImage<byte>& out)
	{
		if( (in.size()/3*2) != out.size())
			throw Exceptions::Vision::IncompatibleImageSizes(__FUNCTION__);

		for(int yy=0, y=0; y < in.size().y-2; y+=3, yy+=2)
		{
			int xx=0, x=0;
			for(; x < in.size().x-47; x+=48, xx+=32)
			{
				//New CPUs are fast with unaligned loads.
				reduce_48(in[y]+x, in[y+1]+x, in[y+2]+x, out[yy]+xx, out[yy+1]+xx);
			}

			//Resample any remaining pixels.
			for(; x < in.size().x-2; x+=3, xx+=2)
			{
				// a b c
				// d e f
				// g h i

				int b = in[y][x+1]*2;
				int d = in[y+1][x]*2;
				int f = in[y+1][x+2]*2;
				int h = in[y+2][x+1]*2;
				int e = in[y+1][x+1];

				out[yy][xx]     = static_cast<byte>((in[  y][  x]*4+b+d+e)/9);
				out[yy][xx+1]   = static_cast<byte>((in[  y][x+2]*4+b+f+e)/9);
				out[yy+1][xx]   = static_cast<byte>((in[y+2][  x]*4+h+d+e)/9);
				out[yy+1][xx+1] = static_cast<byte>((in[y+2][x+2]*4+h+f+e)/9);
			}
		}
	}

}
