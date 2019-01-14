#include "cvd/vision.h"
#include <emmintrin.h>

using namespace std;

namespace CVD {

#ifdef WIN32
#define shift_left_7(thing) _mm_slli_epi16(thing,7)
#else
    // I have to use this because gcc 3.3 has an internal bug with _mm_slli_epi16
#define shift_left_7(thing) asm ( "psllw $0x7, %0  \n\t" : : "x"(thing) : "%0" );
#endif

    void gradient(const byte* in, short (*out)[2], int w, int h)
    {
        const byte zeros[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        const byte* curr = in + w;
        out += w;
        int total = (w*(h-2)) >> 4;
        __m128i zero = _mm_loadu_si128((const __m128i*)zeros);
        while (total--) {
            __m128i up = _mm_load_si128((const __m128i*)(curr-w));
            __m128i down = _mm_load_si128((const __m128i*)(curr+w));            
            __m128i hor_left, hor_right;
            {
                __m128i hor = _mm_load_si128((const __m128i*)curr);
                hor_left = _mm_slli_si128(hor, 1);
                hor_right = _mm_srli_si128(hor, 1);
            }
            {
                __m128i left = _mm_unpacklo_epi8(hor_right, zero);
                __m128i right = _mm_unpacklo_epi8(hor_left, zero);
                __m128i hdiff = _mm_insert_epi16(_mm_sub_epi16(right,left), short(curr[1]) - short(curr[-1]), 0);
                __m128i vdiff = _mm_sub_epi16(_mm_unpacklo_epi8(down, zero), _mm_unpacklo_epi8(up, zero));
                {
                    __m128i part1 = _mm_unpacklo_epi16(hdiff, vdiff);
                    shift_left_7(part1);
                    _mm_stream_si128((__m128i*)out, part1);
                }
                {
                    __m128i part2 = _mm_unpackhi_epi16(hdiff, vdiff);
                    shift_left_7(part2);
                    _mm_stream_si128((__m128i*)(out+4), part2);
                }
            }
            {
                __m128i left = _mm_unpackhi_epi8(hor_right, zero);
                __m128i right = _mm_unpackhi_epi8(hor_left, zero);
                __m128i hdiff = _mm_insert_epi16(_mm_sub_epi16(right,left), short(curr[16]) - short(curr[14]), 7);
                __m128i vdiff = _mm_sub_epi16(_mm_unpackhi_epi8(down, zero),_mm_unpackhi_epi8(up, zero));
                {
                    __m128i part3 = _mm_unpacklo_epi16(hdiff, vdiff);
                    shift_left_7(part3);
                    _mm_stream_si128((__m128i*)(out+8), part3);
                }
                {
                    __m128i part4 = _mm_unpackhi_epi16(hdiff, vdiff);
                    shift_left_7(part4);
                    _mm_stream_si128((__m128i*)(out+12), part4);
                }
            }
            curr += 16;
            out += 16;
        }
        _mm_empty();
    }

    void gradient(const BasicImage<byte>& im, BasicImage<short[2]>& out)
    {
		if( im.size() != out.size())
			throw Exceptions::Vision::IncompatibleImageSizes("gradient");

		if (is_aligned<16>(im.data()) && is_aligned<16>(out.data()) && im.size().x % 16 == 0)
		{
			zeroBorders(out);
			gradient(im.data(), out.data(), im.size().x, im.size().y);
		}
		else
			gradient<byte,short[2]>(im,out);

    }

};
