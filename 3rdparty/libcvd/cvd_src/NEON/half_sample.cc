#include "cvd/vision.h"

#include <arm_neon.h>

namespace CVD
{
	namespace Internal{
				
		void halfSampleNEON( const BasicImage<byte> & in, BasicImage<byte> & out ){
			for( int y = 0; y < in.size().y; y += 2){
				const byte * in_top = in[y];
				const byte * in_bottom = in[y+1];
				byte * out_data = out[y >> 1];
				for( int x = in.size().x; x > 0 ; x-=16, in_top += 16, in_bottom += 16, out_data += 8){
					uint8x8x2_t top  = vld2_u8((const uint8_t *)in_top);
					uint8x8x2_t bottom = vld2_u8((const uint8_t *)in_bottom);
					uint16x8_t sum = vaddl_u8( top.val[0], top.val[1]); 
					sum = vaddw_u8( sum, bottom.val[0] ); 
					sum = vaddw_u8( sum, bottom.val[1] ); 
					uint8x8_t final_sum = vshrn_n_u16(sum, 2);
					vst1_u8(out_data, final_sum);
				}
			}
		}
	}

	void halfSample(const BasicImage<byte>& in, BasicImage<byte>& out)
	{   
		if( (in.size()/2) != out.size())
			throw Exceptions::Vision::IncompatibleImageSizes("halfSample");

		if ((in.size().x % 16) == 0)
			Internal::halfSampleNEON(in, out);
		else
			halfSample<byte>(in, out);
	}
}
