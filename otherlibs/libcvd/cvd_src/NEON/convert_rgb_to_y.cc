#include <cvd/image_convert.h>
#include <arm_neon.h>

namespace CVD {

	namespace Internal {
		void convert_rgb_to_grey_NEON(const BasicImage<Rgb<byte> >& from, BasicImage<byte>& to, const unsigned r, const unsigned g, const unsigned b) {
			const uint8x8_t red = vdup_n_u8(r);
			const uint8x8_t green = vdup_n_u8(g);
			const uint8x8_t blue = vdup_n_u8(b);
			
			for( int y = 0; y < from.size().y; ++y){
				const Rgb<byte> * in = from[y];
				byte * out = to[y];
				for( int x = 0; x < from.size().x; x+=8, in += 8, out += 8){
					uint8x8x3_t in_data  = vld3_u8((const uint8_t *)in);

					uint16x8_t sum = vmull_u8(in_data.val[0], red);
					sum = vmlal_u8(sum, in_data.val[1], green);
					sum = vmlal_u8(sum, in_data.val[2], blue);

					uint8x8_t final_sum = vshrn_n_u16(sum, 8); // divide by 256
					vst1_u8(out, final_sum);
				}
			}
		}
	}

	void ConvertImage<Rgb<byte>, byte, Pixel::CIE<Rgb<byte>, byte>, 1>::convert(const BasicImage<Rgb<byte> >& from, BasicImage<byte>& to) 
	{

		if((from.size().x % 8) == 0){
            // red (77) + green (150) + blue (29) = 256
            Internal::convert_rgb_to_grey_NEON( from, to, 77, 150, 29);
		} else {
			const int multiple_of_8_width = (from.size().x / 8) * 8;
			const ImageRef end_fast(multiple_of_8_width, from.size().y);

            // red (77) + green (150) + blue (29) = 256
			Internal::convert_rgb_to_grey_NEON( from.sub_image(ImageRef_zero, end_fast), to.sub_image(ImageRef_zero, end_fast).ref(), 77, 150, 29);
			
			for(int y = 0; y < from.size().y; ++y){
				const Rgb<byte> * in = from[y]+multiple_of_8_width;
				byte * out = to[y]+multiple_of_8_width;
				for(int x = multiple_of_8_width; x < from.size().x; ++x, ++in, ++out){
					Pixel::CIE<Rgb<byte>,byte>::convert(*in, *out);
				}
			}
		}
	}
}
