#include "cvd/colourspace_convert.h"


namespace CVD {
	namespace {
		void yuv420p_to_rgb_c(const unsigned char* y, unsigned char* rgb, unsigned int width, unsigned int height)
		{
			const byte* u = y + width*height;
			const byte* v = u + width*height/4;

			unsigned int halfwidth = width >> 1;
			int yy, uu, vv, ug_plus_vg, ub, vr;
			int r,g,b;
			unsigned int i,j;
			for (i=0; i<height; ++i) {
				for (j=0;j<halfwidth;++j) {
					yy = y[0] << 8;
					uu = *u - 128;
					vv = *v - 128;
					ug_plus_vg = uu * 88 + vv * 183;
					ub = uu * 454;
					vr = vv * 359;
					r = (yy + vr) >> 8;
					g = (yy - ug_plus_vg) >> 8;
					b = (yy + ub) >> 8;
					rgb[0] = r < 0 ? 0 : (r > 255 ? 255 : (unsigned char)r);
					rgb[1] = g < 0 ? 0 : (g > 255 ? 255 : (unsigned char)g);
					rgb[2] = b < 0 ? 0 : (b > 255 ? 255 : (unsigned char)b);
					yy = y[1] << 8;
					r = (yy + vr) >> 8;
					g = (yy - ug_plus_vg) >> 8;
					b = (yy + ub) >> 8;
					rgb[3] = r < 0 ? 0 : (r > 255 ? 255 : (unsigned char)r);
					rgb[4] = g < 0 ? 0 : (g > 255 ? 255 : (unsigned char)g);
					rgb[5] = b < 0 ? 0 : (b > 255 ? 255 : (unsigned char)b);

					y += 2;
					rgb += 6;
					u++;
					v++;
				}
				if ((i&1) == 0) {
					u -= halfwidth;
					v -= halfwidth;
				}
			}
		}

	}


	template<> void convert_image(const BasicImage<yuv420p>& from, BasicImage<Rgb<byte> >& to)
	{
		if (from.size() != to.size())
			throw Exceptions::Image::IncompatibleImageSizes(__FUNCTION__);

		const byte* yuv420 = static_cast<const byte*>(from.data());
		yuv420p_to_rgb_c(yuv420, (byte*)to.data(), from.size().x, from.size().y);
	}

	template<> void convert_image(const BasicImage<yuv420p>& from, BasicImage<byte>& to)
	{
		if (from.size() != to.size())
			throw Exceptions::Image::IncompatibleImageSizes(__FUNCTION__);
		
		memcpy(to.data(), from.data(), to.size().area());
	}	
}
