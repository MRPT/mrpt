#include "cvd/image.h"
#include "cvd/utility.h"
#include "cvd/colourspace.h"
#include "cvd/colourspaces.h"
#include "cvd/image_convert.h"


#include <iostream>
namespace CVD {
	
	namespace{
		unsigned char saturate(int i)
		{
			if(i<0)
				return 0;
			else if(i>255)
				return 255;
			else 
				return i;
		}

		struct yuv422_ind{
			static const int y1 = 0;
			static const int uu = 1;
			static const int y2 = 2;
			static const int vv = 3;
		};

		struct vuy422_ind{
			static const int y1=1;
			static const int uu=0;
			static const int vv=2;
			static const int y2=3;
		};

		template<class C, class Ind>
		void convert_422(const BasicImage<C>& from, BasicImage<Rgb<byte> >& to)
		{
			int yy, uu, vv, ug_plus_vg, ub, vr;
			int r,g,b;

			size_t bytes_per_row = from.size().x * 2;

			for(int y=0; y < from.size().y; y++)
			{
				const unsigned char* yuv = reinterpret_cast<const unsigned char*>(from.data()) + bytes_per_row*y;

				for(int x=0; x < from.size().x; x+=2, yuv+=4)
				{
					uu = yuv[Ind::uu] - 128;
					vv = yuv[Ind::vv] - 128;
					ug_plus_vg = uu * 88 + vv * 183;
					ub = uu * 454;
					vr = vv * 359;

					yy = yuv[Ind::y1] << 8;
					r = (yy + vr) >> 8;
					g = (yy - ug_plus_vg) >> 8;
					b = (yy + ub) >> 8;
					to[y][x+0].red   = saturate(r);
					to[y][x+0].green = saturate(g);
					to[y][x+0].blue  = saturate(b);

					yy = yuv[Ind::y2] << 8;
					r = (yy + vr) >> 8;
					g = (yy - ug_plus_vg) >> 8;
					b = (yy + ub) >> 8;
					to[y][x+1].red   = saturate(r);
					to[y][x+1].green = saturate(g);
					to[y][x+1].blue  = saturate(b);
				}
			}
		}

		template<class C, class Ind> void convert_422_grey(const BasicImage<C>& from, BasicImage<byte>& to)
		{
			//yuv422 / vuy422 is along the lines of yuyv
			//which is 4 bytes for 2 pixels, i.e. 2 bytes per pixel
			
			size_t bytes_per_row = from.size().x * 2;

			for(int y=0; y < from.size().y; y++)
			{
				const unsigned char* yuv = reinterpret_cast<const unsigned char*>(from.data()) + bytes_per_row*y;
				for(int x=0; x < from.size().x; x+=2, yuv+=4)
				{
					to[y][x+0] = yuv[Ind::y1];
					to[y][x+1] = yuv[Ind::y2];
				}
			}
		}
	}

	template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<Rgb<byte> >& to)
	{
		convert_422<yuv422, yuv422_ind>(from, to);
	}

	template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<byte>& to)
	{
		convert_422_grey<yuv422, yuv422_ind>(from, to);
	}

	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<Rgb<byte> >& to)
	{
		convert_422<vuy422, vuy422_ind>(from, to);
	}

	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<byte>& to)
	{
		convert_422_grey<vuy422, vuy422_ind>(from, to);
	}
}
