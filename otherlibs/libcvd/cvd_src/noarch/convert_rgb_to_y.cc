#include <cvd/image_convert.h>
#include <cvd/utility.h>
namespace CVD {

	void ConvertImage<Rgb<byte>, byte, Pixel::CIE<Rgb<byte>, byte>, 1>::convert(const BasicImage<Rgb<byte> >& from, BasicImage<byte>& to) 
	{
		for(int y=0; y < from.size().y; y++)
			for(int x=0; x < from.size().x; x++)
				Pixel::CIE<Rgb<byte>,byte>::convert(from[y][x], to[y][x]);
	}
}
