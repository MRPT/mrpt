//FIXME!!
#include <cvd/image_convert_fwd.h>
#include <cvd/colourspaces.h>
#include <cvd/colourspace.h>
#include <cvd/colourspace_convert.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

namespace CVD
{

/*

  template<> void convert_image(const BasicImage<yuv411>& from, BasicImage<Rgb<byte> >& to)
  {
    ColourSpace::yuv411_to_rgb(reinterpret_cast<const unsigned char*>(from.data()),
			       from.totalsize(),
			       reinterpret_cast<unsigned char*>(to.data()));
  }

  template<> void convert_image(const BasicImage<yuv411>& from, BasicImage<byte>& to)
  {
    ColourSpace::yuv411_to_y(reinterpret_cast<const unsigned char*>(from.data()),
			     from.totalsize(),
			     reinterpret_cast<unsigned char*>(to.data()));
  }

  template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<Rgb<byte> >& to)
  {
    ColourSpace::yuv422_to_rgb(reinterpret_cast<const byte*>(from.data()), reinterpret_cast<byte*>(to.data()), from.size().x, from.size().y);
  }

  template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<byte>& to)
  {
    ColourSpace::yuv422_to_grey(reinterpret_cast<const byte*>(from.data()), to.data(), from.size().x, from.size().y);
  }

  // Name changed from 'convert_image' to prevent conflict with previous convert_image
  // with same method signature.
  template<> std::pair<Image<byte>,Image<Rgb<byte> > > convert_image_pair(const BasicImage<yuv411>& from)
  {
    Image<byte> rety(from.size());
    Image<Rgb<byte> > retc(from.size());

    ColourSpace::yuv411_to_rgb_y(reinterpret_cast<const unsigned char*>(from.data()),
				 from.totalsize(),
				 reinterpret_cast<unsigned char*>(retc.data()),
				 reinterpret_cast<unsigned char*>(rety.data()));

    return std::pair<Image<byte>,Image<Rgb<byte> > >(rety, retc);

  }
	
	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<Rgb<byte> >& to)
	{
		ColourSpace::vuy422_to_rgb(reinterpret_cast<const byte*>(from.data()), reinterpret_cast<byte*>(to.data()), from.size().x, from.size().y);
	}

	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<byte>& to)
	{
		ColourSpace::vuy422_to_grey(reinterpret_cast<const byte*>(from.data()), to.data(), from.size().x, from.size().y);
	}	

	*/
}
