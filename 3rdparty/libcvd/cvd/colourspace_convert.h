#ifndef CVD_INCLUDE_COLOURSPACE_CONVERT_H
#define CVD_INCLUDE_COLOURSPACE_CONVERT_H
#include <cvd/image_convert_fwd.h>
#include <cvd/colourspaces.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>

namespace CVD
{
		
	/// Convert Bayer pattern of various forms to greyscale data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr>& from, BasicImage<byte>& to);
	template<> void convert_image(const BasicImage<bayer_grbg>& from, BasicImage<byte>& to);
	template<> void convert_image(const BasicImage<bayer_gbrg>& from, BasicImage<byte>& to);
	template<> void convert_image(const BasicImage<bayer_rggb>& from, BasicImage<byte>& to);
	
	/// Convert Bayer pattern of various forms to rgb data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr>& from, BasicImage<Rgb<byte> >& to);
	template<> void convert_image(const BasicImage<bayer_grbg>& from, BasicImage<Rgb<byte> >& to);
	template<> void convert_image(const BasicImage<bayer_gbrg>& from, BasicImage<Rgb<byte> >& to);
	template<> void convert_image(const BasicImage<bayer_rggb>& from, BasicImage<Rgb<byte> >& to);

	/// Convert 16bit Bayer pattern of various forms to greyscale data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr16>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_grbg16>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_gbrg16>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_rggb16>& from, BasicImage<unsigned short>& to);

	/// Convert 16bit Bayer pattern of various forms to rgb data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr16>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_grbg16>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_gbrg16>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_rggb16>& from, BasicImage<Rgb<unsigned short> >& to);

	/// Convert 16bit big endian Bayer pattern of various forms to greyscale data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr16be>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_grbg16be>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_gbrg16be>& from, BasicImage<unsigned short>& to);
	template<> void convert_image(const BasicImage<bayer_rggb16be>& from, BasicImage<unsigned short>& to);

	/// Convert 16bit big endian Bayer pattern of various forms to rgb data
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<bayer_bggr16be>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_grbg16be>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_gbrg16be>& from, BasicImage<Rgb<unsigned short> >& to);
	template<> void convert_image(const BasicImage<bayer_rggb16be>& from, BasicImage<Rgb<unsigned short> >& to);

	template<> void convert_image(const BasicImage<MJPEG>& from, BasicImage<Rgb<byte> >& to);

	/// Convert YUV 411 pixel data to RGB
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	//template<> void convert_image(const BasicImage<yuv411>& from, BasicImage<Rgb<byte> >& to);


	/// Convert YUV 411 pixel data to Y only
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
//	template<> void convert_image(const BasicImage<yuv411>& from, BasicImage<byte>& to);
	
	/// Convert YUV 422 pixel data to RGB
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<Rgb<byte> >& to);
	  

	/// Convert YUV 422 pixel data to Y only
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<yuv422>& from, BasicImage<byte>& to);

	// Name changed from 'convert_image' to prevent conflict with previous convert_image
	// with same method signature.

	/// Convert YUV 411 pixel data to both Y and RGB
	/// @param from The input data
	/// @ingroup gImageIO
	//template<> std::pair<Image<byte>,Image<Rgb<byte> > > convert_image_pair(const BasicImage<yuv411>& from);
	
	
	/// Convert VUY 422 pixel data to RGB
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<Rgb<byte> >& to);
	
	/// Convert VUY 422 pixel data to Y only
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<vuy422>& from, BasicImage<byte>& to);	
	
	/// Convert YUV420p pixel data to RGB
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<yuv420p>& from, BasicImage<Rgb<byte>>& to);

	/// Convert YUV420p pixel data to Y only
	/// @param from The input data
	/// @param to The output data
	/// @ingroup gImageIO
	template<> void convert_image(const BasicImage<yuv420p>& from, BasicImage<byte>& to);


//	template<> struct IsConvertible<yuv411,      Rgb<byte> > { static const bool is=1; };
//	template<> struct IsConvertible<yuv411,      byte>       { static const bool is=1; };
	template<> struct IsConvertible<yuv420p,     Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<yuv420p,     byte>       { static const bool is=1; };
	template<> struct IsConvertible<yuv422,      Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<yuv422,      byte>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr,  byte>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg,  byte>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg,  byte>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb,  byte>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr,  Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg,  Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg,  Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb,  Rgb<byte> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr16  ,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg16  ,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg16  ,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb16  ,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr16  ,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg16  ,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg16  ,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb16  ,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr16be,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg16be,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg16be,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb16be,unsigned short>       { static const bool is=1; };
	template<> struct IsConvertible<bayer_bggr16be,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_grbg16be,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_gbrg16be,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<bayer_rggb16be,Rgb<unsigned short> > { static const bool is=1; };
	template<> struct IsConvertible<MJPEG,Rgb<byte> > { static const bool is=1; };
}

#endif
