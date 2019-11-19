#ifndef CVD_IMAGE_CONVERT_H
#define CVD_IMAGE_CONVERT_H

#include <cvd/internal/convert_pixel_types.h>
#include <cvd/internal/rgb_components.h>
#include <cvd/image.h>
#include <type_traits>

namespace CVD
{

	// The most general case: one row at a time

	template <class From, class To, class Conv=typename Pixel::DefaultConversion<From,To>::type, int both_pod=std::is_trivially_copyable<From>::value && std::is_trivially_copyable<To>::value> struct ConvertImage {
		static void convert(const BasicImage<From>& from, BasicImage<To>& to) {
			for (int r=0; r<from.size().y; r++)
				Pixel::ConvertPixels<From,To,Conv>::convert(from[r], to[r], from.size().x);
		};
	};

	// The blat case: memcpy all data at once 
	template <class T> struct ConvertImage<T,T,Pixel::GenericConversion<T,T>,1> {
		static void convert(const BasicImage<T>& from, BasicImage<T>& to) {
			for(int y=0; y < from.size().y; y++)
				memcpy(to[y], from[y], from.size().x * sizeof(T));
		};
	};

	template <> struct ConvertImage<Rgb<byte>, byte, Pixel::CIE<Rgb<byte>, byte>, 1> {
		static void convert(const BasicImage<Rgb<byte> >& from, BasicImage<byte>& to);
	};

	template<class Conv, class C, class D> void convert_image(const BasicImage<C>& from, BasicImage<D>& to)
	{
		if (from.size() != to.size())
			throw Exceptions::Image::IncompatibleImageSizes(__FUNCTION__);
		ConvertImage<C,D,Conv>::convert(from, to);
	}

	template<template <class From, class To> class Conv, class C, class D> void convert_image(const BasicImage<C>& from, BasicImage<D>& to)
	{
		if (from.size() != to.size())
			throw Exceptions::Image::IncompatibleImageSizes(__FUNCTION__);
		ConvertImage<C,D,Conv<C,D> >::convert(from, to);
	}

	template<class C, class D> void convert_image(const BasicImage<C>& from, BasicImage<D>& to)
	{
		if (from.size() != to.size())
			throw Exceptions::Image::IncompatibleImageSizes(__FUNCTION__);
		ConvertImage<C,D>::convert(from, to);
	}

	/// Convert an image from one type to another using a specified conversion.
	/// @param D The destination image pixel type
	/// @param C The source image pixel type
	/// @param Conv The conversion to use
	/// @param from The image to convert from
	/// @ingroup gImageIO
	template<class D, class Conv, class C> Image<D> convert_image(const BasicImage<C>& from)
	{
		Image<D> to(from.size());
		convert_image<Conv>(from, to);
		return to;
	}

	template<class D, template <class From, class To> class Conv, class C> Image<D> convert_image(const BasicImage<C>& from)
	{
		Image<D> to(from.size());
		convert_image<Conv>(from, to);
		return to;
	}

	/// Convert an image from one type to another using the default.
	/// @param D The destination image pixel type
	/// @param C The source image pixel type
	/// @param from The image to convert from
	/// @ingroup gImageIO
	template<class D, class C> Image<D> convert_image(const BasicImage<C>& from)
	{
		Image<D> to(from.size());
		convert_image(from, to);
		return to;
	}

	// Function name changed from 'convert_image' to prevent compile-time
	// error arising from the clash with a function of same name declared above.

	/// Convert an image from one type to another using the default, returning a pair of images.
	/// @param D1 The first destination image pixel type
	/// @param D2 The second destination image pixel type
	/// @param C The source image pixel type
	/// @param from The image to convert from
	/// @ingroup gImageIO
	template<class D1, class D2, class C> std::pair<Image<D1>, Image<D2> > convert_image_pair(const BasicImage<C>& from)
	{
		std::pair<Image<D1>, Image<D2> > to(Image<D1>(from.size()), Image<D2>(from.size()));
		convert_image(from, to.first);
		convert_image(from, to.second);
		return to;
	}  


#ifndef DOXYGEN_IGNORE_INTERNAL
	namespace Internal
	{
		template<class C> class ImageConverter{};
		template<class C>  struct ImagePromise<ImageConverter<C> >
		{
			ImagePromise(const BasicImage<C>& im)
				:i(im)
			{}

			const BasicImage<C>& i;
			template<class D> void execute(Image<D>& j)
			{
				j.resize(i.size());
				convert_image(i, j);
			}
		};
	};
	template<class C> Internal::ImagePromise<Internal::ImageConverter<C> > convert_image(const BasicImage<C>& c)
	{
		return Internal::ImagePromise<Internal::ImageConverter<C> >(c);
	}
#else
	///Convert an image from one type to another using the default.
	///Type deduction is automatic, and D does not need to be specified. The following usage will work:
	///
	/// \code
	/// Image<byte> a;
	/// Image<byte> b;
	/// ...
	/// b = convert_image(a);
	/// \endcode
	/// Note that this is performed using lazy evaluation, so convertion happens on evaluation of assignment.
	/// @param D The destination image pixel type
	/// @param C The source image pixel type
	/// @param from The image to convert from
	/// @return The converted image
	/// @ingroup gImageIO
	template<class D> Image<D> convert_image(const BasicImage<C>& from);

#endif
	/// Can two types be converted with CVD::convert_image?
	/// @ingroup gImageIO
	template<class In, class Out> struct IsConvertible
	{ 
		static const bool is=Pixel::DefaultConvertible<In>::is & Pixel::DefaultConvertible<Out>::is;
	};

	/// Can individual pixels of two types be converted with ConvertPixels::convert()?
	/// E.g. Bayer patterns or YUV411 can usually not.
	/// @ingroup gImageIO
	template<class In, class Out> struct PixelByPixelConvertible
	{ 
		static const bool is=Pixel::DefaultConvertible<In>::is & Pixel::DefaultConvertible<Out>::is;
	};

	/// Identity conversion by memcpy is always supported
	template<class InOut> struct PixelByPixelConvertible<InOut, InOut>
	{ 
		static const bool is=1;
	};

}

#endif
