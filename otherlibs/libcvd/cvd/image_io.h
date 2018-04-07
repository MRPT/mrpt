#ifndef CVD_IMAGE_IO_H
#define CVD_IMAGE_IO_H

#include <cvd/config.h>
#include <cvd/exceptions.h>
#include <cvd/image_convert.h>
#include <cvd/internal/load_and_save.h>
#include <cvd/internal/name_builtin_types.h>
#include <cvd/internal/name_CVD_rgb_types.h>
#include <errno.h>
#include <memory>
#include <string>
#include <fstream>
#include <cctype>

#include <cvd/internal/io/pnm_grok.h>
#include <cvd/internal/io/save_postscript.h>
#include <cvd/internal/io/bmp.h>
#include <cvd/internal/io/fits.h>
#include <cvd/internal/io/text.h>
#include <cvd/internal/io/cvdimage.h>
#include <cvd/internal/io/jpeg.h>
#include <cvd/internal/io/tiff.h>
#include <cvd/internal/io/png.h>

namespace CVD
{
	


	////////////////////////////////////////////////////////////////////////////////
	//
	// Image loading
	//

	#ifndef DOXYGEN_IGNORE_INTERNAL
	namespace Internal
	{
		class ImageLoaderIstream{};
		template<> struct ImagePromise<ImageLoaderIstream>
		{
			ImagePromise(std::istream& is)
			:i(is){}

			std::istream& i;
			template<class C> void execute(Image<C>& im)
			{
				img_load(im, i);
			}
		};

		class ImageLoaderString{};
		template<> struct ImagePromise<ImageLoaderString>
		{
			ImagePromise(const std::string& ss)
			:s(ss){}

			const std::string& s;
			template<class C> void execute(Image<C>& im)
			{
				img_load(im, s);
			}
		};
	};
	#endif

	// This is not the real definition, but this is what it would look like if all
	// the macros were expanded. The real definition is above
	/// Contains the enumeration of possible image types
	namespace ImageType
	{
		/// Possible image types
		enum ImageType
		{	
			/// Placeholder type telling @ref save_image to deduce the type from the filename
			Automatic= -2,
			/// Unknown image type (can be returned by @ref string_to_image_type
			Unknown = -1,
			/// PNM image format (PBM, PGM or PPM).
			/// This writes 8 or 16 bit raw PGMs or PPMs. PBM is not currently supported
			PNM = 0 , 
			/// JPEG image format. This is a compressed (lossy) image format, but defaults to 95% quality, which has very few compression artefacts. This image type is only present if libjpeg is available.
			/// RGB and Greyscale JPEGs are supported
			#ifdef CVD_HAVE_JPEG
			JPEG = 1,
			#endif
			/// Windows BMP (or DIB) format. Uncompressed 8 bit grey scale and 24 bit RGB are supported.
			BMP = 2,
			/// PNG image format. 1, 8 and 16 bit, Greyscale, RGB and RGBA images are supported.
			/// This image type is only present if libpng is available.
			#ifdef CVD_HAVE_PNG
			PNG = 3,
			#endif
			/// TIFF image format. 1, 8, 16, 32 (float) and 64 (double) suported. Greyscale, RGB and RGBA supported.
			/// This image type is only present if libtiff is available. G4 FAX encoding is used for bools, otherwise
			/// "Deflate" compression is used.
			#ifdef CVD_HAVE_TIFF
			TIFF = 4,
			#endif
			/// Postscript  format. This outputs a bare PostScript image with the coordinate system set up 
			/// to that (x,y) corresponds to pixel (x,y), with (0,0) being at the top left of the pixel (0,0).
			/// The Y axis is therefore inverted compared to normal postscript drawing, but is image aligned.
			/// To get the drawing axes aligned with the centre of the pixels, write the postscript command
			/// ".5 .5 translate" after the image.
			/// The image data in encoded in ASCII-85 for portability. Helper functions are provided for
			/// generating EPS figures. See CVD::output_eps_header and CVD::output_eps_footer
			PS = 5,
			/// EPS format. This outputs a complete EPS (Encapsulated PostScript) figure.
			EPS = 6,
			/// Plain text format. Grey-scale floating point only. This can be read in to MATLAB
			/// with the load() function. There is no metadata, so it is not possible to support 
			/// multiple types. 
			TXT = 7, TEXT=7,
			/// CVD image format. 8 bit Grey-scale, RGB and RGBA
			/// images are currently supported. Files of this type
			/// have a simple PNM-like header, but use a lossless
			/// compression scheme (line-wise prediction + Huffman).
			CVD = 8,
			/// FITS images. Supports (native) byte, short, unsigned short, int, float and double
			/// of greyscale, RGB and RGBA. Signed char is supported lossley but inefficiently. 
			FITS = 9,
		};
	}

	#ifndef DOXYGEN_IGNORE_INTERNAL

	Internal::ImagePromise<Internal::ImageLoaderIstream> img_load(std::istream& i);
	Internal::ImagePromise<Internal::ImageLoaderString> img_load(const std::string &s);
	#endif

	#if DOXYGEN_INCLUDE_ONLY_FOR_DOCS
	
	/// Load an image from an istream, and return the image.
	/// The template type is deduced automatically, and must not be specified.
	///
	/// The type deduction is performed using lazy evaluation, so the load operation 
	/// is only performed if an image is assigned from this.
	///
	/// @param i The istream to load from
	/// @ingroup gImageIO
	template<class C> Image<C> img_load(std::istream& i);
	
	/// Load an image from a file, and return the image.
	/// The template type is deduced automatically, and must not be specified.
	///
	/// The type deduction is performed using lazy evaluation, so the load operation 
	/// is only performed if an image is assigned from this.
	///
	/// @param i The istream to load from
	/// @ingroup gImageIO
	template<class C> Image<C> img_load(std::string& i);



	#endif	


	/// Load an image from a stream. This function resizes the Image as necessary.
	/// It will also perform image type conversion (e.g. colour to greyscale)
	/// according the Pixel:::CIE conversion.
	/// @param I The pixel type of the image
	/// @param im The image to receive the loaded image data
	/// @param i The stream
	/// @ingroup gImageIO
	template<class I> void img_load(Image<I>& im, std::istream& i)
	{
	  if(!i.good())
	  {
	    //Check for one of the commonest errors and put in
	    //a special case
	    std::ifstream* fs;
	    if((fs = dynamic_cast<std::ifstream*>(&i)) && !fs->is_open())
	      throw Exceptions::Image_IO::IfstreamNotOpen();
	    else
	      throw Exceptions::Image_IO::EofBeforeImage();
	  }
	  unsigned char c = i.peek();
	  
	  if(!i.good())
	    throw Exceptions::Image_IO::EofBeforeImage();

	  if(c == 'P')
	    CVD::Internal::readImage<I, PNM::Reader>(im, i);
#ifdef CVD_HAVE_JPEG
	  else if(c == 0xff)
	    CVD::Internal::readImage<I, JPEG::reader>(im, i);
#endif
#ifdef CVD_HAVE_TIFF
	  else if(c == 'I' || c == 'M') //Little or big endian TIFF
	    CVD::Internal::readImage<I, TIFF::tiff_reader>(im, i);
#endif
#ifdef CVD_HAVE_PNG
	  else if(c == 0x89)
	    CVD::Internal::readImage<I, PNG::png_reader>(im, i);
#endif
	  else if(c == 'B')
	    CVD::Internal::readImage<I, BMP::Reader>(im, i);
	  else if(c == 'S')
	    CVD::Internal::readImage<I, FITS::reader>(im, i);
	  else if(c == 'C')
		CVD::Internal::readImage<I, CVDimage::reader>(im, i);
	  else if(c == ' ' || c == '\t' || isdigit(c) || c == '-' || c == '+')
	    CVD::Internal::readImage<I, TEXT::reader>(im, i);
	  else
	    throw Exceptions::Image_IO::UnsupportedImageType();
	}
		
	//  syg21
	template<class I> void img_load(Image<I> &im, const std::string &s)
	{
	  std::ifstream i(s.c_str(), std::ios::in|std::ios::binary);

	  if(!i.good())
	    throw Exceptions::Image_IO::OpenError(s, "for reading", errno);
	  img_load(im, i);
	}	


	////////////////////////////////////////////////////////////////////////////////
	//
	// Image saving
	//


	/// Deduce an image type from a filename suffix.
	///	@param name The name of the image file
	ImageType::ImageType string_to_image_type(const std::string& name);


	/// Save an image to a stream. This function will convert types if necessary.
	/// @param PixelType The pixel type of the image
	/// @param Conversion The conversion class to use
	/// @param im The image to save
	/// @param o The stream 
	/// @param t The image file format to use (see ImageType::ImageType for a list of supported formats)
	/// @ingroup gImageIO
	template<class PixelType> 
	void img_save(const BasicImage<PixelType>& im, std::ostream& o, ImageType::ImageType t, const std::map<std::string, Parameter<> >& p = std::map<std::string, Parameter<> >())
	{
	  switch (t) {
	  default:
	  case ImageType::PNM:  
	  case ImageType::Automatic:
	  case ImageType::Unknown:
		Internal::writeImage<PixelType, PNM::Writer>(im, o, p); break;
	  #ifdef CVD_HAVE_JPEG
		  case ImageType::JPEG: Internal::writeImage<PixelType, JPEG::writer>(im,o, p); break;
	  #endif
	  #ifdef CVD_HAVE_PNG
		  case ImageType::PNG: Internal::writeImage<PixelType, PNG::png_writer>(im,o, p); break;
	  #endif
	  #ifdef CVD_HAVE_TIFF
		  case ImageType::TIFF: Internal::writeImage<PixelType, TIFF::tiff_writer>(im,o, p); break;
	  #endif
	  case ImageType::FITS: Internal::writeImage<PixelType, FITS::writer>(im,o, p); break;
	  case ImageType::BMP:  Internal::writeImage<PixelType, BMP::Writer>(im, o, p); break;
	  case ImageType::TXT:  Internal::writeImage<PixelType, TEXT::writer>(im, o, p); break;
	  case ImageType::PS:   Internal::writeImage<PixelType, PS::writer>(im, o, p); break;
	  case ImageType::EPS:  Internal::writeImage<PixelType, PS::eps_writer>(im, o, p); break;
	  case ImageType::CVD:  Internal::writeImage<PixelType, CVDimage::writer>(im,o, p); break;
	  }
	}

	template<class PixelType> void img_save(const BasicImage<PixelType>& im, const std::string& name, ImageType::ImageType t, ImageType::ImageType d = ImageType::PNM, const std::map<std::string, Parameter<> >& p = std::map<std::string, Parameter<> >())
	{
	  std::ofstream out(name.c_str(), std::ios::out|std::ios::binary);
	  if(!out.good())
	    throw Exceptions::Image_IO::OpenError(name, "for writing", errno);

	  if(t == ImageType::Automatic)
	  {
		t = string_to_image_type(name);
		if(t == ImageType::Unknown)
			t = d;
	  }

	  img_save(im, out, t, p);
	}

	template<class PixelType> void img_save(const BasicImage<PixelType>& im, const std::string& name, const std::map<std::string, Parameter<> >& p = std::map<std::string, Parameter<> >())
	{
	    img_save(im, name, ImageType::Automatic, ImageType::PNM, p);
	}

	////////////////////////////////////////////////////////////////////////////////
	//
	// Legacy pnm_* functions
	//

	/// Save an image to a stream as a PNM. 
	/// @b Deprecated Use img_save() instead, i.e. <code>img_save(im, o, ImageType::PNM);</code>
	/// @param PixelType The pixel type of the image
	/// @param im The image
	/// @param o The stream
	/// @ingroup gImageIO
	template<class PixelType> void pnm_save(const BasicImage<PixelType>& im, std::ostream& o)
	{
		img_save(im, o, ImageType::PNM);
	}

	/// Load a PNM image from a stream.
	/// @b Deprecated Use img_load(Image<I>& im, std::istream& i) instead. This can handle 
	/// and automatically detect other file formats as well.
	///
	///	Loading is simplistic, and automatic conversions of values are performed.
	/// So, for instance bytes are assumed to be in the range 0--255 and floats in
	/// the range 0--1, so loading bytes in to an Image<float> will result in an
	/// image with floats in the range 0--1. In the reverse direction, 
	/// floats falling outside the range 0--1, will wrap when converted to bytes.
	///
	/// The image loader ignores the data range given by the file, if it is given.
	///
	/// @param PixelType The pixel type of the image
	/// @param im The image
	/// @param i The stream
	/// @ingroup gImageIO
	template<class PixelType> void pnm_load(Image<PixelType>& im, std::istream& i)
	{
		img_load(im, i);
	}

	////////////////////////////////////////////////////////////////////////////////
	//
	// Postscript helper functions
	//

	/// Outputs an EPS footer to an ostream
	/// @param o the ostream
	/// @ingroup gImageIO
	void output_eps_footer(std::ostream& o);

	/// Outputs an EPS header to an ostream. Typical use is to output the header,
	/// save a raw PS image, output some other PS (eg annotations) and the output
	/// the EPS footer.
	/// @param o the ostream
	/// @param xs the width of the image
	/// @param ys the height of the image
	/// @ingroup gImageIO
	void output_eps_header(std::ostream& o, int xs, int ys);


	/// Outputs an EPS header to an ostream. Typical use is to output the header,
	/// save a raw PS image, output some other PS (eg annotations) and the output
	/// the EPS footer.
	/// @param o the ostream
	/// @param s size  of the image
	/// @ingroup gImageIO
	void output_eps_header(std::ostream & o, const ImageRef& s);

	/// Outputs an EPS header to an ostream. Typical use is to output the header,
	/// save a raw PS image, output some other PS (eg annotations) and the output
	/// the EPS footer.
	/// @param o the ostream
	/// @param im the image
	/// @ingroup gImageIO
	template<class PixelType> void output_eps_header(std::ostream& o, const BasicImage<PixelType>& im)
	{
		output_eps_header(o, im.size());
	}



}

#endif
