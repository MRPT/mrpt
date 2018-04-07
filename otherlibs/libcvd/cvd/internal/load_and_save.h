#ifndef CVD_LOAD_AND_SAVE_H
#define CVD_LOAD_AND_SAVE_H

#include <iostream>
#include <string>
#include <typeinfo>
#include <map>
#include <memory>
#include <cvd/exceptions.h>
#include <cvd/image_convert.h>
#include <cvd/colourspace_convert.h>
#include <cvd/internal/convert_pixel_types.h>
#include <cvd/internal/name_CVD_rgb_types.h>
#include <cvd/internal/io/parameter.h>

namespace CVD {

	namespace Exceptions
	{
		/// %Exceptions specific to image loading and saving
		/// @ingroup gException
		namespace Image_IO
		{
			/// Base class for all Image_IO exceptions
			/// @ingroup gException
			struct All: public CVD::Exceptions::All
			{};

			/// This image type is not supported
			/// @ingroup gException
			struct UnsupportedImageType: public All
			{
				UnsupportedImageType();
			};

			/// The file ended before the image
			/// @ingroup gException
			struct EofBeforeImage: public All
			{
				EofBeforeImage();
			};

			/// The ifstream which the file is being read from is not open
			/// @ingroup gException
			struct IfstreamNotOpen: public All
			{
				IfstreamNotOpen();
			};

			/// The image was incorrect
			/// @ingroup gException
			struct MalformedImage: public All
			{
				MalformedImage(const std::string &); ///< Construct from a message string
			};

			/// The loaded image is not the right size
			/// @ingroup gException
			struct ImageSizeMismatch: public All
			{
				ImageSizeMismatch(const ImageRef& src, const ImageRef& dest); ///< Construct from the two sizes
			};

			/// Error writing the image
			/// @ingroup gException
			struct WriteError: public All
			{
				WriteError(const std::string& err); ///< Construct from a message string
			};

			/// Cannot seek in this stream
			/// @ingroup gException
			struct UnseekableIstream: public All
			{
				UnseekableIstream(const std::string& type); ///< Construct from a message string
			};

			/// Type mismatch reading the image (image data is either 8- or 16-bit, and it must be the same in the file)
			/// @ingroup gException
			struct ReadTypeMismatch: public All
			{
				ReadTypeMismatch(bool read8); ///< Constructor is passed <code>true</code> if it was trying to read 8-bit data
				ReadTypeMismatch(const std::string& available, const std::string& requested);
			};

			/// Type mismatch reading the image (image data is either 8- or 16-bit, and it must be the same in the file)
			/// @ingroup gException
			struct WriteTypeMismatch: public All
			{
				WriteTypeMismatch(const std::string& available, const std::string& requested);
			};
			
			/// An error occurred in one of the helper libraries
			/// @ingroup gException
			struct InternalLibraryError: public All
			{
				InternalLibraryError(const std::string& lib, const std::string err); ///< Construct from the library name and the error string
			};

			/// This image subtype is not supported
			/// @ingroup gException
			struct UnsupportedImageSubType: public All
			{
				UnsupportedImageSubType(const std::string &, const std::string&); ///< Construct from a subtype string and an error string
			};

			/// Error in opening file
			/// @ingroup gException
			struct OpenError: public All
			{
				OpenError(const std::string&, const std::string&, int); ///< Construct from the filename and the error number
			};


		}
	}
	
	namespace Internal
	{
		template<class C, int i> struct save_default_
		{
			static const bool use_16bit=1;
		};
		  
		template<class C> struct save_default_<C,1>
		{
			static const bool use_16bit=(CVD::Pixel::traits<typename CVD::Pixel::Component<C>::type>::bits_used) > 8;
		};
		  
		template<class C> struct save_default
		{
			static const bool use_16bit = save_default_<C, CVD::Pixel::traits<typename CVD::Pixel::Component<C>::type>::integral>::use_16bit;
		};


		////////////////////////////////////////////////////////////////////////////////	
		//Mechanisms for generic image loading
		//
		// Image readers are duck-types and must provide the following:
		// typedef  Types         Typelist containing types which can be loaded
		// string   datatype()    Stringified name of datatype on disk
		// string   name()        Name of the reader (JPEG, TIFF, etc)
		// ImageRef size()        size()
		// void get_raw_pixel_line(T) Where T is available for everything in Types
		// Constructor accepting istream;

		
		//Basic typelist.
		struct Head{};
		template<class A, class B> struct TypeList
		{
			typedef A Type;
			typedef B Next;
		};

		

		////////////////////////////////////////////////////////////////////////////////
		//
		// Read data and process if necessary. 
		// In the case where the in-memory and on-disk datatypes match, no processing 
		// is performed.
		template<class PixelType, class DiskPixelType, class ImageLoader> struct read_and_maybe_process
		{
			static void exec(BasicImage<PixelType>& im, ImageLoader& r)
			{
				Image<DiskPixelType> rowbuf(ImageRef(r.size().x, 1));

				for(int row = 0; row < r.size().y; row++)
				{
					r.get_raw_pixel_line(rowbuf.data());
					PixelType* rowptr;

					if(r.top_row_first())
						rowptr = im[row];
					else
						rowptr = im[im.size().y - row-1];

					Pixel::ConvertPixels<DiskPixelType, PixelType>::convert(rowbuf.data(), rowptr, r.size().x);
				}
			}
		};

		template<class PixelType, class ImageLoader> struct read_and_maybe_process<PixelType, PixelType, ImageLoader>
		{
			static void exec(BasicImage<PixelType>& im, ImageLoader& r)
			{
				for(int row = 0; row < r.size().y; row++)
					if(r.top_row_first())
						r.get_raw_pixel_line(im[row]);
					else
						r.get_raw_pixel_line(im[im.size().y - row-1]);
			}
		};

		template<class PixelType, class DiskPixelType, class ImageLoader> struct read_and_then_process
		{
			static void exec(BasicImage<PixelType>& im, ImageLoader& r)
			{
				Image<DiskPixelType> imgbuf(r.size());
				read_and_maybe_process<DiskPixelType,DiskPixelType, ImageLoader>::exec(imgbuf, r);

				convert_image(imgbuf, im);
			}
		};


		////////////////////////////////////////////////////////////////////////////////	
		//
		// Iterate over the typelist, and decide which type to load. 
		//
		template<class PixelType, class ImageLoader, class List > struct Reader
		{	
			static void read(BasicImage<PixelType>& im, ImageLoader& r)
			{
				if(r.datatype() == PNM::type_name<typename List::Type>::name())
				{
					//std::cout << "converting " << r.datatype() << " -> " << PNM::type_name<PixelType>::name() << " PixelByPixel: " << PixelByPixelConvertible<typename List::Type, PixelType>::is << " Convertible: " << IsConvertible<typename List::Type, PixelType>::is << std::endl;
					if (PixelByPixelConvertible<typename List::Type, PixelType>::is)
						read_and_maybe_process<PixelType, typename List::Type, ImageLoader>::exec(im, r);
					else if (IsConvertible<typename List::Type, PixelType>::is)
						read_and_then_process<PixelType, typename List::Type, ImageLoader>::exec(im, r);
					else
						throw CVD::Exceptions::Image_IO::ReadTypeMismatch(r.datatype(), PNM::type_name<PixelType>::name());
				}
				else
					Reader<PixelType, ImageLoader, typename List::Next>::read(im, r);
			}
		};

		template<class PixelType, class ImageLoader> struct Reader<PixelType, ImageLoader, Head>
		{
			static void read(BasicImage<PixelType>&, ImageLoader& r)
			{	
				throw Exceptions::Image_IO::UnsupportedImageSubType(r.name(), r.datatype() + " not yet supported");
			}
		};

		
		////////////////////////////////////////////////////////////////////////////////	
		//
		// Driver functions for loading images.
		//

		template<class T, class ImageLoader> void readImage(BasicImage<T>& im, ImageLoader& r)
		{
			Reader<T, ImageLoader, typename ImageLoader::Types>::read(im, r);
		}

		template <class T, class ImageLoader> void readImage(BasicImage<T>& im, std::istream& in)
		{
			ImageLoader loader(in);
			ImageRef size = loader.size();
			if (im.size() != size)
				throw Exceptions::Image_IO::ImageSizeMismatch(size, im.size());

			readImage(im, loader);
		}

		template <class T, class ImageLoader> void readImage(Image<T>& im, std::istream& in)
		{
		  ImageLoader loader(in);
		  im.resize(loader.size());
		  readImage(im, loader);
		}

		////////////////////////////////////////////////////////////////////////////////	
		//
		// Functions for attempting to choose an image type based on the datatype.
		// The template code provides information detailing the ideal image type.
		// The writer object is initialized with this information, and returns the type
		// that it will write.
		//
		// Unlike loading, where the type on disk is known only dynamically, the type
		// being saved and any necessary conversions can be deduced statically.
		//
		// The writer objects provide an interface with the following parts:
		//
		// template<class C> Outgoing::type              For a given incoming pixel type C, this is the outgoing type
		// constructor(ostream&, ImageRef, string type)  Construct an image writer for a given type
		// void write_raw_pixel_line(T*)                 Write pixels of type T.

	
		////////////////////////////////////////////////////////////////////////////////
		//
		// Select an outgoing type, convert if necessary and then save.
		//
		template<class Pixel, class ImageWriter, class OutgoingPixel> struct maybe_process_and_write
		{	
			static void write(std::ostream& os, const BasicImage<Pixel>& im, const std::map<std::string, Parameter<> >& p)
			{
				ImageWriter w(os, im.size(), CVD::PNM::type_name<OutgoingPixel>::name(), p);
				Image<OutgoingPixel> row(ImageRef(im.size().x, 1));

				if(w.top_row_first)
					for(int r=0; r < im.size().y; r++)
					{
						CVD::Pixel::ConvertPixels<Pixel, OutgoingPixel>::convert(im[r], row.data(), im.size().x);
						w.write_raw_pixel_line(row.data());
					}
				else
					for(int r=im.size().y-1; r >= 0; r--)
					{
						CVD::Pixel::ConvertPixels<Pixel, OutgoingPixel>::convert(im[r], row.data(), im.size().x);
						w.write_raw_pixel_line(row.data());
					}
			}
		};

		template<class Pixel, class ImageWriter> struct maybe_process_and_write<Pixel, ImageWriter, Pixel>
		{	
			static void write(std::ostream& os, const BasicImage<Pixel>& im, const std::map<std::string, Parameter<> >& p)
			{
				ImageWriter w(os, im.size(), CVD::PNM::type_name<Pixel>::name(), p);

				if(w.top_row_first)
					for(int r=0; r < im.size().y; r++)
						w.write_raw_pixel_line(im[r]);
				else
					for(int r=im.size().y-1; r >= 0; r--)
						w.write_raw_pixel_line(im[r]);

			}
		};

		template<class Pixel, class Writer> void writeImage(const BasicImage<Pixel>& im, std::ostream& o, const std::map<std::string, Parameter<> >& p)
		{
			maybe_process_and_write<Pixel, Writer, typename Writer::template Outgoing<Pixel>::type>::write(o, im, p);
		}

	
	}


}

#endif
