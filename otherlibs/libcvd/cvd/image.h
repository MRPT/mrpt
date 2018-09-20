//-*- c++ -*-
//////////////////////////////////////////////////////////////////////////
//                                                                      //
//  CVD::image.h                                                        //
//                                                                      //
//  Definitions for of template class CVD::Image, fast_image		//
//                                                                      //
//  derived from IPRS_* developed by Tom Drummond                       //
//	Reworked to provide class heirachy and memory managementby E. Rosten//
//                                                                      //
//////////////////////////////////////////////////////////////////////////

#ifndef CVD_IMAGE_H
#define CVD_IMAGE_H

#include <cstring>
#include <cvd/image_ref.h>
#include <cvd/exceptions.h>
#include <string>
#include <utility>
#include <type_traits>
#include <iterator>


namespace CVD {

namespace Exceptions {

  /// @ingroup gException
  namespace Image {
      /// Base class for all Image_IO exceptions
        /// @ingroup gException
        struct All: public CVD::Exceptions::All {};

        /// Input images have incompatible dimensions
        /// @ingroup gException
        struct IncompatibleImageSizes : public All {
            IncompatibleImageSizes(const std::string & function)
            {
                what = "Incompatible image sizes in " + function;
            };
        };

        /// Input ImageRef not within image dimensions
        /// @ingroup gException
        struct ImageRefNotInImage : public All {
            ImageRefNotInImage(const std::string & function)
            {
                what = "Input ImageRefs not in image in " + function;
            };
        };
    }
}

#ifndef DOXYGEN_IGNORE_INTERNAL
namespace Internal
{
	template<class C> struct ImagePromise
	{};
};
#endif

#ifdef CVD_IMAGE_DEBUG
	#define CVD_IMAGE_ASSERT(X,Y)  if(!(X)) throw Y()
	#define CVD_IMAGE_ASSERT2(X,Y,Z)  if(!(X)) throw Y(Z)
#else
	#define CVD_IMAGE_ASSERT(X,Y)
	#define CVD_IMAGE_ASSERT2(X,Y,Z)
#endif

/// Fatal image errors (used for debugging). These are not included in the
/// main CVD::Exceptions namespace since they are fatal errors which are 
/// only thrown if the library is compiled with <code>-D CVD_IMAGE_DEBUG</code>.
/// This compiles in image bounds checking (see CVD::BasicImage::operator[]())
/// and makes image accesses very slow!
/// @ingroup gException
namespace ImageError
{
	/// An attempt was made to access a pixel outside the image. Note that this is
	/// not derived from CVD::Exceptions::All.
	/// @ingroup gException
	class AccessOutsideImage{};
}


namespace Internal
{
	template<class T> inline void memfill(T* data, int n, const T val)
	{
		T* de = data + n;
		for(;data < de; data++)
			*data=val;
	}

	template<> inline void memfill(unsigned char* data, int n, const unsigned char val)
	{
		memset(data, val, n);
	}

	template<> inline void memfill(signed char* data, int n, const signed char val)
	{
		memset(data, val, n);
	}

	template<> inline void memfill(char* data, int n, const char val)
	{
		memset(data, val, n);
	}


	//Detecting a typedef...
	template<class C>
	struct IsDummy
	{
		//Fake function to pretend to return an instance of a type
		template<class S>
		static const C& get();

		//Two typedefs guaranteed to have different sizes
		typedef char One[1];
		typedef char Two[2];


		//Class to give us a size 2 return value or fail on 
		//substituting S
		template<int S> 
		struct SFINAE_dummy
		{
			typedef Two Type;
		};

		static One& detect_dummy(...);

		template<class S>
		static typename SFINAE_dummy<sizeof(typename S::dummy)>::Type& detect_dummy(const S&);

		static const bool Is = (sizeof(detect_dummy(get<C>()))==2);
	};
}


template<class T> class BasicImageIterator
{
	public:
		//Make it look like a standard iterator
		typedef std::forward_iterator_tag iterator_category;
		typedef T value_type;
		typedef std::ptrdiff_t difference_type;
		typedef T* pointer;
		typedef T& reference;

		const BasicImageIterator& operator++()
		{
			ptr++;
			if(ptr == row_end)
			{
				ptr += row_increment;
				row_end += total_width;

				if(ptr >= end)
					end = NULL;
			}
			return *this;
		}

		void operator++(int)
		{
			operator++();
		}
	
		T* operator->() const { return ptr; }
		T& operator*() const { return *ptr;}

		bool operator<(const BasicImageIterator& s) const 
		{ 
			//It's illegal to iterate _past_ end(), so < is equivalent to !=
			//for end iterators.
			if(is_end && s.is_end)
				return false;
			else if(is_end)
				return s.end != NULL;
			else if(s.is_end) 
				return end != NULL; 
			else 
				return ptr < s.ptr; 
		}

		bool operator==(const BasicImageIterator& s) const 
		{ 
			return !((*this)!=s);
		}

		bool operator!=(const BasicImageIterator& s) const 
		{ 
			if(is_end && s.is_end)
				return false;
			else if(is_end)
				return s.end != NULL;
			else if(s.is_end) 
				return end != NULL; 
			else 
				return ptr != s.ptr; 
		}



		BasicImageIterator()
		{}

		BasicImageIterator(T* start, int image_width, int row_stride, T* off_end)
		:ptr(start),
		 row_end(start + image_width), 
		 end(off_end), 
		 is_end(false),
		 row_increment(row_stride-image_width), 
		 total_width(row_stride)
		{ }

		//Prevent automatic conversion from a pointer (ie Image::iterator)
		explicit BasicImageIterator(T* end) 
		:ptr(end),is_end(true),row_increment(0),total_width(0)
		{ }

	protected:
		T* ptr;
		T *row_end, *end;
		bool is_end;
		int row_increment, total_width;
};

template<class C> class BasicImage;
template<class C> using SubImage = BasicImage<C>;

namespace Internal
{
	
	//The structure is a little funny. We use inheritance here as a way of conditional
	//namespacing. So think of it like that not subclassing :)
	//
	//Essentially there are two image "types". One is an image of pixels, and includes
	//all the per-pixel types like byte, Rgb<byte> float, etc. These have the properties
	//that there is precisely one element per pixel. That makes pixel access obvious and
	//straightforward, so slicing (sub images) works fine and so on. Types need not be POD
	//
	//The other type is for images of crud. This includes yuv422, yuv420p and MJPEG. These
	//have no easy way of accessing pixels in general and no wasy way of creating sub images.
	//Technically, the types like YUV422 can be sub-imaged and have strides, but I've got no
	//use case for that and no one wants it.
	//
	//Silly types have a typedef called "dummy" because they're a placeholder and so don't
	//actually represent the underlying data.
	//
	//Dummy types are usually of fixed size (e.g. yuv types) and so the size is/should be known
	//at compile time. In this case, the dummy struct must provide a std::ratio called bytes_per_pixel


	//This is the image data holder for silly types lilke 
	template<class T, bool D = Internal::IsDummy<T>::Is> class ImageData
	{
		public:

			ImageData(const ImageData&) = default;
			ImageData& operator=(const ImageData&)=default;
			
			ImageData(void* data, size_t len, ImageRef sz)
			:my_data(data),my_size(sz), data_length(len)
			{
			}

			ImageData(void* data, ImageRef sz)
			:my_data(data),my_size(sz), data_length(T::bytes_per_pixel::num*sz.area()/T::bytes_per_pixel::den)
			{
			}

			/// What is the row stride of the image?
			inline size_t datalength() const
			{
				return data_length;
			}
			
			ImageData()
			{
				my_data=0;
				my_size=ImageRef(0,0);
				data_length=0;
			}


		protected:

			void*    my_data;
			ImageRef my_size;
			size_t   data_length;

			using PointerType = void*;
			using ConstPointerType = const void*;
	};

	template<class T> class ImageData<T, false>
	{
		public:
			/// Construct an image from a block of data, assuming tight packing.
			/// @param data The image data in horizontal scanline order
			/// @param size The size of the image
			ImageData(T* data, const ImageRef& size)
			:my_data(data),my_size(size),my_stride(size.x)
			{
			}

			/// Construct an image from a block of data.
			/// @param data The image data in horizontal scanline order
			/// @param size The size of the image
			/// @param stride The row stride (or width, including the padding)
			ImageData(T* data, const ImageRef& size, int stride)
			:my_data(data),my_size(size),my_stride(stride)
			{
			}

			//We need them here in order to provide the functionality for
			//the asserts below. Technically they can apply to dummy types but
			//why bother?

			/// Is this pixel co-ordinate inside the image?
			/// @param ir The co-ordinate to test
			bool in_image(const ImageRef& ir) const
			{
				return ir.x >=0 && ir.y >=0 && ir.x < my_size.x && ir.y < my_size.y;
			}

			/// Is this pixel co-ordinate inside the image, and not too close to the edges?
			/// @param ir The co-ordinate to test
			/// @param border The size of the border: positive points inside the image.
			bool in_image_with_border(const ImageRef& ir, int border) const
			{
				return ir.x >=border && ir.y >=border && ir.x < my_size.x - border && ir.y < my_size.y - border;
			}

			/// Access a pixel from the image. Bounds checking is only performed if the library is compiled
			/// with <code>-D CVD_IMAGE_DEBUG</code>, in which case an ImageError::AccessOutsideImage exception is 
			/// thrown.
			inline T& operator[](const ImageRef& pos)
			{
				CVD_IMAGE_ASSERT(in_image(pos), ImageError::AccessOutsideImage);
				return (my_data[pos.y*my_stride + pos.x]);
			}
			
			/// Access a pixel from the image. Bounds checking is only performed if the library is compiled
			/// with <code>-D CVD_IMAGE_DEBUG</code>, in which case an ImageError::AccessOutsideImage exception is 
			/// thrown.
			inline const T& operator[](const ImageRef& pos) const 
			{
				CVD_IMAGE_ASSERT(in_image(pos), ImageError::AccessOutsideImage);
				return (my_data[pos.y*my_stride + pos.x]);
			}

			/// Access pointer to pixel row. Returns the pointer to the first element of the passed row.
			/// Allows to use [y][x] on images to access a pixel. Bounds checking is only performed if the library is compiled
			/// with <code>-D CVD_IMAGE_DEBUG</code>, in which case an ImageError::AccessOutsideImage exception is
			/// thrown.
			inline T* operator[](int row)
			{
				CVD_IMAGE_ASSERT(in_image(ImageRef(0,row)), ImageError::AccessOutsideImage);
				return my_data+row*my_stride;
			}

			/// Access pointer to pixel row. Returns the pointer to the first element of the passed row.
			/// Allows to use [y][x] on images to access a pixel. Bounds checking is only performed if the library is compiled
			/// with <code>-D CVD_IMAGE_DEBUG</code>, in which case an ImageError::AccessOutsideImage exception is
			/// thrown.
			inline const T* operator[](int row) const
			{
				CVD_IMAGE_ASSERT(in_image(ImageRef(0,row)), ImageError::AccessOutsideImage);
				return my_data+row*my_stride;
			}

			/// Given a pointer, this returns the image position as an ImageRef
			inline ImageRef pos(const T* ptr) const
			{
				int diff = ptr - my_data;
				return ImageRef(diff % my_stride, diff / my_size.x);
			}

			typedef BasicImageIterator<T> iterator;
			typedef BasicImageIterator<const T> const_iterator;

			/// The data type of the pixels in the image.
			typedef T value_type;
			
			/// Returns an iterator referencing the first (top-left) pixel in the image
			inline iterator begin()
			{
				return BasicImageIterator<T>(my_data, my_size.x, my_stride, end_ptr());
			}
			/// Returns a const iterator referencing the first (top-left) pixel in the image
			inline const_iterator begin() const
			{
				return BasicImageIterator<const T>(my_data, my_size.x, my_stride, end_ptr());
			}

			/// Returns an iterator pointing to one past the end of the image
			inline iterator end()
			{
				//Operator [] would always throw here!
				return BasicImageIterator<T>(end_ptr());
			}
			/// Returns a const iterator pointing to one past the end of the image
			inline const_iterator end() const
			{
				//Operator [] would always throw here!
				return BasicImageIterator<const T>(end_ptr());
			}


			/// What is the row stride of the image?
			inline int row_stride() const
			{
				return my_stride;
			}
			
			/// Return a sub image
			/// @param start Top left pixel of the sub image
			/// @param size width and  height of the sub image
			BasicImage<T> sub_image(const ImageRef& start, const ImageRef& size);

			/// Return a sub image
			/// @param start Top left pixel of the sub image
			/// @param size width and  height of the sub image
			const BasicImage<T> sub_image(const ImageRef& start, const ImageRef& size) const;

			ImageData(const ImageData&) = default;
			ImageData& operator=(const ImageData&)=default;

			ImageData()
			{
				my_data=nullptr;
				my_size = ImageRef(0,0);
				my_stride = 0;
			}

		protected:
				///Return an off-the-end pointer without ever throwing AccessOutsideImage
				T* end_ptr() { return my_data+my_size.y*my_stride; }

				///Return an off-the-end pointer without ever throwing AccessOutsideImage
				const T* end_ptr() const { return my_data+my_size.y*my_stride; }



				T* my_data;
				ImageRef my_size;
				int my_stride;	
				using PointerType = T*;
				using ConstPointerType = const T*;
	};

}

/// A generic image class to manage a block of arbitrarily padded data as an image. Provides
/// basic image access such as accessing a particular pixel co-ordinate. 
/// @param T The pixel type for this image. Typically either
/// <code>CVD::byte</code> or <code>CVD::Rgb<CVD::byte> ></code> are used,
/// but images could be constructed of any available type.
/// 
/// A BasicImage does not manage its own data, but provides access to an 
/// arbitrary externally-managed block of data as though it were an image. Use
/// the derived Image class if you want an image which also has its own data.
/// @ingroup gImage
template<class T> class BasicImage : public Internal::ImageData<T>
{
	static const bool IsDummy = Internal::IsDummy<T>::Is;
	protected:
		using Internal::ImageData<T>::my_size;
		using Internal::ImageData<T>::my_data;
		using typename Internal::ImageData<T>::PointerType;
		using typename Internal::ImageData<T>::ConstPointerType;


	public:
		
		//Inherit all constructors
		using Internal::ImageData<T>::ImageData;


		/// The image data is not destroyed when a BasicImage is destroyed.
		virtual ~BasicImage()
		{}


		/// Returns the raw image data
		inline ConstPointerType data() const
		{
			return my_data;
		}
		
		/// Returns the raw image data
		inline PointerType data()
		{
			return my_data;
		}

		inline void copy_from( const BasicImage<T> & other ){
			CVD_IMAGE_ASSERT2(other.size() == this->size(), Exceptions::Image::IncompatibleImageSizes, "copy_from");
			std::copy(other.begin(), other.end(), this->begin());
		}

		/// What is the size of this image?
		inline ImageRef size() const
		{
			return my_size;
		}

		///Set image data to all zero bytes. 
		///This only works on POD
		inline void  zero() 
		{
			static_assert(std::is_trivially_copyable<T>::value, "Error: zero() only works on POD types");
			for(int y=0; y < my_size.y; y++)
				memset((*this)[y], 0, sizeof(T) * my_size.x);
		}

		/// Set all the pixels in the image to a value. This is a relatively fast operation, using <code>memfill</code>.
		/// Safe to run on empty images.
		/// @param d The value to write into the image
		inline void fill(const T d)
		{
			for(int y=0; y < my_size.y; y++)
				Internal::memfill( (*this)[y], my_size.x, d);
		}

		/// Copy constructor
		/// @param copyof The image to copy
		BasicImage(const BasicImage& copyof)=default;
		
		/// Assignment operator
		/// @param copyof The image to copy
		BasicImage& operator=(const BasicImage&copyof)=default;

	protected:
		
		BasicImage()=default;

};


template<class C> BasicImage<C> Internal::ImageData<C, false>::sub_image(const ImageRef& start, const ImageRef& size)
{
	CVD_IMAGE_ASSERT(in_image(start), ImageError::AccessOutsideImage);
	CVD_IMAGE_ASSERT(in_image(start + size - ImageRef(1,1)), ImageError::AccessOutsideImage);
	return BasicImage<C>( &operator[](start), size, my_stride);
}

template<class C> const BasicImage<C> Internal::ImageData<C, false>::sub_image(const ImageRef& start, const ImageRef& size) const
{
	CVD_IMAGE_ASSERT(in_image(start), ImageError::AccessOutsideImage);
	CVD_IMAGE_ASSERT(in_image(start + size - ImageRef(1,1)), ImageError::AccessOutsideImage);

	C *ptr = my_data + start.y * my_stride + start.x;
	return BasicImage<C>( ptr , size, my_stride);
}

/// A full image which manages its own data.
/// @param T The pixel type for this image. Typically either
/// <code>CVD::byte</code> or <code>CVD::Rgb<CVD::byte> ></code> are used,
/// but images could be constructed of any available type.
///
/// The images behave much like 2D STL vectors. Iterators iterate over the pixels.
///
/// Loading and saving, format conversion and some copying functionality is
/// provided by external functions rather than as part of this class. See
/// the @ref gImageIO "Image loading and saving, and format conversion" module
/// for documentation of these functions.
/// @ingroup gImage
template<class T> 
class Image: public SubImage<T>
{
	private:
		struct CopyPlaceHolder
		{
			const Image* im;
		};

		using SubImage<T>::my_size;
		//using SubImage<T>::my_stride;
		using SubImage<T>::my_data;

	public:

		/// The data type of the pixels in the image.
		typedef T value_type;

		inline Image& copy_from( const SubImage<T> & other ){
			resize(other.size());
			std::copy(other.begin(), other.end(), this->begin());
			return *this;
		}

		///Copy constructor: new allocation and copy the data.
		///@param copy The image to copy
		Image(const Image& i)
		:Image(i.size())
		{
			copy_from(i);
		}
		

		///Move constructor: steal the pointer.
		Image(Image&& move_from)
		{
			static_cast<Internal::ImageData<T>&>(*this) = move_from;
			move_from.erase_fields();
		}

		/// Copy the data.
		///@param copyof The image to copy
		Image& operator=(const Image& i)
		{
			resize(i.size());
			copy_from(i);
			return *this;
		}

		Image& operator=(Image&& move_from)
		{
			static_cast<Internal::ImageData<T>&>(*this) = move_from;
			move_from.erase_fields();

			return *this;
		}


		
		#ifndef DOXYGEN_IGNORE_INTERNAL
		template<class C> const Image& operator=(Internal::ImagePromise<C> p)
		{
			p.execute(*this);
			return *this;
		}

		template<class C> Image(Internal::ImagePromise<C> p)
		:SubImage<T>()
		{
			p.execute(*this);
		}
		#endif
		
		///Default constructor: everything set to zero.
		Image()
		{
	
			erase_fields();
		}


		///Create an empty image of a given size.
		///@param size The size of image to create
		Image(const ImageRef& size)
		{
          resize(size);
		}

		///Create a filled image of a given size
		///@param size The size of image to create
		///@param val  The value to fill the image with
		Image(const ImageRef& size, const T& val)
		{
			resize(size);
			this->fill(val);
		}

		
		///Resize the image (destroying the data).
		///@param size The new size of the image
		void resize(const ImageRef& size)
		{	
			resize_(size, DD<Internal::IsDummy<T>::Is>{} );
		}

		///Resize the image (destroying the data). 
		//The resized image is filled with val.
		///@param size The new size of the image
		///@param val  The value to fill the image with
		void resize(const ImageRef& size, const T& val)
		{
			resize(size);
			fill(val);
		}

		///The destructor removes the image data
		~Image()
		{
			delete_old();
		}

		
	private:
		void delete_old()
		{
			delete_(DD<Internal::IsDummy<T>::Is>{});
			erase_fields();
		}
		
		void erase_fields()
		{
			static_cast<Internal::ImageData<T>&>(*this) = Internal::ImageData<T>();
		}
		
		//Allow type switching on dummy data: the internals are quite different
		//for dummy types.
		template<int Dummy> struct DD{};

		template<int Dummy> void resize_(const ImageRef& size, DD<Dummy>)
		{
			if(size == ImageRef(0,0))
			{
				delete_old();
			}
			else if(size != BasicImage<T>::my_size)
			{
				delete_old();
				static_cast<BasicImage<T>&>(*this) = BasicImage<T>(new T[size.area()], size);
			}
		}

		void resize_(const ImageRef& size, DD<true>)
		{
			if(size == ImageRef(0,0))
			{
				delete_old();
			}
			else if(size != BasicImage<T>::my_size)
			{
				delete_old();
				static_cast<BasicImage<T>&>(*this) = BasicImage<T>(nullptr, size);
				my_data = new char [this->datalength()];
			}
		}

		template<int Dummy> void delete_(DD<Dummy>)
		{
			delete[] my_data;
		}

		void delete_(DD<true>)
		{
			delete[] static_cast<char*>(my_data);
		}

};



} // end namespace
#endif
