#include "cvd/internal/io/png.h"
#include "cvd/image_io.h"
#include "cvd_src/config_internal.h"

#include <png.h>
#include <cstdlib>
#include <iostream>

using namespace CVD;
using namespace CVD::Exceptions;
using namespace CVD::Exceptions::Image_IO;
using namespace PNG;
using namespace std;


static void png_set_swap_if_necessary(png_structp png_ptr, int depth)
{
	#ifdef CVD_INTERNAL_ARCH_LITTLE_ENDIAN
		if(depth == 16)
			  png_set_swap(png_ptr);
	#elif defined CVD_INTERNAL_ARCH_BIG_ENDIAN
	#else 
		#error No endianness specified
	#endif
}

////////////////////////////////////////////////////////////////////////////////
//
// C++ istreams based I/O functions
// 
static void error_fn(png_structp png_ptr, png_const_charp error_msg)
{
	*(string*)png_get_error_ptr(png_ptr) = error_msg;
}

static void warn_fn(png_structp, png_const_charp )
{
}

static void read_fn(png_structp png_ptr, unsigned char*  data, size_t numbytes)
{
	istream* i = (istream*)png_get_io_ptr(png_ptr);
	i->read((char*) data, numbytes);

	//What to do on stream failure?
	//There is no return value, and I do not know if longjmp is safe here.
	//Anyway, multiple rereads of the data will cause the PNG read
	//to fail because it has internal checksums
}

static void write_fn(png_structp png_ptr, unsigned char*  data, size_t numbytes)
{
	ostream* o = (ostream*)png_get_io_ptr(png_ptr);
	o->write((char*) data, numbytes);
}

static void flush_fn(png_structp png_ptr)
{
	ostream* o = (ostream*)png_get_io_ptr(png_ptr);
	(*o) << flush;
}

////////////////////////////////////////////////////////////////////////////////
//
// Implementation of PNGPimpl
//

class CVD::PNG::PNGPimpl
{
	public:
		template<class C> void read_pixels(C*);
		PNGPimpl(std::istream& in);
		~PNGPimpl();
		std::string datatype();
		std::string name();
		ImageRef size();
	
	private:
		std::istream& i;
		std::string type;
		unsigned long row;
		png_structp png_ptr;
		png_infop info_ptr, end_info;

		std::string error_string;
		ImageRef my_size;
};

////////////////////////////////////////////////////////////////////////////////
//
// PNG reading functions
// 

string PNGPimpl::datatype()
{
	return type;
}

string PNGPimpl::name()
{
	return "PNG";
}

ImageRef PNGPimpl::size()
{
	return my_size;
}

#ifdef CVD_INTERNAL_VERBOSE_PNG
	#include <map>

	#define LOG(X) do{ cerr << X; }while(0)

	static string lookup_color_type(int i)
	{
		map<int, string> m;
		#define ADD(X) m[X] = #X
		ADD(PNG_COLOR_TYPE_GRAY);
		ADD(PNG_COLOR_TYPE_GRAY_ALPHA);
		ADD(PNG_COLOR_TYPE_PALETTE);
		ADD(PNG_COLOR_TYPE_RGB);
		ADD(PNG_COLOR_TYPE_RGB_ALPHA);
		ADD(PNG_COLOR_MASK_PALETTE);
		ADD(PNG_COLOR_MASK_COLOR);
		ADD(PNG_COLOR_MASK_ALPHA);

		return m[i];
	}
#else
	#define LOG(X)
#endif

template<class P> void PNGPimpl::read_pixels(P* data)
{
	if(datatype() != PNM::type_name<P>::name())
		throw ReadTypeMismatch(datatype(), PNM::type_name<P>::name());

	if(row  > (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Read past end of image.");


	if(setjmp(png_jmpbuf(png_ptr)))     
		throw Exceptions::Image_IO::MalformedImage(error_string);
	
	unsigned char* cptr = reinterpret_cast<unsigned char*>(data);
	unsigned char** row_ptr = &cptr;

	png_read_rows(png_ptr, row_ptr, NULL, 1);
}



PNGPimpl::PNGPimpl(std::istream& in)
:i(in),type(""),row(0),png_ptr(0),info_ptr(0),end_info(0)
{
	//Read the header and make sure it really is a PNG...
	unsigned char header[8];

	in.read((char*)header, 8);

	if(png_sig_cmp(header, 0, 8))
		throw Exceptions::Image_IO::MalformedImage("Not a PNG image");

	LOG("PNG header found\n");

	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);


	if(!png_ptr)
		throw Exceptions::OutOfMemory();
	
	info_ptr = png_create_info_struct(png_ptr);
	if(!info_ptr)
	{
		png_destroy_read_struct(&png_ptr, NULL, NULL);
		throw Exceptions::OutOfMemory();
	}

	end_info = png_create_info_struct(png_ptr);
	if(!info_ptr)
	{
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		throw Exceptions::OutOfMemory();
	}

	if(setjmp(png_jmpbuf(png_ptr)))     
	{         
		png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
		throw Exceptions::Image_IO::MalformedImage(error_string);
	}

	png_set_error_fn(png_ptr, &error_string, error_fn, warn_fn);

	//png_init_io(png_ptr, stdin);

	png_set_read_fn(png_ptr, &i, read_fn);


	png_set_sig_bytes(png_ptr, 8);
	
	png_read_info(png_ptr, info_ptr);

	png_uint_32 w, h;
	int colour, interlace, dummy, depth;

	png_get_IHDR(png_ptr, info_ptr, &w, &h, &depth, &colour, &interlace, &dummy, &dummy);

	LOG("w         = " << w << endl);
	LOG("h         = " << h << endl);
	LOG("depth     = " << depth << endl);
	LOG("colour    = " << colour<< ": " << lookup_color_type(colour) << endl);
	LOG("  palette = " << (colour & PNG_COLOR_MASK_PALETTE) << endl);
	LOG("  color   = " << (colour & PNG_COLOR_MASK_COLOR) << endl);
	LOG("  alpha   = " << (colour & PNG_COLOR_MASK_ALPHA) << endl);
	LOG("interlace = " << interlace<< endl);
	LOG("channels  = " << (int)png_get_channels(png_ptr, info_ptr) << endl);

	LOG("tRNS?     = " << png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS) << endl);
	
	my_size.x = w;
	my_size.y = h;

	//Figure out the type name, and what processing to to.
	if(depth == 1)
	{	
		//Unpack bools to bytes to ease loading.
		png_set_packing(png_ptr);
		type = PNM::type_name<bool>::name();
	}
	else if(depth <= 8)
	{
		//Expand nonbool colour depths up to 8bpp
		if(depth < 8)
			png_set_expand_gray_1_2_4_to_8(png_ptr);

		type = PNM::type_name<unsigned char>::name();
	}
	else
		type = PNM::type_name<unsigned short>::name();
	
	//Get rid of palette, by transforming it to RGB
	if(colour == PNG_COLOR_TYPE_PALETTE)
	{
		png_set_palette_to_rgb(png_ptr);

		//Check to see if there is a tRNS palette chunk. Note that the PNG_COLOR_MASK_ALPHA is
		//only valid for non indexed images, not paletted ones. So, we need to check here for
		//transparency data.
		if(png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
		{
			colour |= PNG_COLOR_MASK_ALPHA;
			png_set_tRNS_to_alpha(png_ptr);
		}
	}

	
	if(colour & PNG_COLOR_MASK_COLOR)
		if(colour & PNG_COLOR_MASK_ALPHA)
			type = "CVD::Rgba<" + type + ">";
		else
			type = "CVD::Rgb<" + type + ">";
	else
		if(colour & PNG_COLOR_MASK_ALPHA)
			type = "CVD::GreyAlpha<" + type + ">";
		else
			type = type;
	
	if(interlace != PNG_INTERLACE_NONE)
		throw Exceptions::Image_IO::UnsupportedImageSubType("PNG", "Interlace not yet supported");

	png_set_swap_if_necessary(png_ptr, depth);
}

PNGPimpl::~PNGPimpl()
{
	//Clear the stream of any remaining PNG bits.
	//It doesn't matter if there's an error here
	if(!setjmp(png_jmpbuf(png_ptr)))
		png_read_end(png_ptr, end_info);

	//Destroy all PNG structs
	png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
}


////////////////////////////////////////////////////////////////////////////////
//
// Implementation of PNG reader class
//

png_reader::~png_reader()
{
}

png_reader::png_reader(istream& i)
:p(new PNGPimpl(i))
{
}

string png_reader::datatype()
{
	return p->datatype();
}

string png_reader::name()
{
	return p->name();
}

ImageRef png_reader::size()
{
	return p->size();
}


bool png_reader::top_row_first()
{
	return true;
}
//Mechanically generate the pixel reading calls.
#define GEN1(X) void png_reader::get_raw_pixel_line(X*d){p->read_pixels(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)
GEN1(bool)
GEN3(unsigned char)
GEN3(unsigned short)

////////////////////////////////////////////////////////////////////////////////
//
// PNG writing functions.
//

class CVD::PNG::WriterPimpl
{
	public:
		WriterPimpl(std::ostream&, ImageRef size, const std::string& type);
		~WriterPimpl();
		template<class P> void write_line(const P*);

	private:

	long row;
	std::ostream& o;
	ImageRef size;
	std::string type;
	std::string error_string;

	png_structp png_ptr;
	png_infop info_ptr, end_info;

};


WriterPimpl::WriterPimpl(ostream& out, ImageRef sz, const string& type_)
:row(0),o(out),size(sz),type(type_)
{
	//Create required structs
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, &error_string, error_fn, warn_fn);
	if(!png_ptr)
		throw Exceptions::OutOfMemory();

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)     
	{        
		png_destroy_write_struct(&png_ptr,NULL);
		throw Exceptions::OutOfMemory();
	}

	//Set up error handling
	if(setjmp(png_jmpbuf(png_ptr)))     
	{         
		png_destroy_write_struct(&png_ptr, &info_ptr);
		throw Exceptions::Image_IO::WriteError(error_string);
	}

	//Set up stream IO
	png_set_write_fn(png_ptr, &o, write_fn, flush_fn);

	int c_type=0;
	int depth=0;

	if(type == "bool")
	{
		c_type = PNG_COLOR_TYPE_GRAY;
		depth=1;
	}
	else if(type == "unsigned char")
	{
		c_type = PNG_COLOR_TYPE_GRAY;
		depth=8;
	}
	else if(type == "unsigned short")
	{
		c_type = PNG_COLOR_TYPE_GRAY;
		depth=16;
	}
	else if(type == "CVD::Rgb<unsigned char>")
	{
		c_type = PNG_COLOR_TYPE_RGB;
		depth=8;
	}
	else if(type == "CVD::Rgb8")
	{
		c_type = PNG_COLOR_TYPE_RGB;
		depth=8;
		//Note the existence of meaningless filler.
		png_set_filler(png_ptr, 0, PNG_FILLER_AFTER);
	}
	else if(type == "CVD::Rgb<unsigned short>")
	{
		c_type = PNG_COLOR_TYPE_RGB;
		depth=16;
	}
	else if(type == "CVD::Rgba<unsigned char>")
	{
		c_type = PNG_COLOR_TYPE_RGB_ALPHA;
		depth = 8;
	}
	else if(type == "CVD::Rgba<unsigned short>")
	{
		c_type = PNG_COLOR_TYPE_RGB_ALPHA;
		depth = 16;
	}
	else
		throw UnsupportedImageSubType("TIFF", type);


	//Set up the image type
	png_set_IHDR(png_ptr, info_ptr, size.x, size.y, depth, c_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

	//Write the header 
	png_write_info(png_ptr, info_ptr);
	
	//Write the transformations
	png_set_swap_if_necessary(png_ptr, depth);

	//Pack from C++ bools to packed PNG bools
	//This has to be done _after_ writing the info struct.
	if(type == "bool")
		png_set_packing(png_ptr);

}

////////////////////////////////////////////////////////////////////////////////
//
// Main interface funtions
//

template<class P> void WriterPimpl::write_line(const P* data)
{
	unsigned char* chardata = const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(data));

	//Do some type checking
	if(type != PNM::type_name<P>::name())
		throw WriteTypeMismatch(type, PNM::type_name<P>::name());


	//Set up error handling
	if(setjmp(png_jmpbuf(png_ptr)))     
		throw Exceptions::Image_IO::WriteError(error_string);

	//Do some sanity checking
	if(row > size.y)
		throw InternalLibraryError("CVD", "Write past end of image.");


	unsigned char** row_ptr =  & chardata;
	png_write_rows(png_ptr, row_ptr, 1);

	row++;
}

WriterPimpl::~WriterPimpl()
{
	png_write_end(png_ptr, info_ptr);
	png_destroy_write_struct(&png_ptr, &info_ptr);
}

png_writer::png_writer(std::ostream&o, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >&)
:p(new WriterPimpl(o, size, type))
{
}

png_writer::~png_writer()
{
}
//Mechanically generate the pixel writing calls.
#undef GEN1
#undef GEN3
#define GEN1(X) void png_writer::write_raw_pixel_line(const X*d){p->write_line(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)
GEN1(bool)
GEN1(Rgb8)
GEN3(unsigned char)
GEN3(unsigned short)
