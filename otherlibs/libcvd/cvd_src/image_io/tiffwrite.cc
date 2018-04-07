#include "cvd/internal/io/tiff.h"
#include "cvd/image_io.h"
#include <tiffio.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <map>

using namespace CVD;
using namespace CVD::TIFF;
using namespace CVD::Exceptions::Image_IO;
using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of TIFF reading
//

class CVD::TIFF::TIFFWritePimpl
{
	public:
		TIFFWritePimpl(ostream&, ImageRef size, const string& t);
		~TIFFWritePimpl();

		template<class C> void write_raw_pixel_line(const C* data);
		void write_raw_pixel_line(const bool* data);

	private:
		ostream& o;
		ImageRef my_size;
		string   type;
		unsigned long row;
		::TIFF* tif;
		streamoff length;
		long    strip_size;
		vector<uint8>  bool_rowbuf;


		static tsize_t write(thandle_t vis, tdata_t data, tsize_t count);
		static tsize_t read(thandle_t vis, tdata_t data, tsize_t count);
		static toff_t seek(thandle_t vis, toff_t off, int dir);
		static toff_t size(thandle_t vis);
		static int close(thandle_t vis);
		static int map(thandle_t, tdata_t*, toff_t*);
		static void unmap(thandle_t, tdata_t, toff_t);
};


tsize_t TIFFWritePimpl::write(thandle_t vis, tdata_t data, tsize_t count)
{
	TIFFWritePimpl* o = (TIFFWritePimpl*)vis;
	streamoff p = o->o.tellp();
	o->o.write((const char*)data, count);
	return o->o.tellp() - p;
}

tsize_t TIFFWritePimpl::read(thandle_t, tdata_t, tsize_t)
{
	return 0;
}

toff_t TIFFWritePimpl::seek(thandle_t vis, toff_t off, int dir)
{
	TIFFWritePimpl* p = (TIFFWritePimpl*)vis;
	ostream& o(p->o);
	
	if(dir == SEEK_SET)
		o.seekp(off, ios_base::beg);
	else if(dir == SEEK_CUR)
		o.seekp(off, ios_base::cur);
	else if(dir == SEEK_END)
		o.seekp(off, ios_base::end);
	

	//From comments in libtiff:

	// Attempt to workaround problems with seeking past the end of the
	// stream.  ofstream doesn't have a problem with this but
	// ostrstream/ostringstream does. In that situation, add intermediate
	// '\0' characters.
	if(o.fail()) 
	{
		ios::iostate	old_state = o.rdstate();
		ios::iostate	safe_state = old_state & ~ios_base::failbit;
		streamoff		crnt=0;

		// clear the fail bit so else tellp() works
		o.clear(safe_state);
		if(dir == SEEK_SET)
			crnt = 0;
		else if (dir == SEEK_CUR)
			crnt = o.tellp();
		else if(dir == SEEK_END)
		{
			o.seekp(0, ios_base::end);
			crnt = o.tellp();
		}

		// restore original stream state
		o.clear(old_state);	

		// only do something if desired seek position is valid
		if( crnt + off > 0 ) {

			// clear the fail bit 
			o.clear(safe_state);

			// extend the stream by writing zeros
			o.seekp(0, ios::end);
			streamoff extra = crnt + off - o.tellp();
			for(streamoff i = 0; i < extra; i++ )
				o.put(0);

			// retry the seek, just to make sure.
			o.seekp(crnt + off, ios_base::beg);
		}
	}


	return o.tellp();
}

toff_t TIFFWritePimpl::size(thandle_t vis)
{
	TIFFWritePimpl* ii = (TIFFWritePimpl*)vis;
	return ii->length;
}

int TIFFWritePimpl::close(thandle_t)
{
	return 0;
}

int TIFFWritePimpl::map(thandle_t, tdata_t*, toff_t*)
{
	return 0;
}

void TIFFWritePimpl::unmap(thandle_t, tdata_t, toff_t)
{
}


static const int error_size=512;
static char error_msg[error_size]="";

static void tiff_error_handler(const char*, const char* fmt, va_list ap)
{
	int n = vsnprintf(error_msg, error_size, fmt, ap);
	if(n == error_size)
		error_msg[n-1] = 0;
}

template<class T> void TIFFWritePimpl::write_raw_pixel_line(const T* data)
{
	//Do some type checking
	if(type != PNM::type_name<T>::name())
		throw WriteTypeMismatch(type, PNM::type_name<T>::name());
	
	//Do some sanity checking
	if(row >= (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Write past end of image.");


	if(TIFFWriteEncodedStrip(tif, row, const_cast<T*>(data), strip_size) == -1)
		throw WriteError(error_msg);
		
	row++;
}

void TIFFWritePimpl::write_raw_pixel_line(const bool* data)
{
	//Do some type checking
	if(type != PNM::type_name<bool>::name())
		throw WriteTypeMismatch(type, PNM::type_name<bool>::name());
	
	//Do some sanity checking
	if(row >= (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Write past end of image.");

	
	fill(bool_rowbuf.begin(), bool_rowbuf.end(), 0);

	//Pack bools
	for(int i=0; i < my_size.x  ;i++)
		bool_rowbuf[i/8] |=  (data[i] & 1) << (7-i%8);

	if(TIFFWriteScanline(tif, &bool_rowbuf[0], row) == -1)
		throw WriteError(error_msg);

	
		
	row++;
}

TIFFWritePimpl::~TIFFWritePimpl()
{
	if(tif)
		TIFFClose(tif);
}


TIFFWritePimpl::TIFFWritePimpl(ostream& os, ImageRef s, const string& t)
:o(os),my_size(s),type(t),row(0),tif(0)
{
	TIFFSetErrorHandler(tiff_error_handler);
	
	//Find out the file size, and the suitability of the stream
	o.seekp(0, ios_base::end);
	length = o.tellp();
	if(length == -1)
		throw UnseekableIstream("TIFF");
	o.seekp(0, ios_base::beg);


	tif = TIFFClientOpen("std::ostream", "w", this, 
						 read, write, seek, close, size, map, unmap);

	if(tif == NULL)
		throw WriteError(error_msg);

	//Set the correct tags.
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, my_size.x);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, my_size.y);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 1);

	uint16 alpha[] = {EXTRASAMPLE_UNASSALPHA};
	if(t == "bool")
	{
		TIFFSetField(tif, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_CCITTFAX4);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 1);
		TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, my_size.y);
		bool_rowbuf.resize((my_size.x + 7)/8);
	}
	else if(t == "unsigned char")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	}
	else if(t == "unsigned short")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
	}
	else if(t == "float")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
	}
	else if(t == "double")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 64);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
	}
	else if(t == "CVD::Rgb<unsigned char>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	}
	else if(t == "CVD::Rgb<unsigned short>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
	}
	else if(t == "CVD::Rgb<float>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
	}
	else if(t == "CVD::Rgb<double>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 64);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
	}
	else if(t == "CVD::Rgba<unsigned char>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 4);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
		TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, 1, alpha);
	}
	else if(t == "CVD::Rgba<unsigned short>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 4);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
		TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, 1, alpha);
	}
	else if(t == "CVD::Rgba<float>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 4);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
		TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, 1, alpha);
	}
	else if(t == "CVD::Rgba<double>")
	{
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_DEFLATE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 4);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 64);
		TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
		TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, 1,  alpha);
	}
	else
		throw UnsupportedImageSubType("TIFF", t);

	strip_size = TIFFStripSize(tif);
}




////////////////////////////////////////////////////////////////////////////////
//
// Implementation of public parts of TIFF reading
//

tiff_writer::tiff_writer(ostream& o, ImageRef size, const string& type, const std::map<std::string, Parameter<> >&)
:t(new TIFFWritePimpl(o, size, type))
{}

tiff_writer::~tiff_writer()
{
}

//Mechanically generate the pixel reading calls.
#define GEN1(X) void tiff_writer::write_raw_pixel_line(const X*d){t->write_raw_pixel_line(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)

GEN1(bool)
GEN3(unsigned char)
GEN3(unsigned short)
GEN3(float)
GEN3(double)






















