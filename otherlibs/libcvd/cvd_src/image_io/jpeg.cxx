
#include <stdio.h>
extern "C"{
#include <jpeglib.h>
}
#include "cvd/internal/io/jpeg.h"

#include "cvd/image_io.h"
#include "cvd_src/config_internal.h"
using namespace std;
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <setjmp.h>


namespace CVD
{
namespace JPEG
{

class ReadPimpl
{
	public:
		ReadPimpl(std::istream&);
		int channels(){return m_channels;}
		long  x_size() const {return xs;}
		long  y_size() const {return ys;}
		long  elements_per_line() const {return xs * m_channels;}
		void get_raw_pixel_lines(unsigned char*, unsigned long nlines);
		~ReadPimpl();
		string datatype()
		{
			return type;
		}
		
		template<class T> void get_raw_pixel_line(T* d)
		{
			if(datatype() != PNM::type_name<T>::name())
				throw CVD::Exceptions::Image_IO::ReadTypeMismatch(datatype(), PNM::type_name<T>::name());

			get_raw_pixel_lines((unsigned char*)d, 1);
			//FIXME: check rows.
		}

	private:
		long	xs, ys;
		int	m_channels;
		struct jpeg_decompress_struct cinfo;
		struct jpeg_error_mgr jerr;
		std::istream&	i;
		string type;
};

ImageRef reader::size()
{
	return ImageRef(t->x_size(), t->y_size());
}

void reader::get_raw_pixel_line(unsigned char* d)
{
	t->get_raw_pixel_line(d);
}

void reader::get_raw_pixel_line(Rgb<byte>* d)
{
	t->get_raw_pixel_line(d);
}
bool reader::top_row_first()
{
	return true;
}

string reader::datatype()
{
	return t->datatype();
}
string reader::name()
{
	return "JPEG";
}

reader::~reader()
{}

reader::reader(std::istream& i)
:t(new ReadPimpl(i))
{}

struct jpeg_istream_src: public jpeg_source_mgr
{

	istream* i;
	bool eof;
	static const int bufsize=CVD_INTERNAL_JPEG_BUFFER_SIZE;
	JOCTET buf[bufsize+2];

	//Constructor
	static void create(j_decompress_ptr p, istream* is)
	{
		//Allocate memory
		p->src = (jpeg_istream_src*) (*p->mem->alloc_small)((jpeg_common_struct*)p, JPOOL_PERMANENT, sizeof(jpeg_istream_src));

		jpeg_istream_src* me = (jpeg_istream_src*)p->src;

		//Set up virtual member functions
		me->init_source = s_init_src;
		me->fill_input_buffer = s_fill_input_buffer;
		me->skip_input_data = s_skip_input_data;
		me->resync_to_restart = jpeg_resync_to_restart;
		me->term_source = s_term_source;

		//Set up data members
		me->i = is;
		me->bytes_in_buffer = 0;
		me->next_input_byte = me->buf;
		me->eof = 0;
	}
	

	static void s_init_src(j_decompress_ptr p)
	{
		jpeg_istream_src* me = (jpeg_istream_src*)p->src;

		me->bytes_in_buffer = 0;	
	}

	static boolean s_fill_input_buffer(j_decompress_ptr p)
	{

		int n=0;
		jpeg_istream_src* me = (jpeg_istream_src*)p->src;
		me->next_input_byte = me->buf;

		if(me->eof)
		{
			//Output JPEG end-of-image flags
			me->buf[0] = 0xff;
			me->buf[1] = JPEG_EOI;
			me->bytes_in_buffer = 2;
			return TRUE;
		}

		int c;
		for(n=0; n < bufsize;  n++)
		{
			//Get a byte...
			c = me->i->get();
			

			//Check for EOF...
			if(c  == EOF)
			{
				me->eof = 1;
				break;
			}
			
			//Store the byte...
			me->buf[n] = c;
		}

		me->bytes_in_buffer = n;

		if(me->i->eof())
			me->eof = 1;

		if(me->bytes_in_buffer == 0)
			s_fill_input_buffer(p);

		return TRUE;
	}

	static void s_skip_input_data(j_decompress_ptr p, long num)
	{
		jpeg_istream_src* me = (jpeg_istream_src*)p->src;


		if(num > (long)(me->bytes_in_buffer))
		{
			me->i->ignore(num - me->bytes_in_buffer);
			me->bytes_in_buffer = 0;

		}

		else
		{
			me->next_input_byte += num;
			me->bytes_in_buffer -= num;
		}
	}

	static void s_term_source(j_decompress_ptr)
	{
	}
};


void jumpy_error_exit (j_common_ptr cinfo)
{
	//Exceptions don't work from C code, so use setjmp/longjmp to jump back
	//in to C++ code.
	longjmp(*(jmp_buf*)(cinfo->client_data), 1);
}

struct jpeg_error_mgr * jumpy_error_manager (struct jpeg_error_mgr * err)
{
	//Set up most of the useful defaults
	jpeg_std_error(err);
	//Except we don't want to exit on an error.
	err->error_exit = jumpy_error_exit;
	return err;
}

ReadPimpl::ReadPimpl(istream& in)
:i(in)
{
	cinfo.err = jumpy_error_manager(&jerr);

	jmp_buf env;
	cinfo.client_data = &env;

	//Catch "exceptions" and throw proper exceptions
	if(setjmp(env))
	{
		//longjmp called
		char buffer[JMSG_LENGTH_MAX];
		(cinfo.err->format_message)((jpeg_common_struct*)&cinfo, buffer);
		throw CVD::Exceptions::Image_IO::MalformedImage(string("Error in JPEG image: ") + buffer);
	}
	
	jpeg_create_decompress(&cinfo);
	jpeg_istream_src::create(&cinfo, &i);

	jpeg_read_header(&cinfo, TRUE);
	jpeg_start_decompress(&cinfo);

	xs = cinfo.output_width;
	ys = cinfo.output_height;

	m_channels = cinfo.out_color_components;

	if(m_channels == 1)
		type = "unsigned char";
	else
		type = "CVD::Rgb<unsigned char>";
}

void ReadPimpl::get_raw_pixel_lines(unsigned char*data, unsigned long nlines)
{
	jmp_buf env;
	cinfo.client_data = &env;
	
	//Catch "exceptions" and throw proper exceptions
	if(setjmp(env))
	{
		//longjmp called
		char buffer[JMSG_LENGTH_MAX];
		(cinfo.err->format_message)((jpeg_common_struct*)&cinfo, buffer);
		throw CVD::Exceptions::Image_IO::MalformedImage(string("Error in JPEG image: ") + buffer);
	}

	
	unsigned char** datap = &data;
	for(unsigned int i=0; i < nlines; i++)	
	{
		jpeg_read_scanlines(&cinfo, datap, 1);
		data += elements_per_line();
	}
}

ReadPimpl::~ReadPimpl()
{
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Compression 
//

struct jpeg_ostream_dest: public jpeg_destination_mgr
{
	ostream* o;
	static const int bufsize=262144;
	JOCTET buf[bufsize];

	static void create(j_compress_ptr p, ostream* os)
	{
		//Allocate memory
		p->dest = (jpeg_ostream_dest*) (*p->mem->alloc_small)((jpeg_common_struct*)p, JPOOL_PERMANENT, sizeof(jpeg_ostream_dest));

		jpeg_ostream_dest* me = (jpeg_ostream_dest*) p->dest;

		//Set up virtual member functions
		me->init_destination = s_init_destination;
		me->empty_output_buffer = s_empty_output_buffer;
		me->term_destination = s_term_destination;

		//Set up data members
		me->o = os;
	}

	static void s_init_destination(j_compress_ptr cinfo)
	{
		jpeg_ostream_dest* me = (jpeg_ostream_dest*) cinfo->dest;

		me->next_output_byte = &me->buf[0];
		me->free_in_buffer = bufsize;
	}

	static boolean s_empty_output_buffer(j_compress_ptr cinfo)
	{
		jpeg_ostream_dest* me = (jpeg_ostream_dest*) cinfo->dest;

		//Docs say we should do this: 
		me->o->write((const char*)me->buf, bufsize);

		s_init_destination(cinfo);
		return TRUE;
	}

	static void s_term_destination(j_compress_ptr cinfo)
	{
		jpeg_ostream_dest* me = (jpeg_ostream_dest*) cinfo->dest;
		me->o->write((const char*)me->buf, bufsize-me->free_in_buffer);
	}
};


////////////////////////////////////////////////////////////////////////////////
//
// JPEG writing.
//

class WritePimpl
{
	public:
  WritePimpl(std::ostream&, int  xsize, int ysize, const string& type, const std::map<std::string, Parameter<> >& p, const std::string& comm="");
		int channels(){return m_channels;}
		long  x_size() const {return xs;}
		long  y_size() const {return ys;}
		long  elements_per_line() const {return xs * m_channels;}
		void 	write_raw_pixel_lines(const unsigned char*, unsigned long);
		template<class C> 	void write_raw_pixel_line(const C*);
		~WritePimpl();
		
	private:
		long	xs, ys, row;
		int	m_channels;
		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr jerr;
		std::ostream& 	o;
		string type;
};



WritePimpl::WritePimpl(std::ostream& out, int xsize, int ysize, const string& t, const std::map<std::string, Parameter<> >& p, const string& comm)
:o(out)
{
	xs = xsize;
	ys = ysize;
	type = t;
	row=0;
	
	if(type == "unsigned char")
		m_channels = 1;
	else if(type == "CVD::Rgb<unsigned char>")
		m_channels = 3;
	else
		throw Exceptions::Image_IO::UnsupportedImageSubType("JPEG", type);

	//Set up setjmp/lonjmp error handling
	cinfo.err = jumpy_error_manager(&jerr);
	jmp_buf env;
	cinfo.client_data = &env;

	//Catch "exceptions" and throw proper exceptions
	if(setjmp(env))
	{
		//longjmp called
		char buffer[JMSG_LENGTH_MAX];
		(cinfo.err->format_message)((jpeg_common_struct*)&cinfo, buffer);
		throw CVD::Exceptions::Image_IO::WriteError(string("JPEG: ") + buffer);
	}

	//Create the compression object
	jpeg_create_compress(&cinfo);
	
	//Get the jpeg_ostream_dest class to handle output.
	jpeg_ostream_dest::create(&cinfo, &o);

	//Setup parameters
	cinfo.image_width = xs;
	cinfo.image_height = ys;
	cinfo.input_components = m_channels;
	cinfo.in_color_space = (m_channels==3) ? JCS_RGB : JCS_GRAYSCALE;

	int quality = 95;
	if(p.count("jpeg.quality"))
	{
		try{
			quality = p.find("jpeg.quality")->second.get<int>();
			quality = max(0,min(100,quality));
		}
		catch(std::bad_cast c)
		{
			cerr << "Warning jpeg.quality is not an int.\n";
		}
	}

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, TRUE);

	jpeg_start_compress(&cinfo, TRUE);

	unsigned int commlen = comm.length();
	
	//NB: not 65536 Marker looks like:
	// marker_byte length_high length_low length*bytes
	// length includes the block header (3 bytes)	

	if(commlen > 65533)
		commlen = 65533;

	//Written without zero termination, since the length is also written
	jpeg_write_marker(&cinfo, JPEG_COM, (JOCTET*)comm.c_str(), comm.length());
}

void WritePimpl::write_raw_pixel_lines(const unsigned char* data, unsigned long nlines)
{
	jmp_buf env;
	cinfo.client_data = &env;
	long elem = elements_per_line();

	if(nlines + row > (unsigned long) ys)
		throw CVD::Exceptions::Image_IO::InternalLibraryError("CVD", "Write past end of image.");
	
	//Catch "exceptions" and throw proper exceptions
	if(setjmp(env))
	{
		//longjmp called
		char buffer[JMSG_LENGTH_MAX];
		(cinfo.err->format_message)((jpeg_common_struct*)&cinfo, buffer);
		throw CVD::Exceptions::Image_IO::MalformedImage(string("JPEG: ") + buffer);
	}

	const unsigned char** datap = &data;
	for(unsigned int i=0; i < nlines; i++)	
	{
		jpeg_write_scanlines(&cinfo, (JSAMPLE**)datap, 1);
		data += elem;
		row++;
	}
}
template<class C> 	void WritePimpl::write_raw_pixel_line(const C*d)
{
	if(type != PNM::type_name<C>::name())
		throw CVD::Exceptions::Image_IO::WriteTypeMismatch(type, PNM::type_name<C>::name());

	write_raw_pixel_lines((const unsigned char*)d, 1); 
}

WritePimpl::~WritePimpl()
{
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
}


////////////////////////////////////////////////////////////////////////////////
//
// Public interfaces to image writing.
//

writer::writer(ostream& o, ImageRef size, const string& s, const std::map<std::string, Parameter<> >& p)
:t(new WritePimpl(o, size.x, size.y, s, p))
{}

writer::~writer()
{}

void writer::write_raw_pixel_line(const byte* data)
{
	t->write_raw_pixel_line(data);
}

void writer::write_raw_pixel_line(const Rgb<byte>* data)
{
	t->write_raw_pixel_line(data);
}

}
}
