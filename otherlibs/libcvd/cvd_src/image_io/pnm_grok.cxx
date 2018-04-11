/*******************************************************************************

						pnm_grok.cxx																			  Ed Rosten 2003

This file provides very basic functionality for accessing PNM images. PNM images
are either greyscale (PBMs are treated as PGMs of depth 1) or RGB, and either 1 
or 2 bytes per element. 

Images are dealt with by lines. 1 byte images are delt with as lines of
unsigned chars and 2 byte images are dealt with as lines of unsigned shorts.
Luma images are stored as an array on luma pixel values and RGB images are
dealt with as RGBRGBRGB... packed in to an array. The base class provides the
method elements_per_line() which returns the length of the array needed to
store a single line of the image.

The library here provides no image re-encoding. A 2 byte colour image can only
be read as a 2 byte colour image. The exception is that a PBM is read as a PGM 
of depth 1.

In case of an error, an exception of type err is thrown. The error codes are
arranged in to errors to do with the PNM images and things probably resulting
from a logic error in the calling program.

*******************************************************************************/


#include <iostream>
#include <ctype.h>
#include <limits>

#include <cvd/internal/io/pnm_grok.h>
#include <cvd/internal/load_and_save.h>
#include "cvd_src/config_internal.h"

/* Compile time, arch specific parameters

PNMs are big endian, so for them to be loaded quickly (ie using fwrite), the
bytes have to be swapped on little endian machines. This is OK for loading
since the byte swapping can be done in place, but not OK for saving since the
array of data is passed as const, so it can't be swapped in place.

Short and long PNMs are the same thing. they're 2 byte PNMs (long because
they're longer than  byte PNMs and short since they fit in to the "short" 
data type).
*/


#if defined CVD_INTERNAL_ARCH_LITTLE_ENDIAN
#define SWAP_BYTES
#elif defined CVD_INTERNAL_ARCH_BIG_ENDIAN

#else
#error "No endianness specified!"
#endif

#define PBM 0
#define PGM 1
#define PPM 2

using namespace std;
using namespace CVD::Exceptions::Image_IO;

namespace CVD
{
    namespace PNM
    {

		//All possible error codes. These are loosely divided in to data errors (ie
		//problems with the input image) and programmer errors (runtime errors
		//which are almost definitely caused by some logic error in the program).
		enum errors
		{
			E_NONE = 0,										//Data errors
			E_NOT_PNM,								
			E_HEOF,							
			E_DEOF,	
			E_BAD_SIZE,
			E_BAD_MAXVAL,
			E_PBM_NOT_IMPLEMENTED,
			E_OUT_OF_MEMORY,
			E_UNTERMINATED_HEADER,
			E_PAM_NOT_IMPLEMENTED,
			E_LAST_DATA_ERROR,

			E_PROGRAMMER_ERROR = 255,		//All data errors are sit below this

	
			//Probably programmer errors:
			E_NO_LONG_PBM,
			E_WRITE_PAST_END,
			E_NOT_2_BYTE,
			E_NOT_1_BYTE,
			E_READ_PAST_END,

			E_MAX											//All errors sit below this
		};

		string name_error(errors e)
		{
			const char* data_error_names[]=
			{
				"None.",
				"Not a PNM.",
				"EOF in header.",
				"EOF in data.",
				"Bad size specified.",
				"Invalid colour MAXVAL.",
				"Binary PBMs are not implemented.",
				"Out of memory.",
				"Unterminated header.",
				"Portable Arbirtary Maps are not implemented.",
				"Last data error. This should *never* occur."
			};
				
			const char* prog_error_names[]=
			{
				"Long PBMs are silly.",
				"Trying to write past end of file.",
				"This is not a 2 byte PNM.",
				"This is not a 1 byte PNM.",
				"Trying to read past the end of the file."
			};

			const char* invalid = "Not a valid error.";

			string ret;

			if(e < 0 || e >= E_MAX)
				ret=string("internal error: ") + invalid;
			else if(e >= E_LAST_DATA_ERROR && e <= E_PROGRAMMER_ERROR)
				ret=string("internal error: ") + invalid;
			else if(e < E_LAST_DATA_ERROR)
				ret=data_error_names[e];
			else
				ret=string("internal error: ") + prog_error_names[e-E_PROGRAMMER_ERROR - 1];

			return "Error in PNM image: " + ret;
		}


		class pnm_in
		{
			public:
			  pnm_in(std::istream&);
			  ImageRef size();
			  string datatype();

				template<class C>
				void get_raw_pixel_lines(C * c, unsigned long nlines);
							
			private:
			  bool m_is_2_byte;
			  int  m_channels;
			  string elemtype;
			  std::istream&		i;
			  bool		is_text;
			  int   type, maxval;
			  int   lines_so_far;
			  void		read_header();
			  bool  can_proc_lines(unsigned long);
			  long		xs, ys;
		};

		bool clean(istream& i)
		{
			////////////////////////////////////////////////////////////////////////////
			//
			// Strip leading whitespae and PNM comments from an istream
			// comments start with a '#' and and with a newline

			unsigned char  c;

			for(;;)
			{
				i >> c;

				if(i.eof()) //I'm pedantic like that.
					return 0;
				else if(c == '#') //Eat a whole line of comment
				{
					do
					{
						i >> c;
						if(i.eof())		//I'm still a pedant
							return 0;
					}while(c != '\n');
				}
				else if(!isspace(c))
				{
					i.putback(c);
					return 1;
				}
			}
		}

		////////////////////////////////////////////////////////////////////////////////
		//
		// base PNM class definitions
		//
		////////////////////////////////////////////////////////////////////////////////

		bool pnm_in::can_proc_lines(unsigned long nl)
		{
			//Slightly oddly named. Update the number of processed lines and return 
			//an error condition if the lines can not be processed.

			lines_so_far += nl;
			if(lines_so_far > ys)
				return false;
			else
				return true;
		}

		////////////////////////////////////////////////////////////////////////////////
		//
		// Input PNM class definitions
		//
		////////////////////////////////////////////////////////////////////////////////

		pnm_in::pnm_in(std::istream& in)
		:i(in)
		{
			lines_so_far=0;
			read_header();
		}

		void pnm_in::read_header()
		{
			//Fix the stream before returning.
#define RETURN(X) do{													\
				i >> skipws;											\
				if(X == E_NONE)											\
					return;												\
				else													\
					throw Exceptions::Image_IO::MalformedImage(name_error(X)); \
			}while(0)																	

#define CLEAN	do{														\
				if(!clean(i))											\
					throw Exceptions::Image_IO::MalformedImage(name_error(E_HEOF)); \
			}while(0)

#define GET(X)	do{														\
				CLEAN;													\
				i>>X;													\
				if(i.eof())												\
					throw Exceptions::Image_IO::MalformedImage(name_error(E_HEOF)); \
			}while(0)

			char c1, c2;
		
			// i >> (int) will eat whitespace before and after the integer. This breaks
			// reading maxval since it can (and does) eat in to the image.
			i >> noskipws;


			//Read and check magic number

			i >> c1 >> c2;
			if(i.eof())
				RETURN(E_HEOF);

			if(c1 != 'P' || c2 < '1' || c2 > '7')
				RETURN(E_NOT_PNM);

			if(c2 == '7')
				RETURN(E_PAM_NOT_IMPLEMENTED);

			if(strchr("123", c2))
				is_text = true;
			else
				is_text = false;

			if(c2 == '1' || c2 == '4')
				type = PBM;
			else if(c2 == '2' || c2 == '5')
				type = PGM;
			else
				type = PPM;
			

			if(type == PGM)
				m_channels = 1;
			else 
				m_channels = 3;

			//Read and check image dimensions
			GET(xs);
			GET(ys);

			if(xs <= 0 || ys <= 0)
				RETURN(E_BAD_SIZE);
		
			//Read if necessasy and set MAXVAL
			if(type != PBM)
			{	
				GET(maxval);

				if(maxval <= 0 || maxval > 65535)
					RETURN(E_BAD_MAXVAL);
								
				if(maxval <= 255)
					m_is_2_byte = false;
				else
					m_is_2_byte = true;
			}
			else
			{
				maxval = 255;
				m_is_2_byte = false;
			}
			
			if(type == PBM)
				elemtype = "bool";
			else if(type == PGM)
			{
				if(m_is_2_byte)
					elemtype = PNM::type_name<unsigned short>::name();
				else
					elemtype = PNM::type_name<byte>::name();
			}
			if(type == PPM)
			{
				if(m_is_2_byte)
					elemtype = PNM::type_name<Rgb<unsigned short> >::name();
				else
					elemtype = PNM::type_name<Rgb<byte> >::name();
			}
			
				
		
			//Remove everything to the beginning of the image
			if(is_text)
				CLEAN;
			else
			{
				//Binary PNMs have a single byte of whitespace.
		
				unsigned char tmp;
				i >> tmp;
				if(!isspace(tmp))
					RETURN(E_UNTERMINATED_HEADER);
			}

			//hee hee hee. This makes the SGI compiler segfault :-)
			//	RETURN(E_NONE);
			i >> skipws;

			//HACK
			//cerr << "is_rgb=" << m_is_rgb <<". xsize=" << xs << ". ysize=" << ys << ". is_text=" << is_text << ". type=" << type  << ". maxval=" << maxval << endl; 

		
			return;
#undef RETURN
#undef CLEAN
#undef GET
		}

		
		//Text reading functions
		void get_num(istream& i, byte* c)
		{
			int j=0;
			i >> j;  //Ignore EOF: be leniant
			*c = j & 0xff;
		}

		void get_num(istream& i, bool* c)
		{
			i >> ws;
			int j = cin.get();
			//PBMs are inverted
			*c = (j != '0');
		}

		void get_num(istream& i, unsigned short* c)
		{
			int j=0;
			i >> j;  //Ignore EOF: be leniant
			*c = j & 0xffff;
		}

		template<class C> void get_num(istream& i, Rgb<C>* c)
		{
				get_num(i, & (c->red));
				get_num(i, & (c->green));
				get_num(i, & (c->blue));
		}
		
		//Data reading functions
		void really_swap(unsigned char* s, size_t n_elem)
		{
			#ifdef SWAP_BYTES
				for(size_t j=0; j < n_elem; j++)
					swap(s[2*j], s[2*j+1]);
			#endif
		}

		void maybe_swap(unsigned short* s, size_t n)
		{
				really_swap((unsigned char*)s, n);
		}
		void maybe_swap(Rgb<unsigned short>*s , size_t n)
		{
				really_swap((unsigned char*)s, n*3);
		}

		void maybe_swap(bool*, size_t)
		{}
		void maybe_swap(byte*, size_t)
		{}
		void maybe_swap(Rgb<byte>*, size_t)
		{}

		template<class C>
		void pnm_in::get_raw_pixel_lines(C * c, unsigned long nlines)
		{
			if(datatype() != PNM::type_name<C>::name())
				throw ReadTypeMismatch(datatype(), PNM::type_name<C>::name());

			//Reading into uchars is sufficiently different from reading in to
			//ushorts, so it has ben done as 2 functions as opposed to one 
			//tmeplated one.

			unsigned long k, npix;
			if(!can_proc_lines(nlines))
				throw Exceptions::Image_IO::MalformedImage(name_error(E_READ_PAST_END));
		
			//Load a bunch of pixels in to memory without doing internal buffering
			npix = xs * nlines;

			if(is_text)			//Load text PNM data
			{
				for(k=0; k < npix; k++)
					get_num(i, c++);
			}
			else if(type != PBM)
			{
				if(type == PBM)
					throw Exceptions::Image_IO::MalformedImage(name_error(E_PBM_NOT_IMPLEMENTED));

				i.read((char*)c, m_channels*npix * (m_is_2_byte?2:1));
				maybe_swap(c, npix);
			}

			//Don't report errors. The spec says to be really leniant with
			//respect to errors in text only pnms
			if(!is_text  && i.fail())
				throw Exceptions::Image_IO::MalformedImage(name_error(E_DEOF));

		}

		ImageRef pnm_in::size()
		{
			return ImageRef(xs, ys);
		}

		string pnm_in::datatype()
		{
			return elemtype;
		}



		////////////////////////////////////////////////////////////////////////////////
		//
		// Implementation of PNM reader class
		//

		Reader::~Reader()
		{
		}

		Reader::Reader(istream& i)
		:p(new pnm_in(i))
		{
		}

		string Reader::datatype()
		{
			return p->datatype();
		}

		bool Reader::top_row_first()
		{
			return true;
		}
		string Reader::name()
		{
			return "PNM";
		}

		ImageRef Reader::size()
		{
			return p->size();
		}


		//Mechanically generate the pixel reading calls.
		#define GEN1(X) void Reader::get_raw_pixel_line(X*d){p->get_raw_pixel_lines(d, 1);}
		#define GEN2(X) GEN1(X) GEN1(Rgb<X>)
		GEN1(bool)
		GEN2(unsigned char)
		GEN2(unsigned short)


		#undef GEN1
		#undef GEN2
		////////////////////////////////////////////////////////////////////////////////
		//
		// Output PNM class definitions
		//
		////////////////////////////////////////////////////////////////////////////////

		class pnm_writer
		{
			public:
				pnm_writer(std::ostream&, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >& p);
				~pnm_writer();

				//void write_raw_pixel_line(const bool*);
				template<class C> void write_raw_pixel_line(const C*);

			private:

				void write_shorts(const unsigned char* data, size_t n_shorts);
				void write_binary(const unsigned char* data, size_t n);
				void write_binary(const unsigned short* data, size_t n);
				void write_binary(const Rgb<unsigned char>* data, size_t n);
				void write_binary(const Rgb<unsigned short>* data, size_t n);


				template<class P> void sanity_check(const P*);

			bool text;
			long row;
			std::ostream& o;
			ImageRef size;
			std::string type;
			vector<unsigned char>  rowbuf;
		};


		void writePNMHeader(ostream& out, int channels, ImageRef size, int maxval, bool text, const std::string& comments)
		{
			char m[3] = {'P',' ', '\n'};
			if (channels == 1)
				m[1] = '2';
			else 
				m[1] = '3';
			if (!text)
				m[1] += 3;
			out.write(m, 3);
			bool freshLine = true;
			for (size_t i=0; i<comments.length(); i++) {
				if (freshLine)
					out << "# ";
				char c = comments[i];
				if (c == '\n') {
					freshLine = true;
					out << endl;
				} else {
					out << c;
					freshLine = false;
				}
			}
			if (!freshLine)
				out << endl;
			out << size.x << " " << size.y << endl << maxval << endl;
		}

		pnm_writer::pnm_writer(std::ostream& out, ImageRef size_, const std::string& type_, const std::map<std::string, Parameter<> >& p)
		:text(0),row(0),o(out),size(size_),type(type_)
		{
			if(p.count("pnm.raw"))
			{
				try{
						text=!(p.find("pnm.raw")->second.get<bool>());
				}
				catch(std::bad_cast c){
						cerr << "Warning pnm.raw is not a bool.\n";
				}
			}
			
			if(type == "unsigned char")
					writePNMHeader(out, 1, size, 255, text, "");
			else if(type == "unsigned short")
					writePNMHeader(out, 1, size, 65535, text, "");
			else if(type == "CVD::Rgb<unsigned char>")
					writePNMHeader(out, 3, size, 255, text, "");
			else if(type == "CVD::Rgb<unsigned short>")
					writePNMHeader(out, 3, size, 65535, text, "");
			else
				throw UnsupportedImageSubType("PNM", type);
		}

		pnm_writer::~pnm_writer()
		{}

		void pnm_writer::write_shorts(const unsigned char* data, size_t n_shorts)
		{
			#ifdef SWAP_BYTES
					rowbuf.resize(n_shorts*sizeof(short));
					copy(data, data + sizeof(short)*n_shorts, rowbuf.begin());

					for (size_t i=0; i<rowbuf.size(); i+=2)
						swap(rowbuf[i], rowbuf[i+1]);

					o.write((char*)&rowbuf[0], rowbuf.size());
					//for (size_t i=0; i<n_shorts; i++)
							//o << data[2*i+1] << data[2*i];
			#else
					o.write((const char*)data, n_shorts*sizeof(unsigned short));
			#endif
		}

		void pnm_writer::write_binary(const unsigned char* data, size_t n)
		{
			o.write(reinterpret_cast<const char*>(data), n);
		}
		void pnm_writer::write_binary(const unsigned short* data, size_t n)
		{
			write_shorts(reinterpret_cast<const unsigned char*>(data), n);
		}
		void pnm_writer::write_binary(const Rgb<unsigned char>* data, size_t n)
		{
			o.write(reinterpret_cast<const char*>(data), n*3);
		}
		void pnm_writer::write_binary(const Rgb<unsigned short>* data, size_t n)
		{
			write_shorts(reinterpret_cast<const unsigned char*>(data), n*3);
		}

		template<class C> void write_text(const C* data, size_t n, ostream& o)
		{
			for(size_t i=0; i < n; i++)
				o << (int) data[i] << endl;
		}

		template<class C> void write_text(const Rgb<C>* data, size_t n, ostream& o)
		{
			for(size_t i=0; i < n; i++)
				o << (int) data[i].red << " " << (int)data[i].green << " " << (int)data[i].blue << endl;
		}

		template<class C> 
		void pnm_writer::write_raw_pixel_line(const C* data)
		{
			if(type != PNM::type_name<C>::name())
				throw WriteTypeMismatch(type, PNM::type_name<C>::name());
		
			//Do some sanity checking
			if(row >= size.y)
				throw InternalLibraryError("CVD", "Write past end of image.");
		
			row++;
				
			if(text)
				write_text(data, size.x, o);
			else
				write_binary(data, size.x);
		}

		////////////////////////////////////////////////////////////////////////////////
		//
		// Public interface
		//

		Writer::Writer(std::ostream& out, ImageRef size_, const std::string& type_, const std::map<std::string, Parameter<> >&  p)
		:p(new pnm_writer(out, size_, type_, p))
		{}

		Writer::~Writer()
		{}

		//Mechanically generate the pixel writing calls.
		#define GEN1(X) void Writer::write_raw_pixel_line(const X*d){p->write_raw_pixel_line(d);}
		#define GEN2(X) GEN1(X) GEN1(Rgb<X>)
		//GEN1(bool)
		GEN2(unsigned char)
		GEN2(unsigned short)

        #undef GEN1
        #undef GEN2
	}
}
