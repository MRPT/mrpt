#include <cvd/internal/io/save_postscript.h>

#include <cvd/internal/load_and_save.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
using namespace std;

namespace CVD
{

void output_eps_header(ostream& o, int xs, int ys)
{
	//Standard EPS header. This is test more by concensus from various postscript handlers
	//than with any specific reference to documentation, but it seems to work.
	o << 
	"%!PS-Adobe-3.0 EPSF-3.0"																"\n"
	"%%BoundingBox: 0 0 " << xs << " " << ys << 											"\n"
	"%%Creator: libCVD"																		"\n"
	"%%Pages: 0"																			"\n"	
	"%%EndComments"																			"\n"	
	"%%EndProlog"																			"\n"
	//This saves the dictionary state and enables a temporary dictionary.
	//this should prevent the eps file from polluting the vm or dictionary.
	//it's kind of the job of whatever includes EPS to do this, but this makes
	//the job easier and improves reliability. Note that in Level 2 ps, the 
	//dictionary can expand if more than 20 commands are added later.
	//If custom PS is added, it must leave the stack empty at the end.
	"save"																					"\n"
	"20 dict begin"																			"\n";
}

void output_eps_header(ostream& o, const ImageRef& s)
{
	output_eps_header(o, s.x, s.y);
}

void output_eps_footer(ostream& o)
{
	//End the temporary dictionary and restore the VM and graphics state.
	o << "end restore\n"
	//I'm not sure exactly what the end of an EPS image should be, but this 
	//seems to work.
	  << "%%EOF\n";
}


namespace PS
{
	
	class WritePimpl
	{
		public:
			WritePimpl(std::ostream&, ImageRef size, const string& type, const string& extra_header="");

			void 	write_raw_pixel_lines(const unsigned char*, unsigned long);
			~WritePimpl();
			int channels(){return m_channels;}
			long  x_size() const {return xs;}
			long  y_size() const {return ys;}

			std::ostream& 	o;

		private:
			long	xs, ys;
			int	m_channels;
			std::string bytes_to_base85(int n);
			void output_header();
			int lines;
			unsigned char buf[4];
			int  num_in_buf;
			string currentline;
	};














  string WritePimpl::bytes_to_base85(int n)
{
	//This function converts 4 raw bytes to 5 base-85 bytes. If there
	//are less than 4 bytes, then the data is zero padded.

	//Conversion to base85 is done the obvious way, but 33 is added
	//to each number to make it a printable byte.

	//There is a shortcut which means that !!!!! is abbreviated as z

	unsigned long int num = 0;
	
	for(int i=0; i <n; i++)
		num |= ((unsigned long)buf[i]) << ((3-i) * 8);
	
	if(num == 0)
		return "z";

	string r;
	r.resize(5);


	for(int i=0; i < 5; i++)
	{
		r[4-i] = (char)(num % 85 + 33);
		num /= 85;
	}

	return r;
}


void WritePimpl::output_header()
{
	//Output the required instructions. This sets up a 72 dpi image, the ``right'' way up, 
	//going from 0,0 - xs, ys. Since numbers are 72 per inch, the image is 72 dpi
	o << " 0 " << ys << " translate 1 -1 scale " << xs << " " << ys << " 8 matrix  currentfile\n"
		" /ASCII85Decode filter false " << m_channels << " colorimage\n";


	// <xsize> <ysize> <bits per channel> <transform> <datasrc n> ... <datasrc 1> <multi> <num of channels> colorimage
	// transform is an affine transformation matrix. We use the identity here, 
	// and preform the the transform before the image, so that any subsequent
	// graphics appear in the coordinate frame of the image.

}



WritePimpl::WritePimpl(std::ostream& out, ImageRef size, const string& type, const string& extra_header)
:o(out)
{
	xs = size.x;
	ys = size.y;
	num_in_buf = lines = 0;


	if(type == "unsigned char")
		m_channels = 1;
	else if(type == "CVD::Rgb<unsigned char>")
		m_channels = 3;
	else
		throw Exceptions::Image_IO::UnsupportedImageSubType("PS", type);

	out << extra_header;
	output_header();
	currentline.reserve(80);
}

WritePimpl::~WritePimpl()
{
	//Is there anything left in the buffer?
	if(num_in_buf != 0)
			currentline += bytes_to_base85(num_in_buf);

	//Output if there is anything left
	if(currentline.size())
		o << currentline << endl;

	//Delimit the data
	if(lines == ys)
		o << "~>\n";
}

void WritePimpl::write_raw_pixel_lines(const unsigned char* data, unsigned long nlines)
{
	if((long)nlines + lines > ys)
		throw CVD::Exceptions::Image_IO::InternalLibraryError("CVD", "Write past end of image.");

	const unsigned char* end = data + (nlines * xs)*m_channels;	
	const unsigned char* d = data;

	for(; d < end; d++)
	{
		//Stuff 4 bytes in to the buffer
		buf[num_in_buf++] = *d;

		//Stuff the processed bytes in to a string, if need be
		if(num_in_buf == 4)
		{
			currentline += bytes_to_base85(num_in_buf);
			num_in_buf = 0;
		}

		//Output the line of it is wide enough
		if(currentline.size() >= 75)
		{
			o << currentline << endl;
			currentline.resize(0);
		}
	}
	
	lines += nlines;
	
}

////////////////////////////////////////////////////////////////////////////////
//
// Public interfaces to image writing.
//

writer::writer(ostream& o, ImageRef size, const string& s, const map<string, Parameter<> >&)
:t(new WritePimpl(o, size, s))
{}

writer::~writer()
{}

void writer::write_raw_pixel_line(const byte* data)
{
	t->write_raw_pixel_lines(data, 1);
}

void writer::write_raw_pixel_line(const Rgb<byte>* data)
{
	t->write_raw_pixel_lines((byte*)data, 1);
}


string eps_header(ImageRef s)
{
	ostringstream os;
	output_eps_header(os, s);
	return os.str();
}

eps_writer::eps_writer(ostream& o, ImageRef size, const string& s, const map<string, Parameter<> >&)
:t(new WritePimpl(o, size, s, eps_header(size)))
{}

eps_writer::~eps_writer()
{
	ostream& o = t->o;
	//Destruct the PS writer to print the footer.,
	t.reset();	
	output_eps_footer(o);
}

void eps_writer::write_raw_pixel_line(const byte* data)
{
	t->write_raw_pixel_lines(data, 1);
}

void eps_writer::write_raw_pixel_line(const Rgb<byte>* data)
{
	t->write_raw_pixel_lines((byte*)data, 1);
}

}
}
