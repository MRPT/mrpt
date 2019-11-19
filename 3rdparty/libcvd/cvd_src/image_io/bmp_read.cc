#include "cvd/internal/io/bmp.h"

#include "cvd/image_io.h"
using namespace std;
#include <iostream>

namespace CVD
{
namespace BMP
{

void readBMPHeader(unsigned int& width, unsigned int& height, unsigned int& channels, unsigned int& compression, istream& in);

class ReadPimpl
{
	public:
		ReadPimpl(std::istream&);
		ImageRef size(){ return s;}
		void get_raw_pixel_lines(unsigned char*, unsigned long nlines);
		string datatype()
		{
			return type;
		}
		
		void get_raw_pixel_line(byte* d)
		{
			char dummy[4];
			//Read the data
			i.read((char*)d, s.x);
			
			//Eat the padding
			i.read(dummy, rowSize - s.x);
		}

		void get_raw_pixel_line(Rgb<byte>* d)
		{
			char dummy[4];
			//if(channels == 1)
				//Depalette image
				//Not supported
			
			i.read((char*)d, s.x*3);
			//Turn bgr into rgb
			for(int x=0; x < s.x; x++)
				swap(d[x].red, d[x].blue);

			//Eat the padding
			i.read(dummy, rowSize - s.x*3);
		}

		template<class T> void get_raw_pixel_line(T* d)
		{
			if(datatype() != PNM::type_name<T>::name())
				throw CVD::Exceptions::Image_IO::ReadTypeMismatch(datatype(), PNM::type_name<T>::name());
			get_raw_pixel_line(d);
			//FIXME: check rows.
		}

	private:
		ImageRef s;
		std::istream&	i;
		string type;
		int channels;
		int rowSize;
		bool notgray;
		//Rgb<byte> palette[256];
};

ReadPimpl::ReadPimpl(std::istream& in)
:i(in)
{
	unsigned int w,h,ch, comp;
	readBMPHeader(w,h,ch,comp,in);

	channels=ch;
	s.x = w;
	s.y = h;

	if(channels == 1)
	{
		//Read the palette. Greyscale BMPs are implemented using palettes
		notgray = false;
		for (int i=0; i<256; i++) {
			byte buf[4];
			in.read((char*)buf,4);
			//palette[i].red = buf[2];
			//palette[i].green = buf[1];
			//palette[i].blue = buf[0];
			if (buf[0] != i || buf[0] != buf[1] || buf[1] != buf[2])
				notgray = true;
		}

		if(notgray)
			throw(Exceptions::Image_IO::UnsupportedImageSubType("Windows BMP/DIB", "Nontrivial paletted images."));
		
		rowSize = s.x;
	}
	else
		rowSize = s.x*3;

	if (rowSize % 4)
		rowSize += 4 - (rowSize%4);

	if(channels == 3 || notgray)
		type = PNM::type_name<Rgb<byte> >::name();
	else
		type = PNM::type_name<byte>::name();
}

////////////////////////////////////////////////////////////////////////////////
//
// The public interface
//

ImageRef Reader::size()
{
	return t->size();
}

void Reader::get_raw_pixel_line(unsigned char* d)
{
	t->get_raw_pixel_line(d);
}

void Reader::get_raw_pixel_line(Rgb<byte>* d)
{
	t->get_raw_pixel_line(d);
}

string Reader::datatype()
{
	return t->datatype();
}


bool Reader::top_row_first()
{
	return false;
}
string Reader::name()
{
	return "BMP";
}

Reader::~Reader()
{}

Reader::Reader(std::istream& i)
:t(new ReadPimpl(i))
{}

}
}
