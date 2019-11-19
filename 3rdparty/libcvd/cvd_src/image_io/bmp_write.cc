#include "cvd/internal/load_and_save.h"
#include "cvd/internal/io/bmp.h"
#include <vector>
using namespace std;
using namespace CVD;

namespace CVD{
namespace BMP{

void writeBMPHeader(unsigned int width, unsigned int height, unsigned int channels, std::ostream& out);

class WritePimpl
{
	public:	
		WritePimpl(ostream& o_, ImageRef sz, const string& t)
		:o(o_),size(sz)
		{
			int ch;

			if(t == "unsigned char")
				ch=1;
			else if(t == "CVD::Rgb<unsigned char>")
			{
				ch=3;
				int l=size.x*3;
				if(l%4 != 0)
					l = (l/4+1)*4;
				buf.resize(l);
			}
			else
				throw Exceptions::Image_IO::UnsupportedImageSubType("BMP", t);

			writeBMPHeader(size.x, size.y, ch, o);

			pad = ((ch*size.x) % 4) ? (4 - ((ch*size.x) % 4)) : 0;
		}

		void write_raw_pixel_line(const byte* r)
		{
			char zeros[]={0,0,0,0};
			o.write((const char*)r, size.x);
			o.write(zeros,pad);
		}

		void write_raw_pixel_line(const Rgb<byte>* r)
		{
			//Transform to BGR
			for(int i=0; i < size.x; i++)
			{
				buf[i*3+0] = r[i].blue;
				buf[i*3+1] = r[i].green;
				buf[i*3+2] = r[i].red;
			}
			o.write(&buf[0], buf.size());
		}

	private:
		ostream& o;
		ImageRef size;
		int pad;
		vector<char> buf;


};



Writer::Writer(std::ostream&o, ImageRef size, const std::string& type, const std::map<std::string, Parameter<> >&)
:t(new WritePimpl(o, size, type))
{}

Writer::~Writer()
{}

void Writer::write_raw_pixel_line(const byte* l)
{
	t->write_raw_pixel_line(l);
}
void Writer::write_raw_pixel_line(const Rgb<byte>* l)
{
	t->write_raw_pixel_line(l);
}


}
}
