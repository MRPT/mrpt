#include "cvd/internal/io/fits.h"
#include "cvd/image_io.h"
#include "cvd_src/config_internal.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

using namespace CVD;
using namespace CVD::FITS;
using namespace CVD::Exceptions::Image_IO;
using namespace std;


////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of TIFF reading
//

class CVD::FITS::WritePimpl
{
	public:
		WritePimpl(ostream&, ImageRef size, const string& t);
		~WritePimpl();

		template<class C> void write_raw_pixel_line(const C* data);
		template<class C> void write_raw_pixel_line_(const C* data);
		void write_raw_pixel_line_(const unsigned short* dat);
		void write_raw_pixel_line_(const Rgb<unsigned short>* dat);
		void write_raw_pixel_line_(const Rgba<unsigned short>* dat);
		void axes(const string& s, int x, int y);
		void write(const string& s);
		void write_short(short s);
	private:
		ostream& o;
		ImageRef my_size;
		string   type;
		unsigned long row;
		unsigned int cards;
		int bpp, a1, a2, a3;
		bool raw;
		vector<unsigned char> data;
};

void WritePimpl::write(const string& s)
{
	string header;
	header.resize(80);
	fill(header.begin(), header.end(), ' ');
	copy(s.begin(), s.end(), header.begin());
	o << header;
	cards++;
}

string int8(int i)
{
	ostringstream o;
	o << setw(8) << setfill(' ') << i;
	return o.str();
}

void WritePimpl::axes(const string& s, int x, int y)
{
	if(s.substr(0,9) == "CVD::Rgba")
	{
		write("NAXIS   =                    3"); 
		write("NAXIS1  =             " + int8(x)); 
		write("NAXIS2  =             " + int8(y)); 
		write("NAXIS3  =                    4"); 
		a3=4;

	}
	else if(s.substr(0,8) == "CVD::Rgb")
	{
		write("NAXIS   =                    3"); 
		write("NAXIS1  =             " + int8(x)); 
		write("NAXIS2  =             " + int8(y)); 
		write("NAXIS3  =                    3"); 
		a3=3;
	}
	else
	{
		write("NAXIS   =                    2"); 
		write("NAXIS1  =             " + int8(x)); 
		write("NAXIS2  =             " + int8(y)); 
		a3=1;
	}
	a1=x;
	a2=y;
}

template<class C> void WritePimpl::write_raw_pixel_line(const C* dat)
{
	//Do some type checking
	if(type != PNM::type_name<C>::name())
		throw WriteTypeMismatch(type, PNM::type_name<C>::name());
	
	//Do some sanity checking
	if(row >= (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Write past end of image.");
	
	write_raw_pixel_line_(dat);
	row++;
}
		

template<class C> void WritePimpl::write_raw_pixel_line_(const C* dat)
{
	const unsigned char* d = reinterpret_cast<const unsigned char*>(dat);
	for(unsigned int i=0; i < my_size.x * sizeof(C); i++)
		data.push_back(d[i]);
}

void WritePimpl::write_short(short s)
{
	int num = (int)s - 32768;
	short ns = num;
	data.push_back((ns&0xff00)>>8);
	data.push_back(ns&0xff);
}

void WritePimpl::write_raw_pixel_line_(const unsigned short* dat)
{
	for(int i=0; i < my_size.x; i++)
		write_short(dat[i]);
}

void WritePimpl::write_raw_pixel_line_(const Rgb<unsigned short>* dat)
{
	for(int i=0; i < my_size.x; i++)
	{
		write_short(dat[i].red);
		write_short(dat[i].green);
		write_short(dat[i].blue);
	}
}
void WritePimpl::write_raw_pixel_line_(const Rgba<unsigned short>* dat)
{
	for(int i=0; i < my_size.x; i++)
	{
		write_short(dat[i].red);
		write_short(dat[i].green);
		write_short(dat[i].blue);
		write_short(dat[i].alpha);
	}
}


WritePimpl::WritePimpl(ostream& os, ImageRef s, const string& t)
:o(os),my_size(s),type(t),row(0),cards(0)
{
	raw=1;
	write("SIMPLE  =                    T");

	if(type == "unsigned char" || type == "CVD::Rgb<unsigned char>" || type == "CVD::Rgba<unsigned char>")
	{
		write("BITPIX  =                    8"); 
		axes(type, s.x, s.y);
		bpp=1;
	}
	else if(type == "short" || type == "CVD::Rgb<short>" || type == "CVD::Rgba<short>")
	{
		write("BITPIX  =                   16"); 
		axes(type, s.x, s.y);
		bpp=2;
	}
	else if(type == "unsigned short" || type == "CVD::Rgb<unsigned short>" || type == "CVD::Rgba<unsigned short>")
	{
		write("BITPIX  =                   16"); 
		axes(type, s.x, s.y);
		write("BZERO   =                32768"); 
		bpp=2;
		raw=0;
	}
	else if(type == "int" || type == "CVD::Rgb<int>" || type == "CVD::Rgba<int>")
	{
		write("BITPIX  =                   32"); 
		axes(type, s.x, s.y);
		bpp=4;
	}
	else if(type == "float" || type == "CVD::Rgb<float>" || type == "CVD::Rgba<float>")
	{
		write("BITPIX  =                  -32"); 
		axes(type, s.x, s.y);
		bpp=4;
	}
	else if(type == "double" || type == "CVD::Rgb<double>" || type == "CVD::Rgba<double>")
	{
		write("BITPIX  =                  -64"); 
		axes(type, s.x, s.y);
		bpp=8;
	}
	else
	{
		throw UnsupportedImageSubType("FITS", t);
	}

	write("END");

	while(cards % 36)
		write("");
}

WritePimpl::~WritePimpl()
{	
	//Make the data big endian
	#ifdef CVD_INTERNAL_ARCH_LITTLE_ENDIAN
		size_t nelems = data.size() / bpp;
		if(raw)
			for(size_t i=0; i < nelems; i++)
				reverse(&data[i*bpp], &data[i*bpp] + bpp);
	#elif defined CVD_INTERNAL_ARCH_BIG_ENDIAN

	#else 
		#error No endianness specified
	#endif

	vector<unsigned char> raw_data(data.size());

	//Convert inline to planar and flip
	for(int c=0; c < a1; c++)
		for(int r=0; r < a2; r++)
			for(int p=0; p < a3; p++)
				for(int b=0; b < bpp; b++)
						raw_data[p*a1*a2*bpp+ (r*a1+c)*bpp+b] = data[(((a2-r-1) * a1 + c)*a3 + p)*bpp + b];
	
	o.write((char*)&raw_data[0], raw_data.size());
}



////////////////////////////////////////////////////////////////////////////////
//
// Implementation of public parts of FITS reading
//

CVD::FITS::writer::writer(ostream& o, ImageRef size, const string& type, const map<string, Parameter<> >&)
:t(new WritePimpl(o, size, type))
{}

CVD::FITS::writer::~writer()
{
	delete t;
}

//Mechanically generate the pixel reading calls.
#define GEN1(X) void CVD::FITS::writer::write_raw_pixel_line(const X*d){t->write_raw_pixel_line(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)

GEN3(unsigned char)
GEN3(unsigned short)
GEN3(short)
GEN3(int)
GEN3(float)
GEN3(double)






















