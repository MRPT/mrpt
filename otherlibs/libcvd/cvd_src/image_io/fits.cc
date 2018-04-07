#include "cvd_src/config_internal.h"
#include "cvd/internal/io/fits.h"
#include "cvd/image_io.h"
#include <algorithm>
#include <vector>
#include <deque>
#include <iostream>
#include <sstream>

using namespace CVD;
using namespace CVD::FITS;
using namespace CVD::Exceptions::Image_IO;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of FITS reading
//

////////////////////////////////////////////////////////////////////////////////
//
// Useful string manipulation functions
//


//Trim off whitespace on the right hand side
static string rtrim(string str)
{
	string ws (" \t\f\v\n\r");
	size_t pos;
  
	pos=str.find_last_not_of(ws);
	if (pos!=string::npos)
		str.erase(pos+1);
	else
		str.clear();

    return str;
}

//Trim off whitespace on the left hand side
static string ltrim(const string& str)
{
	string ws (" \t\f\v\n\r");
	size_t pos;
  
	pos=str.find_first_not_of(ws);
	if (pos!=string::npos)
		return string(str.begin()+pos, str.end());
	else
		return "";
}

static int get_int(const string& s)
{
	int i;
	istringstream is(s);
	is >> i;

	if(s.size() == 0 || is.bad())
		throw(Exceptions::Image_IO::MalformedImage("Invalid integer: `" + s + "'"));
	return i;
}

static int get_uint(const string& s)
{
	unsigned int i;
	istringstream is(s);
	is >> i;

	if(s.size() == 0 || is.bad())
		throw(Exceptions::Image_IO::MalformedImage("Invalid unsigned integer: `" + s + "'"));
	return i;
}

static double get_double(const string& s)
{
	double i;
	istringstream is(s);
	is >> i;

	if(s.size() == 0 || is.bad())
		throw(Exceptions::Image_IO::MalformedImage("Invalid double: `" + s + "'"));
	return i;
}


////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of FITS reading
//

class CVD::FITS::ReadPimpl
{
	public:
		ReadPimpl(istream&);
		~ReadPimpl();
		ImageRef size();
		string datatype();
		template<class C> void get_raw_pixel_line(C* data);

		template<class C> void           convert_raw_pixel_line(unsigned char*, C* data);
		template<template<class> class C> void convert_raw_pixel_line(unsigned char*, C<unsigned short>* data);
		void                             convert_raw_pixel_line(unsigned char*, unsigned short* data);


	private:
		istream& i;
		unsigned long row;
		ImageRef my_size;
		string   type;
	
		//HDU is the FITS term: header description units.
		//The HDU has multiple 36 card stacks of 80 column cards.
		//which is 2880 bytes
		char HDU_card_stack[2880], *card;

		void next_card()
		{
			if(card != NULL)
					card += 80;

			if(card == NULL || card == HDU_card_stack + 2880)
			{	
				i.read(HDU_card_stack, 2880);
				if(i.eof())
					throw(Exceptions::Image_IO::MalformedImage("EOF in header."));

				card =  HDU_card_stack;
			}
		}

		bool match_card_start(const string& s)
		{
			if(s.size() > 80)
				return 0;
			return equal(s.begin(), s.end(), card);
		}
		
		//keywords are left justified, in columns 1--8
		string get_keyword()
		{
			string k;
			k.resize(8);
			copy(card, card+8, k.begin());
			return rtrim(k);
		}

		void check_for_seperator()
		{
			if(card[8] != '=' || card[9] != ' ')
				throw(Exceptions::Image_IO::MalformedImage("Missing `= ' separator in card."));
		}

		void expect_keyword(const string& k)
		{
			check_for_seperator();
			if(k != get_keyword())
				throw Exceptions::Image_IO::MalformedImage("FITS images missing "+k+"(got" + get_keyword() + ")");
		}

		//numeric fields are right justified, in columns 11--30
		string get_numeric_field()
		{
			string f;
			f.resize(20);
			copy(card+10, card+30, f.begin());
			return ltrim(f);
		}

		vector<unsigned char> data;
		int bytes_per_pixel;

};


template<class T> void ReadPimpl::get_raw_pixel_line(T* d)
{
	if(datatype() != PNM::type_name<T>::name())
		throw ReadTypeMismatch(datatype(), PNM::type_name<T>::name());

	if(row  > (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Read past end of image.");


	unsigned char* rowp = &data[row * bytes_per_pixel * my_size.x];
	row++;

	convert_raw_pixel_line(rowp, d);
}

template<class T> void ReadPimpl::convert_raw_pixel_line(unsigned char* rowp, T* d)
{
	copy(rowp, rowp + my_size.x * bytes_per_pixel,	(unsigned char*)(d));
}

template<template<class> class T> void ReadPimpl::convert_raw_pixel_line(unsigned char* rowp, T<unsigned short>* d)
{	
	unsigned short* ds = (unsigned short*)d;
	int elements=my_size.x * sizeof(T<unsigned short>)/sizeof(unsigned short);
	for(int i=0; i < elements; i++)
		ds[i] = (unsigned short)(static_cast<int>( ((short*)(rowp))[i]) + 32768);
}


void ReadPimpl::convert_raw_pixel_line(unsigned char* rowp, unsigned short* d)
{
	for(int i=0; i < my_size.x; i++)
		d[i] = (unsigned short)(static_cast<int>( ((short*)(rowp))[i]) + 32768);
}




string ReadPimpl::datatype()
{
	return type;
}

ImageRef ReadPimpl::size()
{
	return my_size;
}

ReadPimpl::~ReadPimpl()
{	
}


ReadPimpl::ReadPimpl(istream& is)
:i(is),row(0),card(NULL)
{
	next_card();
	if(!match_card_start("SIMPLE  =                    T"))
		throw(Exceptions::Image_IO::MalformedImage("FITS images must start with \"SIMPLE  =                    T\""));

	next_card();
	expect_keyword("BITPIX");
	int bpp = get_int(get_numeric_field());
	
	if(bpp == 8)
		type = PNM::type_name<byte>::name();
	else if(bpp == 16)
		type = PNM::type_name<short>::name();
	else if(bpp == 32)
		type = PNM::type_name<int>::name();
	else if(bpp == -32)
		type = PNM::type_name<float>::name();
	else if(bpp == -64)
		type = PNM::type_name<double>::name();
	else
		throw Exceptions::Image_IO::MalformedImage("FITS images has unrecognised BITPIX (" + get_numeric_field() + ")");
		
	
	unsigned int num_axes;
	next_card();
	expect_keyword("NAXIS");
	num_axes=get_uint(get_numeric_field());

	if(num_axes < 1 || num_axes>3)
		throw Exceptions::Image_IO::UnsupportedImageSubType("FITS", get_numeric_field() + " axes given (1, 2 or 3 supported).");
	
	//Read in the axis lengths
	//select obvious defailts for missing axes
	int a1=0, a2=1, a3=1;
	next_card();
	expect_keyword("NAXIS1");
	a1 = get_uint(get_numeric_field());

	if(num_axes > 1)
	{
		next_card();
		expect_keyword("NAXIS2");
		a2 = get_uint(get_numeric_field());

		if(num_axes > 2)
		{
			next_card();
			expect_keyword("NAXIS3");
			a3 = get_uint(get_numeric_field());
		}
	}
	my_size.x = a1;
	my_size.y = a2;
	
	double add_for_zero=0;
	string bzero;

	//Read the rest of the header
	//The last useful card has END as the first keyword, but no separator.
	//The remaining cards in the current stack should be blank.
	//Since we read a stack at a time, the read pointer will be in the
	//correct place.
	for(;;)
	{
		next_card();
		if(get_keyword() == "BZERO")
		{
			check_for_seperator();
			add_for_zero = get_double(bzero=get_numeric_field());
		}
		else if(get_keyword() == "END")
			break;
	}

	if(add_for_zero == 0) //OK
	{}
	else if(add_for_zero == 32768. && bpp == 16)
		type = "unsigned short";
	else
		throw Exceptions::Image_IO::UnsupportedImageSubType("FITS", "BZERO is " + bzero + ". Only 0 or 32768 (for int16 images) supported");

	
	//Figure out the colourspace
	if(a3 == 1)
		type = type;
	else if(a3 == 2)
		type = "CVD::GreyAlpha<" + type  + ">";
	else if(a3 == 3)
		type = "CVD::Rgb<" + type  + ">";
	else if(a3 == 4)
		type = "CVD::Rgba<" + type  + ">";
	else 
		throw Exceptions::Image_IO::UnsupportedImageSubType("FITS", "3rd axis (colour planes) has"+ get_numeric_field() + " elements(1, 2, 3 or 4 supported).");


	//We should really use BZERO, too.
	
	//Images are planar. So, deplanarize here.
	//Also, correct the endianness
	//FIXME this could be much more efficient.
	bpp = abs(bpp)/8;
	
	size_t nelems = a1 * a2 * a3;
	size_t nbytes = nelems * bpp;
	vector<unsigned char> raw_data;
	raw_data.resize(nbytes);
	data.resize(nbytes);
	i.read((char*)(&raw_data[0]), nbytes);

	if(i.eof())
		throw(Exceptions::Image_IO::MalformedImage("EOF in image."));
		

	//Make the data follow the native byte ordering
	#ifdef CVD_INTERNAL_ARCH_LITTLE_ENDIAN
		for(size_t i=0; i < nelems; i++)
			reverse(&raw_data[i*bpp], &raw_data[i*bpp] + bpp);
	#elif defined CVD_INTERNAL_ARCH_BIG_ENDIAN

	#else
			#error No endianness defined.
	#endif
	
	//Convert planar to inline
	for(int c=0; c < a1; c++)
		for(int r=0; r < a2; r++)
			for(int p=0; p < a3; p++)
				for(int b=0; b < bpp; b++)
						data[((r * a1 + c)*a3 + p)*bpp + b] = raw_data[p*a1*a2*bpp+ ((a2-r-1)*a1+c)*bpp+b];
	
	bytes_per_pixel = a3 * bpp;
}




////////////////////////////////////////////////////////////////////////////////
//
// Implementation of public parts of TIFF reading
//

reader::reader(istream& i)
:t(new ReadPimpl(i))
{}

reader::~reader()
{
}

string reader::datatype()
{
	return t->datatype();
}

string reader::name()
{
	return "FITS";
}

ImageRef reader::size()
{
	return t->size();
};

bool reader::top_row_first()
{
	return true;
}

//Mechanically generate the pixel reading calls.
#define GEN1(X) void reader::get_raw_pixel_line(X*d){t->get_raw_pixel_line(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)

GEN3(unsigned char)
GEN3(signed short)
GEN3(unsigned short)
GEN3(signed int)
GEN3(float)
GEN3(double)
