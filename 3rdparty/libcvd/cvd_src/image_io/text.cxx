#include "cvd/image_io.h"
#include <iterator>
#include <vector>
#include <sstream>
#include <iostream>

using namespace CVD;
using namespace CVD::TEXT;
using namespace CVD::Exceptions::Image_IO;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of TIFF reading
//

class CVD::TEXT::ReadPimpl
{
	public:
		ReadPimpl(istream&);
		~ReadPimpl();
		ImageRef size();
		string datatype();
		void get_raw_pixel_line(double* data);

	private:
		istream& i;
		unsigned long row;
		ImageRef my_size;

		vector<vector<double> > raster_data;
};


void ReadPimpl::get_raw_pixel_line(double* d)
{
	if(row  > (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Read past end of image.");
	
	copy(raster_data[row].begin(), raster_data[row].end(), d);
	row ++;
}

string ReadPimpl::datatype()
{
	return "double";
}

ImageRef ReadPimpl::size()
{
	return my_size;
}

ReadPimpl::~ReadPimpl()
{	
}


ReadPimpl::ReadPimpl(istream& is)
:i(is),row(0)
{
	my_size.x = -1;
	my_size.y = 0;

	string line;
	raster_data.reserve(8192);

	for(;;)
	{
		getline(i, line);

		if(i.fail())
			break;
		
		istringstream l(line);
		raster_data.resize(my_size.y + 1);
		copy(istream_iterator<double>(l), istream_iterator<double>(), back_inserter(raster_data.back()));

		if(raster_data.back().size() == 0)
			break;
		
		if(my_size.y == 0)
			my_size.x = raster_data[0].size();
		else if(my_size.x != (int)raster_data.back().size())
		{
			ostringstream err;
			err << "All rows must have the same number of columns: bad row is " 
				<< my_size.y;
			throw MalformedImage(err.str());
		}

		my_size.y++;
	}
}




////////////////////////////////////////////////////////////////////////////////
//
// Implementation of public parts of text reading
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

bool reader::top_row_first()
{
	return true;
}
string reader::name()
{
	return "TEXT";
}

ImageRef reader::size()
{
	return t->size();
};

//Mechanically generate the pixel reading calls.
void reader::get_raw_pixel_line(double*d){t->get_raw_pixel_line(d);}
