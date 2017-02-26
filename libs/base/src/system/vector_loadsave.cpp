/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

bool  mrpt::system::vectorToTextFile( const vector<float> &vec, const string &fileName, bool append, bool byRows )
{
	FILE	*f=os::fopen(fileName.c_str(), append ? "at" : "wt");
	if (!f) return false;

	for (vector<float>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%e ":"%e\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

bool  mrpt::system::vectorToTextFile( const vector<double> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<double>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%e ":"%e\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

bool  mrpt::system::vectorToTextFile( const vector<int> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<int>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%i ":"%i\n",*it);

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

bool  mrpt::system::vectorToTextFile( const vector<size_t> &vec, const string &fileName, bool append, bool byRows  )
{
	FILE	*f=os::fopen(fileName.c_str(),append ? "at" : "wt");
	if (!f) return false;

	for (vector<size_t>::const_iterator	it=vec.begin();it!=vec.end();++it)
		os::fprintf(f,byRows ? "%u ":"%u\n",static_cast<unsigned int>(*it));

	if (byRows) os::fprintf(f,"\n");

	os::fclose(f);
	return true;	// All ok.
}

bool  mrpt::system::vectorFromTextFile( std::vector<double> &vec, const std::string &fileName, bool byRows )
{
	FILE	*f = os::fopen( fileName.c_str(), "r" );
	if (!f) return false;

	double number = 0;

	while ( !feof(f) )
	{
		size_t readed = fscanf( f, byRows ? "%lf" : "%lf\n", &number );
		if ( (!byRows) || (readed == 1) )
			vec.push_back( number );
	}

	return true;
}

/*---------------------------------------------------------------
					loadBinaryFile
  ---------------------------------------------------------------*/
bool mrpt::system::loadBinaryFile( vector_byte &out_data, const std::string &fileName )
{
	try
	{
		CFileInputStream	fi(fileName);
		size_t  N = fi.getTotalBytesCount();

		out_data.resize(N);
		if (N)
		{
			size_t NN = fi.ReadBuffer( &out_data[0], N);
			return NN==N;
		}
		else return true;
	}
	catch(...) { return false; }
}

/*---------------------------------------------------------------
					vectorToBinaryFile
  ---------------------------------------------------------------*/
bool mrpt::system::vectorToBinaryFile( const vector_byte &vec, const std::string &fileName )
{
	try
	{
		mrpt::utils::CFileOutputStream	of(fileName);
		if (!vec.empty())
			of.WriteBuffer( &vec[0], sizeof(vec[0])*vec.size() );
		return true;
	}
	catch(...) { return false; }
}
