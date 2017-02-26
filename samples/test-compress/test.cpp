/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/vector_loadsave.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/compress.h>
#include <cstdio>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		if (argc<2)
		{
			cerr << "Usage: test-compress <input_file> [compression level 1-9]" << endl;
			return -1;
		}

		vector_byte	buf;

		if (!mrpt::system::loadBinaryFile( buf, argv[1]))
		{
			cerr << "Error loading file: " << argv[1] << endl;
			return -1;
		}

		string	gzfile = format("%s.gz",argv[1]);
		int		compress_level = 9;
		if (argc>=3)
		{
			compress_level = atoi( argv[2] );
		}
		CTicTac	tictac;

		tictac.Tic();

		if (!mrpt::compress::zip::compress_gz_file( gzfile, buf, compress_level))
		{
			cerr << "Error writing compressing file: " << gzfile << endl;
			return -1;
		}

		double t = tictac.Tac();
		cout << format("Compressed %s (compress level=%i) in %.04f seconds.",gzfile.c_str(),compress_level,t) << endl;

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
