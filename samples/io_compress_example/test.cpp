/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/vector_loadsave.h>
#include <mrpt/io/zip.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/core/format.h>
#include <cstdio>
#include <iostream>

using namespace mrpt;
using namespace std;

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		if (argc < 2)
		{
			cerr << "Usage: test-compress <input_file> [compression level 1-9]"
				 << endl;
			return -1;
		}

		std::vector<uint8_t> buf;

		if (!mrpt::io::loadBinaryFile(buf, argv[1]))
		{
			cerr << "Error loading file: " << argv[1] << endl;
			return -1;
		}

		string gzfile = format("%s.gz", argv[1]);
		int compress_level = 9;
		if (argc >= 3)
		{
			compress_level = atoi(argv[2]);
		}
		mrpt::system::CTicTac tictac;

		tictac.Tic();

		if (!mrpt::io::zip::compress_gz_file(gzfile, buf, compress_level))
		{
			cerr << "Error writing compressing file: " << gzfile << endl;
			return -1;
		}

		double t = tictac.Tac();
		cout << format(
					"Compressed %s (compress level=%i) in %.04f seconds.",
					gzfile.c_str(), compress_level, t)
			 << endl;

		return 0;
	}
	catch (const std::exception& e)
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
