/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/base.h>

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
