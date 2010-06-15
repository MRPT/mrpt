/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils::metaprogramming;
using namespace std;


TEST(Containers,saveAndLoadTextFile)
{
	const string fil = mrpt::system::getTempFileName();

	const double vals[10] = { 1,2,3,4,5,6,7,8,9,10 };

	{
		vector_double v;
		loadVector(v,vals);
		v.saveToTextFileAsVector(fil,MATRIX_FORMAT_ENG, false);
		CMatrixFixedNumeric<double,5,2> M;
		M.loadFromTextFileAsVector(fil);
		mrpt::system::deleteFile(fil);

		vector_double diff = (v-M);
		EXPECT_EQ( diff.Abs().sumAll(), 0 );
	}
	{
		vector_double v;
		loadVector(v,vals);
		v.saveToTextFileAsVector(fil,MATRIX_FORMAT_ENG, true);
		CMatrixFixedNumeric<double,5,2> M;
		M.loadFromTextFileAsVector(fil);
		mrpt::system::deleteFile(fil);

		vector_double diff = (v-M);
		EXPECT_EQ( diff.Abs().sumAll(), 0 );
	}
}



