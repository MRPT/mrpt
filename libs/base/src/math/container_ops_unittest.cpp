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


#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils::metaprogramming;
using namespace std;



TEST(dynamicsize_vector,resize)
{
	{
		mrpt::vector_double v;
		EXPECT_TRUE(v.size()==0);
	}

	for (int i=0;i<10;i++)
	{
		mrpt::vector_double v(i);
		EXPECT_TRUE(v.size()==i);
	}

	for (int i=0;i<10;i++)
	{
		mrpt::vector_double v;
		v.resize(i);
		EXPECT_TRUE(v.size()==i);
	}

	for (int i=10;i>=0;i--)
	{
		mrpt::vector_double v;
		v.resize(i);
		EXPECT_TRUE(v.size()==i);
	}

	{
		mrpt::vector_double v;
		for (int i=0;i<10;i++)
		{
			v.push_back(double(i));
			EXPECT_TRUE(v.size()==(i+1));
		}
		for (int i=0;i<10;i++)
		{
			EXPECT_TRUE(v[i]==i);
		}
	}

	{
		mrpt::vector_double v;
		v.reserve(10);
		for (int i=0;i<10;i++)
		{
			v.push_back(double(i));
			EXPECT_TRUE(v.size()==(i+1));
		}
		for (int i=0;i<10;i++)
		{
			EXPECT_TRUE(v[i]==i);
		}
	}
}


