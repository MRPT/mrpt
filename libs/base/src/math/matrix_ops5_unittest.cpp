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

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.


#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils::metaprogramming;
using namespace std;


TEST(Matrices, loadFromArray)
{
	EIGEN_ALIGN16 const double nums[3*4] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12 };

	CMatrixFixedNumeric<double,3,4> mat;
	mat.loadFromArray(nums);

	for (int r=0;r<3;r++)
		for (int c=0;c<4;c++)
			EXPECT_EQ( nums[4*r+c], mat(r,c) );
}

TEST(Matrices, CMatrixFixedNumeric_loadWithEigenMap)
{
	EIGEN_ALIGN16 double nums[3*4] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12 };

	// Row major
	const CMatrixFixedNumeric<double,3,4> mat = Eigen::Map<CMatrixFixedNumeric<double,3,4>::Base, Eigen::Aligned  >(nums);

	for (int r=0;r<3;r++)
		for (int c=0;c<4;c++)
			EXPECT_EQ( nums[4*r+c], mat(r,c) );
}

TEST(Matrices, EigenMatrix_loadWithEigenMap)
{
	EIGEN_ALIGN16 double nums[3*4] = {
		1,2,3,4,
		5,6,7,8,
		9,10,11,12 };
	// Col major
	const Eigen::Matrix<double,3,4> mat = Eigen::Map<Eigen::Matrix<double,3,4>, Eigen::Aligned >(nums);

	for (int r=0;r<3;r++) // Transposed!!
		for (int c=0;c<4;c++)
			EXPECT_EQ( nums[3*c+r], mat(r,c) );
}
