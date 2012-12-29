/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
