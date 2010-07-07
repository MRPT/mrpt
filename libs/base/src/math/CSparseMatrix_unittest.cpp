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


#include <mrpt/math_mrpt.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


void generateRandomSparseMatrix(size_t N, size_t M, CSparseMatrix &MAT)
{

}


void do_test_init_to_unit(size_t N)
{
	CMatrixDouble	 dense1;
	dense1.unit(N);

	CSparseMatrix SM(dense1);

	CMatrixDouble    dense_out;
	SM.get_dense(dense_out);

	EXPECT_TRUE( dense_out==dense1 ) <<
		"Failed with N=" << N << "\n";
}

TEST(SparseMatrix, InitFromDenseUnit)
{
	do_test_init_to_unit(1);
	do_test_init_to_unit(10);
	do_test_init_to_unit(100);
}


void do_test_init_random(size_t N)
{
	CMatrixDouble	 dense1;
	mrpt::random::randomGenerator.drawGaussian1DMatrix(dense1);
	CSparseMatrix SM(dense1);
	CMatrixDouble    dense_out;
	SM.get_dense(dense_out);
	EXPECT_TRUE( dense_out==dense1 ) <<
		"Failed with N=" << N << "\n";
}

TEST(SparseMatrix, InitFromDenseRandom)
{
	do_test_init_random(1);
	do_test_init_random(10);
	do_test_init_random(100);
}



TEST(SparseMatrix, InitFromSparse)
{
	CMatrixDouble  D(4,5);
	mrpt::math::CSparseMatrixTemplate<double>  S(4,5);
	D(1,2) = 2.0;
	S(1,2) = 2.0;

	D(3,1) = -7.0;
	S(3,1) = -7.0;

	CSparseMatrix SM(S);
	CMatrixDouble    dense_out;
	SM.get_dense(dense_out);
	EXPECT_TRUE( dense_out==D ) 
		<< "Dense: \n" << D 
		<< "Sparse:\n" << dense_out << endl;
}

