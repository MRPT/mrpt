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


const double   dat_A[] = { 4, 5, 8, -2, 1, 3 };
const double   dat_B[] = { 2, 6, 9, 8 };
const double   dat_Cok[] = {53,64, -2,32, 29,30  };


#define CHECK_AND_RET_ERROR(_COND_,_MSG_)    EXPECT_FALSE(_COND_) << _MSG_;


TEST(Matrices, setSize)
{
	{
		CMatrixFixedNumeric<double,6,6>  M;
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(6,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(10,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(5,6);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(6,6);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(10,10);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
}

TEST(Matrices, extractSubmatrixSymmetricalBlocks)
{
	{
		const double vals[] = {
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15 };
		const CMatrixFixedNumeric<double,8,8>  M(vals);

		std::vector<size_t> vs;
		vs.push_back(1);
		vs.push_back(3);

		CMatrixDouble E;
		M.extractSubmatrixSymmetricalBlocks(2,vs,E);

		const double valsE[] = {
			3,4,7,8,
			10,11,14,15,
			3,4,7,8,
			10,11,14,15 };
		const CMatrixDouble44  E_expected(valsE);

		EXPECT_TRUE( E_expected == E );
	}
}

TEST(Matrices, extractSubmatrixSymmetrical)
{
	{
		const double vals[] = {
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15 };
		const CMatrixFixedNumeric<double,8,8>  M(vals);

		std::vector<size_t> vs;
		vs.push_back(2);
		vs.push_back(3);
		vs.push_back(6);
		vs.push_back(7);

		CMatrixDouble E;
		M.extractSubmatrixSymmetrical(vs,E);

		const double valsE[] = {
			3,4,7,8,
			10,11,14,15,
			3,4,7,8,
			10,11,14,15 };
		const CMatrixDouble44  E_expected(valsE);

		EXPECT_TRUE( E_expected == E );
	}
}


