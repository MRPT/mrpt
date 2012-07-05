/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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


