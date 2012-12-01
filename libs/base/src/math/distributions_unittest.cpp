/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


#include <mrpt/base.h>
#include <mrpt/math/distributions.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils::metaprogramming;
using namespace std;

const double eps = 1e-12;

TEST(distributions,normalPDF_1d)
{
	EXPECT_NEAR( normalPDF(0,0,1),0.398942280401433, eps);
	EXPECT_NEAR( normalPDF(5,5,1),0.398942280401433, eps);
	EXPECT_NEAR( normalPDF(0,0,2),0.199471140200716, eps);
	EXPECT_NEAR( normalPDF(1,0,1),0.241970724519143, eps);
}

TEST(distributions,normalPDF_vector)
{
	const double cov_vals [3*3] = {
		4.0,  2.0,  1.0,
		2.0,  3.0,  0.5,
		1.0,  0.5,  1.0
	};
	const double x1_vals[3] = {1.0, 0.0, 0.0};
	const double x2_vals[3] = {1.0, 2.0, 3.0};

	const CMatrixDouble33  COV(cov_vals);
	const CMatrixFixedNumeric<double,3,1> x0;
	const CMatrixFixedNumeric<double,3,1> x1(x1_vals);
	const CMatrixFixedNumeric<double,3,1> x2(x2_vals);

	EXPECT_NEAR( normalPDF(x0,x0,COV), 0.02592116832548877620, eps); // sprintf('%.20f',mvnpdf([0;0;0],[0;0;0],S))
	EXPECT_NEAR( normalPDF(x2,x2,COV), 0.02592116832548877620, eps); // sprintf('%.20f',mvnpdf([0;0;0],[0;0;0],S))

	EXPECT_NEAR( normalPDF(x1,x0,COV), 0.02061240910323311470, eps); // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
	EXPECT_NEAR( normalPDF(x2,x0,COV), 0.00008423820480102986, eps); // sprintf('%.20f',mvnpdf([1;2;3],[0;0;0],S))

	EXPECT_NEAR( normalPDF(x1,COV), 0.02061240910323311470, eps); // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
	EXPECT_NEAR( normalPDF(x2,COV), 0.00008423820480102986, eps); // sprintf('%.20f',mvnpdf([1;0;0],[0;0;0],S))
}

TEST(distributions,erfc)
{
	EXPECT_NEAR( mrpt::math::erfc(0), 1, eps );
	EXPECT_NEAR( mrpt::math::erfc(1), 0.157299207050285, eps );
	EXPECT_NEAR( mrpt::math::erfc(2), 0.004677734981047, eps );
}

TEST(distributions,erf)
{
	EXPECT_NEAR( mrpt::math::erf(0), 0, eps );
	EXPECT_NEAR( mrpt::math::erf(1), 0.842700792949715, eps );
	EXPECT_NEAR( mrpt::math::erf(2), 0.995322265018953, eps );
}

TEST(distributions,normalCDF)
{
	EXPECT_NEAR( mrpt::math::normalCDF(0), 0.5 , eps );
	EXPECT_NEAR( mrpt::math::normalCDF(1), 0.841344746068543 , eps );
	EXPECT_NEAR( mrpt::math::normalCDF(2), 0.977249868051821 , eps );
	EXPECT_NEAR( mrpt::math::normalCDF(3), 0.998650101968370 , eps );
}

TEST(distributions,chi2inv)
{
	EXPECT_NEAR( mrpt::math::chi2inv(0.0, 1), 0 , eps );
	EXPECT_NEAR( mrpt::math::chi2inv(1e-4, 1), 1.570796335019566e-08 , eps );
	EXPECT_NEAR( mrpt::math::chi2inv(0.95, 1), 3.841458820694126 , eps );
	EXPECT_NEAR( mrpt::math::chi2inv(0.95, 2), 5.991464547107981 , eps );
	EXPECT_NEAR( mrpt::math::chi2inv(0.95, 3), 7.814727903251178 , eps );
}

TEST(distributions,chi2PDF)
{
	EXPECT_NEAR( mrpt::math::chi2PDF(1.0, 1), 0.241970724519143 , eps );
	EXPECT_NEAR( mrpt::math::chi2PDF(2.0, 1), 0.103776874355149 , eps );
	EXPECT_NEAR( mrpt::math::chi2PDF(3.0, 1), 0.051393443267923 , eps );
	EXPECT_NEAR( mrpt::math::chi2PDF(4.0, 1), 0.026995483256594 , eps );
}
