/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/wrap2pi.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace std;

TEST(Wrap2PI_tests, angDistance)
{
	using mrpt::math::angDistance;

	EXPECT_NEAR( angDistance(0.0,1.0), 1.0,  1e-10);
	EXPECT_NEAR( angDistance(1.0,1.0), 0.0,  1e-10);
	EXPECT_NEAR( angDistance(1.0,0.0),-1.0,  1e-10);

	EXPECT_NEAR( angDistance( -(M_PI-0.1) ,  (M_PI-0.1)  ),-0.2,  1e-6);
	EXPECT_NEAR( angDistance(  (M_PI-0.1) , -(M_PI-0.1)  ),+0.2,  1e-6);
	
}
