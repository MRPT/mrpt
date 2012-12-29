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


#include <mrpt/hwdrivers/CGPSInterface.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace std;

TEST(CGPSInterface, NMEA_parser)
{
	// Test with a correct frame:
	{
		const char *test_cmd = "$GPGGA,101830.00,3649.76162994,N,00224.53709052,W,2,08,1.1,9.3,M,47.4,M,5.0,0120*58";
		mrpt::slam::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

		EXPECT_TRUE(obsGPS.has_GGA_datum);
		EXPECT_NEAR(obsGPS.GGA_datum.latitude_degrees, 36+49.76162994/60.0,1e-10);
		EXPECT_NEAR(obsGPS.GGA_datum.longitude_degrees, -(002+24.53709052/60.0),1e-10);
		EXPECT_NEAR(obsGPS.GGA_datum.altitude_meters, 9.3,1e-10);
	}

	// Test with an empty frame:
	{
		const char *test_cmd = "$GPGGA,,,,,,0,,,,M,,M,,*6";
		mrpt::slam::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_FALSE(parse_ret);
	}
}
