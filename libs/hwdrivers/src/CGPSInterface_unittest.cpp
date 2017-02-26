/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/hwdrivers/CGPSInterface.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace std;

// Example cmds: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf

TEST(CGPSInterface, parse_NMEA_GGA)
{
	// Test with a correct frame:
	{
		const char *test_cmd = "$GPGGA,101830.00,3649.76162994,N,00224.53709052,W,2,08,1.1,9.3,M,47.4,M,5.0,0120*58";
		mrpt::obs::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

		const gnss::Message_NMEA_GGA * msg = obsGPS.getMsgByClassPtr<gnss::Message_NMEA_GGA>();
		EXPECT_TRUE(msg!=NULL);
		if (!msg) return;
		EXPECT_NEAR(msg->fields.latitude_degrees, 36+49.76162994/60.0,1e-10);
		EXPECT_NEAR(msg->fields.longitude_degrees, -(002+24.53709052/60.0),1e-10);
		EXPECT_NEAR(msg->fields.altitude_meters, 9.3,1e-10);
	}
	// Test with an empty frame:
	{
		const char *test_cmd = "$GPGGA,,,,,,0,,,,M,,M,,*6";
		mrpt::obs::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_FALSE(parse_ret);
	}
}

TEST(CGPSInterface, parse_NMEA_RMC)
{
	const char *test_cmd = "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10";
	mrpt::obs::CObservationGPS obsGPS;
	const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
	EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

	const gnss::Message_NMEA_RMC * msg = obsGPS.getMsgByClassPtr<gnss::Message_NMEA_RMC>();

	EXPECT_TRUE(msg!=NULL);
	if (!msg) return;
	EXPECT_NEAR(msg->fields.latitude_degrees, 37+ 23.2475/60.0,1e-10);
	EXPECT_NEAR(msg->fields.longitude_degrees, -(121+58.3416/60.0),1e-10);
}

TEST(CGPSInterface, parse_NMEA_GLL)
{
	const char *test_cmd = "$GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41";
	mrpt::obs::CObservationGPS obsGPS;
	const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
	EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

	const gnss::Message_NMEA_GLL * msg = obsGPS.getMsgByClassPtr<gnss::Message_NMEA_GLL>();

	EXPECT_TRUE(msg!=NULL);
	if (!msg) return;
	EXPECT_NEAR(msg->fields.latitude_degrees, 37+ 23.2475/60.0,1e-10);
	EXPECT_NEAR(msg->fields.longitude_degrees, -(121+58.3416/60.0),1e-10);
}

TEST(CGPSInterface, parse_NMEA_VTG)
{
	const char *test_cmd = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48";
	mrpt::obs::CObservationGPS obsGPS;
	const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
	EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

	const gnss::Message_NMEA_VTG * msg = obsGPS.getMsgByClassPtr<gnss::Message_NMEA_VTG>();

	EXPECT_TRUE(msg!=NULL);
	if (!msg) return;
	EXPECT_NEAR(msg->fields.true_track, 54.7,1e-6);
	EXPECT_NEAR(msg->fields.magnetic_track, 34.4,1e-6);
	EXPECT_NEAR(msg->fields.ground_speed_knots, 5.5,1e-6);
	EXPECT_NEAR(msg->fields.ground_speed_kmh, 10.2,1e-6);
}

TEST(CGPSInterface, parse_NMEA_ZDA)
{
	const char *test_cmd = "$GPZDA,181813,14,10,2003,00,00*4F";
	mrpt::obs::CObservationGPS obsGPS;
	const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
	EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

	const gnss::Message_NMEA_ZDA * msg = obsGPS.getMsgByClassPtr<gnss::Message_NMEA_ZDA>();

	EXPECT_TRUE(msg!=NULL);
	if (!msg) return;
	EXPECT_TRUE(msg->fields.date_day==14);
	EXPECT_TRUE(msg->fields.date_month==10);
	EXPECT_TRUE(msg->fields.date_year==2003);
	EXPECT_TRUE(msg->fields.UTCTime.hour==18);
	EXPECT_TRUE(msg->fields.UTCTime.minute==18);
	EXPECT_TRUE(msg->fields.UTCTime.sec==13.0); // Replaced from EXPECT_EQ() to avoid a "bus error" in a gtest template under armhf.
}
