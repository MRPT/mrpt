/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	LIBRARY: mrpt_ros bridge
	FILE: gps.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros1bridge/gps.h>

bool mrpt::ros1bridge::fromROS(
	const sensor_msgs::NavSatFix& msg, mrpt::obs::CObservationGPS& obj)
{
	mrpt::obs::gnss::Message_NMEA_GGA gga;
	gga.fields.altitude_meters = msg.altitude;
	gga.fields.latitude_degrees = msg.latitude;
	gga.fields.longitude_degrees = msg.longitude;

	switch (msg.status.status)
	{
		case -1: gga.fields.fix_quality = 0; break;
		case 0: gga.fields.fix_quality = 1; break;
		case 2: gga.fields.fix_quality = 2; break;
		case 1: gga.fields.fix_quality = 3; break;
		default: gga.fields.fix_quality = 0;  // never going to execute default
	}
	obj.setMsg(gga);
	return true;
}

bool mrpt::ros1bridge::toROS(
	const mrpt::obs::CObservationGPS& obj, const std_msgs::Header& msg_header,
	sensor_msgs::NavSatFix& msg)
{
	// 1) sensor_msgs::NavSatFix:: header
	msg.header = msg_header;

	// 2) other NavSatFix Parameters, the following 3 could be wrong too

	if (obj.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
	{
		const mrpt::obs::gnss::Message_NMEA_GGA& gga =
			obj.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
		msg.altitude = gga.fields.altitude_meters;
		msg.latitude = gga.fields.latitude_degrees;
		msg.longitude = gga.fields.longitude_degrees;

		/// following parameter assigned as per
		/// http://mrpt.ual.es/reference/devel/structmrpt_1_1obs_1_1gnss_1_1_message___n_m_e_a___g_g_a_1_1content__t.html#a33415dc947663d43015605c41b0f66cb
		/// http://mrpt.ual.es/reference/devel/gnss__messages__ascii__nmea_8h_source.html
		switch (gga.fields.fix_quality)
		{
			case 0: msg.status.status = -1; break;
			case 1: msg.status.status = 0; break;
			case 2: msg.status.status = 2; break;
			case 3: msg.status.status = 1; break;
			default:
				// this is based on literature available on GPS as the number of
				// types in ROS and MRPT are not same
				msg.status.status = 0;
		}
		// this might be incorrect as there is not matching field in mrpt
		// message type
		msg.status.service = 1;
	}
	/// position_covariance is not available in mrpt
	/// position_covariance type is not available in mrpt
	return true;
}

/// NavSatFix ROS message
/*
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
*/

/// NavSatStatus ROS message
/*
int8 STATUS_NO_FIX=-1
int8 STATUS_FIX=0
int8 STATUS_SBAS_FIX=1
int8 STATUS_GBAS_FIX=2
uint16 SERVICE_GPS=1
uint16 SERVICE_GLONASS=2
uint16 SERVICE_COMPASS=4
uint16 SERVICE_GALILEO=8
int8 status
uint16 service
 */
