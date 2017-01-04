/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

namespace mrpt {
namespace obs {
namespace gnss {

/** List of all known GNSS message types.
  * Normally, each type here has a corresponding class, derived from mrpt::obs::gnss::gnss_message,
  * that stores the message data, but some classes may be still in the "TO-DO" list or just not needed in practice.
  * On the other hand, \b all message classes \b must be associated with one and only one value from this list.
  * \sa mrpt::obs::CObservationGPS, mrpt::obs::gnss::gnss_message
  */
enum gnss_message_type_t
{
	// ====== NMEA ====== 
	NMEA_GGA               = 10,
	NMEA_GLL,
	NMEA_GSA,
	NMEA_GSV,
	NMEA_MSS,
	NMEA_RMC,
	NMEA_VTG,
	NMEA_ZDA,

	// ====== TopCon mmGPS ====== 
	TOPCON_PZS               = 30,
	TOPCON_SATS,

	// ====== Novatel OEM6 ====== 
	// See "OEM6 Family Firmware Reference Manual"
	NV_OEM6_MSG2ENUM         = 1000,

	NV_OEM6_GENERIC_FRAME       = 3000 + NV_OEM6_MSG2ENUM,  // Generic container
	NV_OEM6_GENERIC_SHORT_FRAME = 3001 + NV_OEM6_MSG2ENUM,  // Generic container (short header)


	NV_OEM6_ALIGNBSLNENU     = 1315 + NV_OEM6_MSG2ENUM,
	NV_OEM6_ALIGNBSLNXYZ     = 1314 + NV_OEM6_MSG2ENUM,
	NV_OEM6_ALIGNDOP         = 1332 + NV_OEM6_MSG2ENUM,
	NV_OEM6_BESTPOS          =   42 + NV_OEM6_MSG2ENUM,   // SPAN: Best available combined GNSS and INS position
	NV_OEM6_BESTSATS         = 1194 + NV_OEM6_MSG2ENUM,
	NV_OEM6_BESTUTM          =  726 + NV_OEM6_MSG2ENUM,
	NV_OEM6_BESTVEL          =   99 + NV_OEM6_MSG2ENUM,
	NV_OEM6_BESTXYZ          =  241 + NV_OEM6_MSG2ENUM,
	NV_OEM6_CLOCKSTEERING    =   26 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPGLL            =  219 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPGGA            =  218 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPGGARTK         =  259 + NV_OEM6_MSG2ENUM,  // More decimal digits than regular GGA
	NV_OEM6_GPGSA            =  221 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPGSV            =  223 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPHDT            = 1045 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPRMC            =  225 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPVTG            =  226 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPZDA            =  227  + NV_OEM6_MSG2ENUM,
	NV_OEM6_IONUTC           =    8  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARKPOS          =  181  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARK2POS         =  615  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARKTIME         =  231  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARK2TIME        =  616  + NV_OEM6_MSG2ENUM,
	NV_OEM6_PPPPOS           = 1538  + NV_OEM6_MSG2ENUM,
	NV_OEM6_RANGECMP         =  140  + NV_OEM6_MSG2ENUM,
	NV_OEM6_RAWEPHEM         =   41  + NV_OEM6_MSG2ENUM,
	NV_OEM6_RXSTATUS         =   93  + NV_OEM6_MSG2ENUM,
	NV_OEM6_VERSION          =   37  + NV_OEM6_MSG2ENUM,

	// ====== Novatel SPAN+OEM6 ====== 
	// See "SPAN on OEM6 firmware reference"

	NV_OEM6_INSPVAS      =   508  + NV_OEM6_MSG2ENUM, // Most recent position, velocity and attitude at full rate of IMU (short header)
	NV_OEM6_INSATTS      =   319  + NV_OEM6_MSG2ENUM, // Most recent attitude (roll, pitch and azimuth) measurements (short header)
	NV_OEM6_INSCOVS      =   320  + NV_OEM6_MSG2ENUM, // Position, attitude, and velocity matrices with respect to the local level frame (short header)
	NV_OEM6_INSVELS      =   324  + NV_OEM6_MSG2ENUM, // Most recent North, East, and Up velocity vector values (short header)
	NV_OEM6_RAWIMUS      =   325  + NV_OEM6_MSG2ENUM  // IMU status indicator and the measurements from the accelerometers and gyros (short header)

};

} } } // End of namespaces

