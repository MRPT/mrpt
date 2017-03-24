/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/matrix_serialization.h> // for << of matrices
#include <mrpt/utils/CMemoryStream.h>
#include <iomanip>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGPS, CObservation,mrpt::obs)

CObservationGPS::CObservationGPS( ) :
	sensorPose(),
	originalReceivedTimestamp(INVALID_TIMESTAMP),
	has_satellite_timestamp(false),
	messages(),
	has_GGA_datum (messages),
	has_RMC_datum (messages),
	has_PZS_datum (messages),
	has_SATS_datum(messages)
{
}

void CObservationGPS::swap(CObservationGPS &o)
{
	std::swap(timestamp, o.timestamp);
	std::swap(originalReceivedTimestamp, o.originalReceivedTimestamp);
	std::swap(has_satellite_timestamp,o.has_satellite_timestamp);
	std::swap(sensorLabel, o.sensorLabel);
	std::swap(sensorPose, o.sensorPose);
	messages.swap( o.messages );
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 11;
	else
	{
		out << timestamp << originalReceivedTimestamp << sensorLabel << sensorPose;
		out << has_satellite_timestamp; // v11

		const uint32_t nMsgs = messages.size();
		out << nMsgs;
		for (message_list_t::const_iterator it=messages.begin();it!=messages.end();++it)
			it->second->writeToStream(out);
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::readFromStream(mrpt::utils::CStream &in, int version)
{
	this->clear();

	switch(version)
	{
	case 10:
	case 11:
		{
			in >> timestamp >> originalReceivedTimestamp >> sensorLabel >>sensorPose;
			if (version>=11)
				in >> has_satellite_timestamp; // v11
			else has_satellite_timestamp = (this->timestamp!=this->originalReceivedTimestamp);

			uint32_t nMsgs;
			in >> nMsgs;
			for (unsigned i=0;i<nMsgs;i++) {
				gnss::gnss_message * msg = gnss::gnss_message::readAndBuildFromStream(in);
				messages[msg->message_type] = gnss::gnss_message_ptr(msg);
			}
		};
		break;

	// OLD VERSIONS: Ensure we can load datasets from many years ago ==========
	case 0:
		{
			bool has_GGA_datum_;
			in >> has_GGA_datum_;
			if (has_GGA_datum_) {
				gnss::Message_NMEA_GGA * datum = new gnss::Message_NMEA_GGA();
				in.ReadBuffer( &datum->fields, sizeof(datum->fields) );
				messages[gnss::NMEA_GGA] = gnss::gnss_message_ptr(datum);
			}

			bool has_RMC_datum_;
			in >> has_RMC_datum_;
			if (has_RMC_datum_) {
				gnss::Message_NMEA_RMC * datum = new gnss::Message_NMEA_RMC();
				in.ReadBuffer( &datum->fields, sizeof(datum->fields) );
				messages[gnss::NMEA_RMC] = gnss::gnss_message_ptr(datum);
			}
		} break;
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		{
			if (version>=3)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;

			bool has_GGA_datum_;
			in >> has_GGA_datum_;
			if (has_GGA_datum_)
			{
				gnss::Message_NMEA_GGA datum;
				gnss::Message_NMEA_GGA::content_t & GGA_datum = datum.fields;

				in  >> GGA_datum.UTCTime.hour >> GGA_datum.UTCTime.minute >> GGA_datum.UTCTime.sec >> GGA_datum.latitude_degrees
					>> GGA_datum.longitude_degrees >> GGA_datum.fix_quality >> GGA_datum.altitude_meters;
				if( version >= 9 ) {
					in  >> GGA_datum.geoidal_distance
						>> GGA_datum.orthometric_altitude
						>> GGA_datum.corrected_orthometric_altitude;
				}
				else {
					GGA_datum.geoidal_distance                  = 0.0f;
					GGA_datum.orthometric_altitude              = 0.0f;
					GGA_datum.corrected_orthometric_altitude    = 0.0f;
				}

				in  >> GGA_datum.satellitesUsed >> GGA_datum.thereis_HDOP >> GGA_datum.HDOP;
				this->setMsg(datum);
			}

			bool has_RMC_datum_;
			in >> has_RMC_datum_;
			if (has_RMC_datum_)
			{
				gnss::Message_NMEA_RMC datum;
				gnss::Message_NMEA_RMC::content_t & RMC_datum = datum.fields;

				in  >> RMC_datum.UTCTime.hour >> RMC_datum.UTCTime.minute >> RMC_datum.UTCTime.sec
					>> RMC_datum.validity_char >> RMC_datum.latitude_degrees >> RMC_datum.longitude_degrees
					>> RMC_datum.speed_knots >> RMC_datum.direction_degrees;
				this->setMsg(datum);
			}
			if (version>1)
					in >> sensorLabel;
			else 	sensorLabel = "";
			if (version>=4)
					in >> sensorPose;
			else	sensorPose.setFromValues(0,0,0,0,0,0);
			if (version>=5)
			{
				bool has_PZS_datum_;
				in >> has_PZS_datum_;
				if (has_PZS_datum_)
				{
					gnss::Message_TOPCON_PZS * datum = new gnss::Message_TOPCON_PZS();
					messages[gnss::TOPCON_PZS] = gnss::gnss_message_ptr(datum);
					gnss::Message_TOPCON_PZS & PZS_datum = *datum;

					in >>
						PZS_datum.latitude_degrees >> PZS_datum.longitude_degrees >> PZS_datum.height_meters >>
						PZS_datum.RTK_height_meters >> PZS_datum.PSigma >> PZS_datum.angle_transmitter >> PZS_datum.nId >>
						PZS_datum.Fix >> PZS_datum.TXBattery >> PZS_datum.RXBattery >> PZS_datum.error;
					// extra data?
					if (version>=6) {
						in >>
							PZS_datum.hasCartesianPosVel >> PZS_datum.cartesian_x >> PZS_datum.cartesian_y >> PZS_datum.cartesian_z >>
							PZS_datum.cartesian_vx >> PZS_datum.cartesian_vy >> PZS_datum.cartesian_vz >>
							PZS_datum.hasPosCov >> PZS_datum.pos_covariance >> PZS_datum.hasVelCov >>
							PZS_datum.vel_covariance >> PZS_datum.hasStats >> PZS_datum.stats_GPS_sats_used >> PZS_datum.stats_GLONASS_sats_used;
						if (version>=8)
							in >> PZS_datum.stats_rtk_fix_progress;
						else
							PZS_datum.stats_rtk_fix_progress=0;
					}
					else {
						PZS_datum.hasCartesianPosVel = PZS_datum.hasPosCov = PZS_datum.hasVelCov = PZS_datum.hasStats = false;
					}
				}
			} // end version >=5

			// Added in V7:
			if (version>=7) {
				gnss::Message_TOPCON_SATS * datum = new gnss::Message_TOPCON_SATS();
				messages[gnss::TOPCON_SATS] = gnss::gnss_message_ptr(datum);
				gnss::Message_TOPCON_SATS & SATS_datum = *datum;
				bool has_SATS_datum_;
				in >> has_SATS_datum_;
				if (has_SATS_datum_)
				{
					in >> SATS_datum.USIs >> SATS_datum.ELs >> SATS_datum.AZs;
				}
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

	if (originalReceivedTimestamp==INVALID_TIMESTAMP)
		originalReceivedTimestamp = timestamp;
}

/*---------------------------------------------------------------
					dumpToStream
 ---------------------------------------------------------------*/
void  CObservationGPS::dumpToStream( CStream &out ) const 
{
	out.printf("\n------------- [CObservationGPS] Dump of %u messages -----------------------\n",static_cast<unsigned int>(messages.size()));
	for (message_list_t::const_iterator it=messages.begin();it!=messages.end();++it)
		it->second->dumpToStream(out);
	out.printf("-------------- [CObservationGPS] End of dump  ------------------------------\n\n");
}

void  CObservationGPS::dumpToConsole(std::ostream &o) const
{
	mrpt::utils::CMemoryStream memStr;
	this->dumpToStream( memStr );
	if (memStr.getTotalBytesCount()) {
		o.write((const char*)memStr.getRawBufferData(),memStr.getTotalBytesCount());
	}
}

mrpt::system::TTimeStamp CObservationGPS::getOriginalReceivedTimeStamp() const {
	return originalReceivedTimestamp;
}

void CObservationGPS::clear()
{
	messages.clear();
	timestamp = INVALID_TIMESTAMP;
	originalReceivedTimestamp = INVALID_TIMESTAMP;
}
void CObservationGPS::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Timestamp (UTC) of reception at the computer: " << mrpt::system::dateTimeToString(originalReceivedTimestamp) << std::endl;
	o << "  (as time_t): " <<  std::fixed << std::setprecision(5) << mrpt::system::timestampTotime_t(originalReceivedTimestamp) << std::endl;
	o << "  (as TTimestamp): " << originalReceivedTimestamp << std::endl;

	o << "Sensor position on the robot/vehicle: " << sensorPose << std::endl;

	this->dumpToConsole(o);
}

bool CObservationGPS::hasMsgType(const gnss::gnss_message_type_t type_id) const 
{
	return messages.find(type_id)!=messages.end();
}

mrpt::obs::gnss::gnss_message* CObservationGPS::getMsgByType(const gnss::gnss_message_type_t type_id) 
{
	message_list_t::iterator it = messages.find(type_id);
	ASSERTMSG_(it!=messages.end(), mrpt::format("[CObservationGPS::getMsgByType] Cannot find any observation of type `%u`",static_cast<unsigned int>(type_id) ));
	return it->second.get();
}
/** \overload */
const mrpt::obs::gnss::gnss_message* CObservationGPS::getMsgByType(const gnss::gnss_message_type_t type_id) const 
{
	message_list_t::const_iterator it = messages.find(type_id);
	ASSERTMSG_(it!=messages.end(), mrpt::format("[CObservationGPS::getMsgByType] Cannot find any observation of type `%u`",static_cast<unsigned int>(type_id) ));
	return it->second.get();
}


// From: http://gnsstk.sourceforge.net/time__conversion_8c-source.html
#define TIMECONV_JULIAN_DATE_START_OF_GPS_TIME (2444244.5)  // [days]
bool TIMECONV_GetJulianDateFromGPSTime(	const unsigned short 	gps_week, const double 	gps_tow, const unsigned int utc_offset, double * 	julian_date)
{
	if( gps_tow < 0.0  || gps_tow > 604800.0 )
		return false;
	// GPS time is ahead of UTC time and Julian time by the UTC offset
	*julian_date = (gps_week + (gps_tow-utc_offset)/604800.0)*7.0 + TIMECONV_JULIAN_DATE_START_OF_GPS_TIME;
	return true;
}

bool TIMECONV_IsALeapYear( const unsigned short year )
{
	bool is_a_leap_year = false;
	if( (year%4) == 0 ) 
	{
		is_a_leap_year = true;
		if( (year%100) == 0 )
		{
			if( (year%400) == 0 )
			     is_a_leap_year = true;
			else is_a_leap_year = false;
		}
	}
	return is_a_leap_year;
}

bool TIMECONV_GetNumberOfDaysInMonth(
	const unsigned short year,        //!< Universal Time Coordinated    [year]
	const unsigned char month,        //!< Universal Time Coordinated    [1-12 months] 
	unsigned char* days_in_month      //!< Days in the specified month   [1-28|29|30|31 days]
	)
{
	unsigned char utmp = 0;
	bool is_a_leapyear = TIMECONV_IsALeapYear( year );

	switch(month)
	{
	case  1: utmp = 31; break;
	case  2: if( is_a_leapyear ){ utmp = 29; }else{ utmp = 28; }break;    
	case  3: utmp = 31; break;
	case  4: utmp = 30; break;
	case  5: utmp = 31; break;
	case  6: utmp = 30; break;
	case  7: utmp = 31; break;
	case  8: utmp = 31; break;
	case  9: utmp = 30; break;
	case 10: utmp = 31; break;
	case 11: utmp = 30; break;
	case 12: utmp = 31; break;
	default: return false; break;    
	}
	*days_in_month = utmp;
	return true;
}
bool TIMECONV_GetUTCTimeFromJulianDate(const double julian_date,  //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
	mrpt::system::TTimeParts &utc
	)
{
	int a, b, c, d, e; // temporary values

	unsigned short year;  
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;        
	unsigned char days_in_month = 0;  
	double td; // temporary double
	double seconds;
	bool result;

	// Check the input.
	if( julian_date < 0.0 )
		return false;

	a = (int)(julian_date+0.5);
	b = a + 1537;
	c = (int)( ((double)b-122.1)/365.25 );
	d = (int)(365.25*c);
	e = (int)( ((double)(b-d))/30.6001 );

	td      = b - d - (int)(30.6001*e) + fmod( julian_date+0.5, 1.0 );   // [days]
	day     = (unsigned char)td;   
	td     -= day;
	td     *= 24.0;        // [hours]
	hour    = (unsigned char)td;
	td     -= hour;
	td     *= 60.0;        // [minutes]
	minute  = (unsigned char)td;
	td     -= minute;
	td     *= 60.0;        // [s]
	seconds = td;
	month   = (unsigned char)(e - 1 - 12*(int)(e/14));
	year    = (unsigned short)(c - 4715 - (int)( (7.0+(double)month) / 10.0 ));
	// check for rollover issues
	if( seconds >= 60.0 )
	{
		seconds -= 60.0;
		minute++;
		if( minute >= 60 )
		{
			minute -= 60;
			hour++;
			if( hour >= 24 )
			{
				hour -= 24;
				day++;
				result = TIMECONV_GetNumberOfDaysInMonth( year, month, &days_in_month );
				if( result == false )
					return false;
				if( day > days_in_month )
				{
					day = 1;
					month++;
					if( month > 12 )
					{
						month = 1;
						year++;
					}
				}
			}
		}
	}

	utc.year   = year;
	utc.month  = month;
	utc.day    = day;
	utc.hour   = hour;
	utc.minute = minute;
	utc.second = seconds;
	return true;
}

bool CObservationGPS::GPS_time_to_UTC(uint16_t gps_week,double gps_sec,const int leap_seconds_count, mrpt::system::TTimeStamp &utc_out)
{
	 mrpt::system::TTimeParts tim;
	if (!GPS_time_to_UTC(gps_week,gps_sec,leap_seconds_count,tim)) return false;
	utc_out = mrpt::system::buildTimestampFromParts(tim);
	return true;
}

bool CObservationGPS::GPS_time_to_UTC(uint16_t gps_week,double gps_sec,const int leap_seconds_count, mrpt::system::TTimeParts &utc_out)
{
	double julian_date = 0.0; 
	if( gps_sec < 0.0  || gps_sec > 604800.0 )
		return false;
	if (!TIMECONV_GetJulianDateFromGPSTime(
		gps_week,
		gps_sec,
		leap_seconds_count,
		&julian_date ) )
		return false;
	if (!TIMECONV_GetUTCTimeFromJulianDate(julian_date,utc_out))
		return false;
	return true;
}

