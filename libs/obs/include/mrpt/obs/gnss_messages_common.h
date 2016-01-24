/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GNSS_MESSAGES_COMMON_H
#define GNSS_MESSAGES_COMMON_H

#include <mrpt/utils/CStream.h>
#include <mrpt/system/datetime.h>
#include <iostream>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt {
namespace obs {
/** GNSS (GPS) data structures, mainly for use within mrpt::obs::CObservationGPS */
namespace gnss {

/** GNSS message types. \sa mrpt::obs::CObservationGPS */
enum gnss_message_type_t
{
	NONE                   = 0,      //!< Empty message

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
	NV_OEM6_GPGSA            =  221 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPGSV            =  223 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPHDT            = 1045 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPRMC            =  225 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPVTG            =  226 + NV_OEM6_MSG2ENUM,
	NV_OEM6_GPZDA            =  227  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARKPOS          =  181  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARK2POS         =  615  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARKTIME         =  231  + NV_OEM6_MSG2ENUM,
	NV_OEM6_MARK2TIME        =  616  + NV_OEM6_MSG2ENUM,
	NV_OEM6_PPPPOS           = 1538  + NV_OEM6_MSG2ENUM,
	NV_OEM6_VERSION          =   37  + NV_OEM6_MSG2ENUM,

	// ====== Novatel SPAN+OEM6 ====== 
	// See "SPAN on OEM6 firmware reference"

	NV_OEM6_INSPVAS      =  1305  + NV_OEM6_MSG2ENUM, // Most recent position, velocity and attitude at full rate of IMU (short header)
	NV_OEM6_INSATTS      =   319  + NV_OEM6_MSG2ENUM, // Most recent attitude (roll, pitch and azimuth) measurements (short header)
	NV_OEM6_INSCOVS      =   320  + NV_OEM6_MSG2ENUM, // Position, attitude, and velocity matrices with respect to the local level frame (short header)
	NV_OEM6_INSVELS      =   324  + NV_OEM6_MSG2ENUM, // Most recent North, East, and Up velocity vector values (short header)
	NV_OEM6_RAWIMUS      =   325  + NV_OEM6_MSG2ENUM, // IMU status indicator and the measurements from the accelerometers and gyros (short header)
};

/** Pure virtual base for all message types. \sa mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP gnss_message {
	gnss_message_type_t  message_type; //!< Type of GNSS message

	gnss_message(gnss_message_type_t msg_type_id) : message_type(msg_type_id) {}
	void writeToStream(mrpt::utils::CStream &out) const; //!< Save to binary stream. Launches an exception upon error
	void readFromStream(mrpt::utils::CStream &in); //!< Load from binary stream into this existing object. Launches an exception upon error.

	bool isOfType(const gnss_message_type_t type_id) const;
	template <class MSG_CLASS> 
	bool isOfClass() const { return isOfType(MSG_CLASS::msg_type); }

	static gnss_message* readAndBuildFromStream(mrpt::utils::CStream &in); //!< Load from binary stream and creates object detecting its type (class factory). Launches an exception upon error
	static gnss_message* Factory(const gnss_message_type_t msg_id); //!< Creates message

	virtual void dumpToStream( mrpt::utils::CStream &out ) const = 0; //!< Dumps the contents of the observation in a human-readable form to a given output stream \sa dumpToConsole()
	void dumpToConsole(std::ostream &o = std::cout) const; //!< Dumps the contents of the observation in a human-readable form to an std::ostream (default=console)
protected:
	virtual void internal_writeToStream(mrpt::utils::CStream &out) const = 0; //!< Save to binary stream. Launches an exception upon error
	virtual void internal_readFromStream(mrpt::utils::CStream &in) = 0; //!< Save to binary stream. Launches an exception upon error
};

/** A smart pointer to a GNSS message. \sa gnss_message, mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP gnss_message_ptr 
{
protected:
	gnss_message *ptr;
public:
	gnss_message_ptr(); //!< Ctor (default: NULL pointer)
	gnss_message_ptr(const gnss_message_ptr &o); //!< Makes a copy of the pointee
	/** Assigns a pointer */
	explicit gnss_message_ptr(const gnss_message* p);
	gnss_message_ptr &operator =(const gnss_message_ptr&o); // Makes a copy of the pointee
	virtual ~gnss_message_ptr();
	bool operator == ( const gnss_message *o ) const { return o==ptr; }
	bool operator == ( const gnss_message_ptr &o )const { return o.ptr==ptr; }
	bool operator != ( const gnss_message *o )const { return o!=ptr; }
	bool operator != ( const gnss_message_ptr &o )const { return o.ptr!=ptr; }
	gnss_message*& get() { return ptr; }
	const gnss_message* get()const { return ptr; }
	gnss_message *& operator ->() { ASSERT_(ptr); return ptr; }
	const gnss_message * operator ->() const  { ASSERT_(ptr); return ptr; }
};


struct OBS_IMPEXP gnss_message_binary_block : public gnss_message {
	gnss_message_binary_block(gnss_message_type_t msg_type_id,uint32_t data_len, void* data_ptr) : gnss_message(msg_type_id),m_content_len(data_len),m_content_ptr(data_ptr) {}
protected:
	void internal_writeToStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE;
	void internal_readFromStream(mrpt::utils::CStream &in) MRPT_OVERRIDE;
private:
	const uint32_t m_content_len;
	void *         m_content_ptr;
};

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push,1)

/** UTC (Coordinated Universal Time) time-stamp structure for GPS messages. \sa mrpt::obs::CObservationGPS */
struct OBS_IMPEXP UTC_time
{
	uint8_t hour;
	uint8_t minute;
	double  sec;

	UTC_time();
	mrpt::system::TTimeStamp getAsTimestamp(const mrpt::system::TTimeStamp &date) const; //!< Build an MRPT timestamp with the hour/minute/sec of this structure and the date from the given timestamp.
	bool operator == (const UTC_time& o) const { return hour==o.hour && minute==o.minute && sec==o.sec; }
	bool operator != (const UTC_time& o) const { return hour!=o.hour || minute!=o.minute || sec!=o.sec; }
	void writeToStream(mrpt::utils::CStream &out) const; //!< Save to binary stream. Launches an exception upon error
	void readFromStream(mrpt::utils::CStream &in); //!< Save to binary stream. Launches an exception upon error
};

#pragma pack(pop) // End of pack = 1
} } } // End of namespaces

#endif
