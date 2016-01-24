/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationGPS_H
#define CObservationGPS_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/gnss_messages.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationGPS , CObservation, OBS_IMPEXP)

	/** This class <b>stores messages</b> from GNSS or GNSS+IMU devices, from consumer-grade inexpensive GPS receivers to Novatel/Topcon/... advanced RTK solutions.
	 *
	 *  See mrpt::hwdrivers::CGPSInterface for a class capable of reading from a serial port or any input stream and \b parsing the ASCII/binary stream into 
	 *  indivual messages \b stored in mrpt::obs::CObservationGPS objects.
	 *
	 *  Supported message types are:
	 *  - NMEA 0183 (ASCII): GGA, RMC
	 *  - Topcon GRIL (Binary): PZS, SATS
	 *  - Novatel GNSS/SPAN OEM6 (Binary): See list of log packets under namespace mrpt::obs::gnss and in enum type mrpt::obs::gnss::gnss_message_type_t
	 * 
	 *  Note that this object has \b two timestamp fields:
	 *  - The standard CObservation::timestamp field in the base class, which should contain the accurate satellite-based UTC timestamp, and 
	 *  - the field CObservationGPS::originalReceivedTimestamp, with the local computer-based timestamp based on the reception of the message in the computer.
	 *
	 * \note <b>[API changed in MRPT 1.4.0]</b> mrpt::obs::CObservationGPS now stores message objects in a more flexible way. API clean-up and extended so the number of GNSS message types is larger and more scalable.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationGPS : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationGPS )

	public:
		typedef std::map<gnss::gnss_message_type_t, gnss::gnss_message_ptr> message_list_t;

		CObservationGPS(  ); //!< ctor

		/** @name GNSS (GPS) data fields
		  * @{ */
		mrpt::poses::CPose3D     sensorPose;//!< The sensor pose on the robot/vehicle
		mrpt::system::TTimeStamp originalReceivedTimestamp; //!< The local computer-based timestamp based on the reception of the message in the computer. \sa CObservation::timestamp in the base class, which should contain the accurate satellite-based UTC timestamp.
		/** The main piece of data in this class: a list of GNNS messages.
		  * Normally users might prefer to access the list via the methods XXXXXXXX
		  * Typically only one message, may be multiple if all have the same timestamp. */
		message_list_t           messages;
		/** @} */

		/** @name Main API to access to the data fields
		  * @{ */
		void  dumpToStream( mrpt::utils::CStream &out ) const; //!< Dumps the contents of the observation in a human-readable form to a given output stream \sa dumpToConsole(), getDescriptionAsText()
		void  dumpToConsole(std::ostream &o = std::cout) const; //!< Dumps the contents of the observation in a human-readable form to an std::ostream (default=console)
		void clear(); //!< Empties this observation, clearing the container \a messages

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; } // See base class docs
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) { sensorPose = newSensorPose; } // See base class docs
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE; // See base class docs
		/** @} */

		/** @name Deprecated, backwards compatible (MRPT <1.4.0) data and types
		  * @{ */
		typedef gnss::UTC_time   TUTCTime;        //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_TopCon_PZS  TGPSDatum_PZS;  //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_TopCon_SATS TGPSDatum_SATS; //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_NMEA_GGA    TGPSDatum_GGA;  //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_NMEA_RMC    TGPSDatum_RMC;  //!< Deprecated, kept for backwards compatibility

		/** Proxy class for type-based testing existence of data inside CObservationGPS::messages */
		template <mrpt::obs::gnss::gnss_message_type_t MSG_TYPE>
		struct internal_msg_test_proxy {
			internal_msg_test_proxy(message_list_t &msgs_) : msgs(msgs_) {}
			operator bool(void) { return msgs.find(MSG_TYPE)!=msgs.end(); }
		private:
			message_list_t           &msgs;
		};

		// Was: bool  has_GGA_datum;
		internal_msg_test_proxy<gnss::NMEA_GGA>    has_GGA_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::NMEA_RMC>    has_RMC_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::TOPCON_PZS>  has_PZS_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::TOPCON_SATS> has_SATS_datum; //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		/** @} */

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationGPS , CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
