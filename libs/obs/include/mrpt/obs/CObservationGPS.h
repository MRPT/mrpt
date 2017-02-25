/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
#include <typeinfo>

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
	 * Normally, users read and write messages by means of these methods:
	 *  - CObservationGPS::getMsgByClass()
	 *  - CObservationGPS::setMsg()
	 *  - CObservationGPS::hasMsgType()
	 *  - CObservationGPS::hasMsgClass()
	 *
	 * Example access to GPS datum:
	 * \code
	 * mrpt::obs::CObservationGPS obs;
	 * //...
	 * if (obs.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>()) {
	 *   const mrpt::obs::gnss::Message_NMEA_GGA &gga = o.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
	 *   //gga.fields.XXX ...
	 * }
	 * \endcode
	 *
	 * \note <b>[API changed in MRPT 1.4.0]</b> mrpt::obs::CObservationGPS now stores message objects in a more flexible way. API clean-up and extended so the number of GNSS message types is larger and more scalable.
	 * \note Porting old code: For example, replace `observation.GGA_datum.XXX` with `observation.getMsgByClass<gnss::Message_NMEA_GGA>().fields.XXX`, etc.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationGPS : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationGPS )

	public:
		typedef std::map<gnss::gnss_message_type_t, gnss::gnss_message_ptr> message_list_t;

		CObservationGPS(); //!< ctor

		/** @name GNSS (GPS) data fields
		  * @{ */
		mrpt::poses::CPose3D     sensorPose;//!< The sensor pose on the robot/vehicle
		mrpt::system::TTimeStamp originalReceivedTimestamp; //!< The local computer-based timestamp based on the reception of the message in the computer. \sa CObservation::timestamp in the base class, which should contain the accurate satellite-based UTC timestamp.
		bool                     has_satellite_timestamp;   //!< If true, CObservation::timestamp has been generated from accurate satellite clock. Otherwise, no GPS data is available and timestamps are based on the local computer clock.
		/** The main piece of data in this class: a list of GNNS messages.
		  * Normally users might prefer to access the list via the methods CObservationGPS::getMsgByClass() and CObservationGPS::setMsg()
		  * Typically only one message, may be multiple if all have the same timestamp. */
		message_list_t           messages;
		/** @} */

		/** @name Main API to access to the data fields
		  * @{ */
		/** Stores a message in the list \a messages, making a copy of the passed object. 
		  * Valid message classes are those derived from mrpt::obs::gnss::gnss_message. If another message of the same type exists, it is overwritten. */
		template <class MSG_CLASS>
		void setMsg(const MSG_CLASS &msg) {
			messages[static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type)].set(new MSG_CLASS(msg));
		}
		/** Returns true if the list \a CObservationGPS::messages contains one of the requested type. \sa mrpt::obs::gnss::gnss_message_type_t, CObservationGPS::getMsgByType() */
		bool hasMsgType(const gnss::gnss_message_type_t type_id) const;
		/** Like \a hasMsgType() but allows querying for message classes, from any of those derived from mrpt::obs::gnss::gnss_message \sa CObservationGPS::hasMsgType(),  */
		template <class MSG_CLASS> bool hasMsgClass() const { return hasMsgType(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type)); }
		/** Returns a pointer to the message in the list CObservationGPS::messages of the requested type. Users normally would prefer using CObservationGPS::getMsgByClass() 
		  * to avoid having to perform a dynamic_cast<>() on the returned pointer.
		  * \exception std::runtime_error If there is no such a message in the list. Please, check existence before calling this method with CObservationGPS::hasMsgType()
		  * \sa mrpt::obs::gnss::gnss_message_type_t, CObservationGPS::getMsgByClass(), CObservationGPS::hasMsgType() */
		mrpt::obs::gnss::gnss_message* getMsgByType(const gnss::gnss_message_type_t type_id);
		/** \overload */
		const mrpt::obs::gnss::gnss_message* getMsgByType(const gnss::gnss_message_type_t type_id) const;

		/** Returns a reference to the message in the list CObservationGPS::messages of the requested class.
		  * \exception std::runtime_error If there is no such a message in the list. Please, check existence before calling this method with CObservationGPS::hasMsgClass()
		  * \sa mrpt::obs::gnss::gnss_message_type_t, CObservationGPS::getMsgByType(), CObservationGPS::hasMsgType() */
		template <class MSG_CLASS>
		MSG_CLASS & getMsgByClass() {
			message_list_t::iterator it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
			ASSERTMSG_(it!=messages.end(), mrpt::format("[CObservationGPS::getMsgByClass] Cannot find any observation of type `%s`",typeid(MSG_CLASS).name()));
			ASSERT_(it->second.get());
			return *dynamic_cast<MSG_CLASS*>(it->second.get());
		}
		/** \overload */
		template <class MSG_CLASS>
		const MSG_CLASS & getMsgByClass() const {
			message_list_t::const_iterator it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
			ASSERTMSG_(it!=messages.end(), mrpt::format("[CObservationGPS::getMsgByClass] Cannot find any observation of type `%s`",typeid(MSG_CLASS).name()));
			ASSERT_(it->second.get());
			return *dynamic_cast<const MSG_CLASS*>(it->second.get());
		}

		/** Like CObservationGPS::getMsgByClass() but returns a NULL pointer if message is not found, instead of launching an exception */
		template <class MSG_CLASS>
		MSG_CLASS * getMsgByClassPtr() {
			message_list_t::iterator it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
			return it==messages.end() ? reinterpret_cast<MSG_CLASS*>(NULL) : dynamic_cast<MSG_CLASS*>(it->second.get());
		}
		/** \overload */
		template <class MSG_CLASS>
		const MSG_CLASS * getMsgByClassPtr() const {
			message_list_t::const_iterator it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
			return it==messages.end() ? dynamic_cast<MSG_CLASS*>(NULL) : dynamic_cast<MSG_CLASS*>(it->second.get());
		}

		void dumpToStream( mrpt::utils::CStream &out ) const; //!< Dumps the contents of the observation in a human-readable form to a given output stream \sa dumpToConsole(), getDescriptionAsText()
		void dumpToConsole(std::ostream &o = std::cout) const; //!< Dumps the contents of the observation in a human-readable form to an std::ostream (default=console)
		void clear(); //!< Empties this observation, clearing the container \a messages
		void swap(CObservationGPS &o);

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE { out_sensorPose = sensorPose; } // See base class docs
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE { sensorPose = newSensorPose; } // See base class docs
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE; // See base class docs

		mrpt::system::TTimeStamp getOriginalReceivedTimeStamp() const MRPT_OVERRIDE; // See base class docs
		/** @} */

		/** @name Deprecated, backwards compatible (MRPT <1.4.0) data and types
		  * @{ */
		typedef gnss::UTC_time   TUTCTime;        //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_TOPCON_PZS  TGPSDatum_PZS;  //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_TOPCON_SATS TGPSDatum_SATS; //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_NMEA_GGA    TGPSDatum_GGA;  //!< Deprecated, kept for backwards compatibility
		typedef gnss::Message_NMEA_RMC    TGPSDatum_RMC;  //!< Deprecated, kept for backwards compatibility

		/** Proxy class for type-based testing existence of data inside CObservationGPS::messages */
		template <mrpt::obs::gnss::gnss_message_type_t MSG_TYPE>
		struct internal_msg_test_proxy {
			internal_msg_test_proxy(message_list_t &msgs_) : msgs(msgs_) {}
			operator bool(void) const { return msgs.find(MSG_TYPE)!=msgs.end(); }
			internal_msg_test_proxy<MSG_TYPE> & operator =( const internal_msg_test_proxy<MSG_TYPE> &) { return *this; }
		private:
			message_list_t           &msgs;
		};

		// Was: bool  has_GGA_datum;
		internal_msg_test_proxy<gnss::NMEA_GGA>    has_GGA_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::NMEA_RMC>    has_RMC_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::TOPCON_PZS>  has_PZS_datum;  //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		internal_msg_test_proxy<gnss::TOPCON_SATS> has_SATS_datum; //!< Evaluates as a bool; true if the corresponding field exists in \a messages.
		/** @} */

		/** @name Utilities
		  * @{ */
		static bool GPS_time_to_UTC(
			uint16_t gps_week,double gps_sec,
			const int leap_seconds_count /**< [in] GPS to UTC time number of leap seconds (normally grabbed from satellital live data) */,
			mrpt::system::TTimeStamp &utc_out /**< [out] UTC timestamp */ ); //!< Return false on invalid input data
		static bool GPS_time_to_UTC(uint16_t gps_week,double gps_sec,const int leap_seconds_count, mrpt::system::TTimeParts &utc_out); //!< \overload
		/** @} */
	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationGPS , CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
