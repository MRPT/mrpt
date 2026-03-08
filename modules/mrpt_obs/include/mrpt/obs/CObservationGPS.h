/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/gnss_messages.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

#include <map>
#include <optional>
#include <typeinfo>

namespace mrpt::obs
{
/** Detailed fix classification, going beyond the coarse NMEA fix_quality.
 *  Populated by bridge code or drivers when the original source has
 *  this information (e.g. ROS NavSatStatus, Novatel logs, u-blox UBX).
 *  Default is UNKNOWN, meaning consumers should fall back to fix_quality.
 *
 *  \note Backwards-compatible: old code that never sets this field sees
 *        UNKNOWN and keeps reading fix_quality as before.
 * \sa CObservationGPS
 * \ingroup mrpt_obs_grp
 */
enum class GnssFixType : uint8_t
{
  UNKNOWN = 0,  ///< Not set / legacy data
  NO_FIX = 1,
  AUTONOMOUS = 2,  ///< Standard GPS/GNSS, no augmentation
  SBAS = 3,        ///< Satellite-Based Augmentation (WAAS/EGNOS/MSAS…)
  GBAS = 4,        ///< Ground-Based Augmentation (local DGNSS base)
  DGPS = 5,        ///< Generic differential (source unspecified)
  RTK_FLOAT = 6,
  RTK_FIXED = 7,
  PPP = 8,  ///< Precise Point Positioning
  DEAD_RECKONING = 9,
  SIMULATION = 10,
};

/** Bitmask flags for gnss_service_mask.
 *  Matches sensor_msgs/NavSatStatus SERVICE_* bits for ROS compatibility.
 *  Combine with bitwise OR: e.g. GnssService::GPS | GnssService::GLONASS
 * \sa CObservationGPS
 * \ingroup mrpt_obs_grp
 */
enum class GnssService : uint16_t
{
  NONE = 0x0000,
  GPS = 0x0001,      ///< USA GPS / QZSS (compatible signal)
  GLONASS = 0x0002,  ///< Russian GLONASS
  BEIDOU = 0x0004,   ///< Chinese BeiDou (also COMPASS)
  GALILEO = 0x0008,  ///< European Galileo
};

/** Bitwise OR for combining GnssService flags */
constexpr GnssService operator|(GnssService a, GnssService b) noexcept
{
  return static_cast<GnssService>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}
/** Bitwise AND for testing GnssService flags */
constexpr GnssService operator&(GnssService a, GnssService b) noexcept
{
  return static_cast<GnssService>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
}
/** True if any flag in `b` is set in `a` */
constexpr bool hasService(GnssService a, GnssService b) noexcept
{
  return (a & b) != GnssService::NONE;
}

/** This class <b>stores messages</b> from GNSS or GNSS+IMU devices, from
 * consumer-grade inexpensive GPS receivers to Novatel/Topcon/... advanced RTK
 * solutions.
 *
 *  See mrpt::hwdrivers::CGPSInterface for a class capable of reading from a
 * serial port or any input stream and \b parsing the ASCII/binary stream into
 *  indivual messages \b stored in mrpt::obs::CObservationGPS objects.
 *
 *  Supported message types are:
 *  - NMEA 0183 (ASCII): GGA, RMC, etc.
 *  - Topcon GRIL (Binary): PZS, SATS
 *  - Novatel GNSS/SPAN OEM6 (Binary): See list of log packets under namespace
 * mrpt::obs::gnss and in enum type mrpt::obs::gnss::gnss_message_type_t
 *
 *  Note that this object has \b two timestamp fields:
 *  - The standard CObservation::timestamp field in the base class, which should
 * contain the accurate satellite-based UTC timestamp, and
 *  - the field CObservationGPS::originalReceivedTimestamp, with the local
 * computer-based timestamp based on the reception of the message in the
 * computer.
 *
 * Normally, users read and write messages by means of these methods:
 *  - CObservationGPS::getMsgByClass()
 *  - CObservationGPS::setMsg()
 *  - CObservationGPS::hasMsgType()
 *  - CObservationGPS::hasMsgClass()
 *
 * There is also a CObservationGPS::fix_type field with information about the
 * source of corrections (SBAS, GBAS, standalone,...) and CObservationGPS::gnss_service_mask
 * to identify the satellite network used in the solution.
 *
 * Example access to GPS datum:
 * \code
 * mrpt::obs::CObservationGPS obs;
 * //...
 * if (obs.hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>()) {
 *   const mrpt::obs::gnss::Message_NMEA_GGA &gga =
 * o.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
 *   //gga.fields.XXX ...
 * }
 * \endcode
 *
 * \note Since MRPT 2.11.12 there is an optional field for ENU covariance for
 * easier compatibility with ROS messages.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationGPS : public CObservation
{
  DEFINE_SERIALIZABLE(CObservationGPS, mrpt::obs)

 public:
  using message_list_t = std::map<gnss::gnss_message_type_t, gnss::gnss_message_ptr>;

  /** @name GNSS (GPS) data fields
   * @{ */
  /** The sensor pose on the robot/vehicle */
  mrpt::poses::CPose3D sensorPose{};

  /** The local computer-based timestamp based on the reception of the message
   * in the computer. \sa CObservation::timestamp in the base class, which
   * should contain the accurate satellite-based UTC timestamp. */
  mrpt::system::TTimeStamp originalReceivedTimestamp{INVALID_TIMESTAMP};

  /** If true, CObservation::timestamp has been generated from accurate
   * satellite clock. Otherwise, no GPS data is available and timestamps are
   * based on the local computer clock. */
  bool has_satellite_timestamp{false};

  /** The main piece of data in this class: a list of GNSS messages.
   * Normally users might prefer to access the list via the methods
   * CObservationGPS::getMsgByClass() and CObservationGPS::setMsg()
   * Typically only one message, may be multiple if all have the same
   * timestamp. */
  message_list_t messages;

  /** If present, it defines the ENU position uncertainty.
   */
  std::optional<mrpt::math::CMatrixDouble33> covariance_enu;

  /** Richer fix type. Complements (not replaces) fix_quality in GGA.
   *  Set to UNKNOWN if the originating driver/bridge does not populate it. */
  GnssFixType fix_type = GnssFixType::UNKNOWN;

  /** Bitmask of active GNSS constellations. Zero means unknown.
   *  Example: GnssService::GPS | GnssService::GALILEO */
  GnssService gnss_service_mask = GnssService::NONE;

  /** @} */

  /** @name Main API to access to the data fields
   * @{ */
  /** Stores a message in the list \a messages, making a copy of the passed
   * object.
   * Valid message classes are those derived from
   * mrpt::obs::gnss::gnss_message. If another message of the same type
   * exists, it is overwritten. */
  template <class MSG_CLASS>
  void setMsg(const MSG_CLASS& msg)
  {
    messages[static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type)].reset(new MSG_CLASS(msg));
  }
  /** Returns true if the list \a CObservationGPS::messages contains one of
   * the requested type. \sa mrpt::obs::gnss::gnss_message_type_t,
   * CObservationGPS::getMsgByType() */
  bool hasMsgType(const gnss::gnss_message_type_t type_id) const;
  /** Like \a hasMsgType() but allows querying for message classes, from any
   * of those derived from mrpt::obs::gnss::gnss_message \sa
   * CObservationGPS::hasMsgType(),  */
  template <class MSG_CLASS>
  bool hasMsgClass() const
  {
    return hasMsgType(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
  }
  /** Returns a pointer to the message in the list CObservationGPS::messages
   * of the requested type. Users normally would prefer using
   * CObservationGPS::getMsgByClass()
   * to avoid having to perform a dynamic_cast<>() on the returned pointer.
   * \exception std::runtime_error If there is no such a message in the list.
   * Please, check existence before calling this method with
   * CObservationGPS::hasMsgType()
   * \sa mrpt::obs::gnss::gnss_message_type_t,
   * CObservationGPS::getMsgByClass(), CObservationGPS::hasMsgType() */
  mrpt::obs::gnss::gnss_message* getMsgByType(const gnss::gnss_message_type_t type_id);
  /** \overload */
  const mrpt::obs::gnss::gnss_message* getMsgByType(const gnss::gnss_message_type_t type_id) const;

  /** Returns a reference to the message in the list CObservationGPS::messages
   * of the requested class.
   * \exception std::runtime_error If there is no such a message in the list.
   * Please, check existence before calling this method with
   * CObservationGPS::hasMsgClass()
   * \sa mrpt::obs::gnss::gnss_message_type_t,
   * CObservationGPS::getMsgByType(), CObservationGPS::hasMsgType() */
  template <class MSG_CLASS>
  MSG_CLASS& getMsgByClass()
  {
    auto it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
    ASSERTMSG_(
        it != messages.end(), mrpt::format(
                                  "[CObservationGPS::getMsgByClass] Cannot "
                                  "find any observation of type `%s`",
                                  typeid(MSG_CLASS).name()));
    ASSERT_(it->second.get());
    return *dynamic_cast<MSG_CLASS*>(it->second.get());
  }
  /** \overload */
  template <class MSG_CLASS>
  const MSG_CLASS& getMsgByClass() const
  {
    auto it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
    ASSERTMSG_(
        it != messages.end(), mrpt::format(
                                  "[CObservationGPS::getMsgByClass] Cannot "
                                  "find any observation of type `%s`",
                                  typeid(MSG_CLASS).name()));
    ASSERT_(it->second.get());
    return *dynamic_cast<const MSG_CLASS*>(it->second.get());
  }

  /** Like CObservationGPS::getMsgByClass() but returns a nullptr pointer if
   * message is not found, instead of launching an exception */
  template <class MSG_CLASS>
  MSG_CLASS* getMsgByClassPtr()
  {
    auto it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
    return it == messages.end() ? static_cast<MSG_CLASS*>(nullptr)
                                : dynamic_cast<MSG_CLASS*>(it->second.get());
  }
  /** \overload */
  template <class MSG_CLASS>
  const MSG_CLASS* getMsgByClassPtr() const
  {
    auto it = messages.find(static_cast<gnss::gnss_message_type_t>(MSG_CLASS::msg_type));
    return it == messages.end() ? static_cast<const MSG_CLASS*>(nullptr)
                                : dynamic_cast<const MSG_CLASS*>(it->second.get());
  }

  /** Dumps the contents of the observation in a human-readable form to a
   * given output stream \sa dumpToConsole(), getDescriptionAsText() */
  void dumpToStream(std::ostream& out) const;
  /** Dumps the contents of the observation in a human-readable form to an
   * std::ostream (use std::cout to print to console) */
  void dumpToConsole(std::ostream& o) const;
  /** Empties this observation, clearing the container \a messages */
  void clear();

  void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
  {
    out_sensorPose = sensorPose;
  }  // See base class docs
  void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
  {
    sensorPose = newSensorPose;
  }                                                           // See base class docs
  void getDescriptionAsText(std::ostream& o) const override;  // See base class docs

  mrpt::system::TTimeStamp getOriginalReceivedTimeStamp() const override;  // See base class docs

  /** Returns a best-effort GnssFixType inferred from GGA fix_quality alone.
   *  Use fix_type directly when available; this is the fallback for
   *  legacy observations where fix_type == UNKNOWN. */
  static GnssFixType inferFixTypeFromGgaQuality(uint8_t fix_quality)
  {
    switch (fix_quality)
    {
      case 0:
        return GnssFixType::NO_FIX;
      case 1:
        return GnssFixType::AUTONOMOUS;
      case 2:
        return GnssFixType::DGPS;  // ambiguous: could be SBAS or GBAS
      case 3:
        return GnssFixType::AUTONOMOUS;  // PPS
      case 4:
        return GnssFixType::RTK_FIXED;
      case 5:
        return GnssFixType::RTK_FLOAT;
      case 6:
        return GnssFixType::DEAD_RECKONING;
      case 8:
        return GnssFixType::SIMULATION;
      default:
        return GnssFixType::UNKNOWN;
    }
  }

  /** Returns fix_type if set, otherwise infers it from GGA fix_quality. */
  GnssFixType getBestFixType() const
  {
    if (fix_type != GnssFixType::UNKNOWN)
    {
      return fix_type;
    }
    if (const auto* gga = getMsgByClassPtr<gnss::Message_NMEA_GGA>())
    {
      return inferFixTypeFromGgaQuality(gga->fields.fix_quality);
    }
    return GnssFixType::UNKNOWN;
  }

  /** @} */

  /** true if the corresponding field exists in \a messages. */
  bool has_GGA_datum() const { return messages.find(gnss::NMEA_GGA) != messages.end(); }
  /** true if the corresponding field exists in \a messages. */
  bool has_RMC_datum() const { return messages.find(gnss::NMEA_RMC) != messages.end(); }

  /** @name Utilities
   * @{ */
  static bool GPS_time_to_UTC(
      uint16_t gps_week,
      double gps_sec,
      const int leap_seconds_count /**< [in] GPS to UTC time number of leap
                      seconds (normally grabbed from
                      satellital live data) */
      ,
      /** Return false on invalid input data */
      mrpt::system::TTimeStamp& utc_out /**< [out] UTC timestamp */);
  /** \overload */
  static bool GPS_time_to_UTC(
      uint16_t gps_week,
      double gps_sec,
      const int leap_seconds_count,
      mrpt::system::TTimeParts& utc_out);
  /** @} */
};  // End of class def.

}  // namespace mrpt::obs

MRPT_ENUM_TYPE_BEGIN(mrpt::obs::GnssService)
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssService, NONE);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssService, GPS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssService, GLONASS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssService, BEIDOU);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssService, GALILEO);
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::obs::GnssFixType)
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, UNKNOWN);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, NO_FIX);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, AUTONOMOUS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, SBAS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, GBAS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, DGPS);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, RTK_FLOAT);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, RTK_FIXED);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, PPP);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, DEAD_RECKONING);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::GnssFixType, SIMULATION);
MRPT_ENUM_TYPE_END()
