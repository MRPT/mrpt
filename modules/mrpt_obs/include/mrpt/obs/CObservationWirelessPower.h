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

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** This represents a measurement of the wireless strength perceived by the
 * robot.
 *  The signal level is given as a percentage.
 *
 * \sa CObservation, mrpt::hwdrivers::CWirelessPower for a software sensor
 * capable of reading this kind of observations.
 * \ingroup mrpt_obs_grp
 */
class CObservationWirelessPower : public CObservation
{
  DEFINE_SERIALIZABLE(CObservationWirelessPower, mrpt::obs)

 public:
  /** @name The data members
   * @{ */

  /** The power or signal strength as sensed by the Wifi receiver (In
   * percentage: [0-100]) */
  double power{0};
  /** The location of the sensing antenna on the robot coordinate framework */
  mrpt::poses::CPose3D sensorPoseOnRobot;

  /** @} */

  void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override;  // See base class docs
  void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override;   // See base class docs
  void getDescriptionAsText(std::ostream& o) const override;                // See base class docs

  // See base class docs:
  bool exportTxtSupported() const override { return true; }
  std::string exportTxtHeader() const override;
  std::string exportTxtDataRow() const override;

};  // End of class def.

}  // namespace mrpt::obs
