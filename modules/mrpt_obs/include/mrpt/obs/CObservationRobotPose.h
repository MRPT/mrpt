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
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** An observation providing an alternative robot pose from an external source.
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationRobotPose : public CObservation
{
  DEFINE_SERIALIZABLE(CObservationRobotPose, mrpt::obs)
 public:
  /** The observed robot pose */
  mrpt::poses::CPose3DPDFGaussian pose;

  /** The pose of the sensor on the robot/vehicle */
  mrpt::poses::CPose3D sensorPose;

  void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override;  // See base class docs.
  void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override;   // See base class docs.
  void getDescriptionAsText(std::ostream& o) const override;                // See base class docs

  // See base class docs:
  bool exportTxtSupported() const override { return true; }
  std::string exportTxtHeader() const override;
  std::string exportTxtDataRow() const override;

};  // End of class def.

}  // namespace mrpt::obs
