/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRobotPose, CObservation, mrpt::obs)

uint8_t CObservationRobotPose::serializeGetVersion() const { return 1; }
void CObservationRobotPose::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << pose;
  out << sensorLabel << timestamp << sensorPose;
}

void CObservationRobotPose::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      in >> pose;
      in >> sensorLabel >> timestamp;
      if (version >= 1) in >> sensorPose;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CObservationRobotPose::getSensorPose(CPose3D& out_sensorPose) const
{
  out_sensorPose = sensorPose;
}

void CObservationRobotPose::setSensorPose(const CPose3D& newSensorPose)
{
  sensorPose = newSensorPose;
}

void CObservationRobotPose::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);

  o << "Sensor pose: " << sensorPose << "\n";
  o << "Pose: " << pose.asString() << "\n";
  o << mrpt::format(
      "\n"
      "Human-readable pose uncertainty:\n"
      "sigma_x = %.03f m\n"
      "sigma_y = %.03f m\n"
      "sigma_z = %.03f m\n"
      "sigma_yaw = %.03f deg\n"
      "sigma_pitch = %.03f deg\n"
      "sigma_roll = %.03f deg\n",
      std::sqrt(pose.cov(0, 0)), std::sqrt(pose.cov(1, 1)), std::sqrt(pose.cov(2, 2)),
      mrpt::RAD2DEG(std::sqrt(pose.cov(3, 3))), mrpt::RAD2DEG(std::sqrt(pose.cov(4, 4))),
      mrpt::RAD2DEG(std::sqrt(pose.cov(5, 5))));
}

// See base class docs:
std::string CObservationRobotPose::exportTxtHeader() const
{
  return mrpt::format(
      "%18s %18s %18s %18s %18s %18s"  // POSE
      // COV
      "%18s %18s %18s %18s %18s %18s"
      "%18s %18s %18s %18s %18s"
      "%18s %18s %18s %18s"
      "%18s %18s %18s"
      "%18s %18s"
      "%18s",
      "X", "Y", "Z", "YAW_RAD", "PITCH_RAD", "ROLL_RAD",  // pose
      // cov
      "var_x", "std_x_y", "std_x_z", "std_x_yaw", "std_x_pitch",
      "std_x_roll",                                                  //
      "var_y", "std_y_z", "std_y_yaw", "std_y_pitch", "std_y_roll",  //
      "var_z", "std_z_yaw", "std_z_pitch", "std_z_roll",             //
      "var_yaw", "std_yaw_pitch", "std_yaw_roll",                    //
      "var_pitch", "std_yaw_roll",                                   //
      "var_roll");
}
std::string CObservationRobotPose::exportTxtDataRow() const
{
  const auto& C = pose.cov;
  return mrpt::format(
      "%18.5f %18.5f %18.5f %18.5f %18.5f %18.5f "  // POSE
      // cov
      "%18.5f %18.5f %18.5f %18.5f %18.5f %18.5f "
      "%18.5f %18.5f %18.5f %18.5f %18.5f "
      "%18.5f %18.5f %18.5f %18.5f "
      "%18.5f %18.5f %18.5f "
      "%18.5f %18.5f "
      "%18.5f ",
      pose.mean.x(), pose.mean.y(), pose.mean.z(), pose.mean.yaw(), pose.mean.pitch(),
      pose.mean.roll(),
      // cov
      C(0, 0), C(0, 1), C(0, 2), C(0, 3), C(0, 4), C(0, 5),  //
      C(1, 1), C(1, 2), C(1, 3), C(1, 4), C(1, 5),           //
      C(2, 2), C(2, 3), C(2, 4), C(2, 5),                    //
      C(3, 3), C(3, 4), C(3, 5),                             //
      C(4, 4), C(4, 5),                                      //
      C(5, 5));
}
