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
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRange, CObservation, mrpt::obs)

uint8_t CObservationRange::serializeGetVersion() const { return 4; }
void CObservationRange::serializeTo(mrpt::serialization::CArchive& out) const
{
  // The data
  out << minSensorDistance << maxSensorDistance << sensorConeAperture;
  const uint32_t n = sensedData.size();
  out << n;
  for (uint32_t i = 0; i < n; i++)
    out << sensedData[i].sensorID << CPose3D(sensedData[i].sensorPose)
        << sensedData[i].sensedDistance << sensedData[i].sensorNoiseStdDeviation;  // v4
  out << sensorLabel << timestamp;
}

void CObservationRange::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    {
      uint32_t i, n;

      // The data
      in >> minSensorDistance >> maxSensorDistance >> sensorConeAperture;

      in >> n;
      sensedData.clear();
      sensedData.resize(n);
      CPose3D aux;
      for (i = 0; i < n; i++)
      {
        if (version >= 3)
          in >> sensedData[i].sensorID;
        else
          sensedData[i].sensorID = i;

        in >> aux >> sensedData[i].sensedDistance;
        sensedData[i].sensorPose = aux.asTPose();

        if (version >= 4) in >> sensedData[i].sensorNoiseStdDeviation;
      }

      if (version >= 1)
        in >> sensorLabel;
      else
        sensorLabel = "";

      if (version >= 2)
        in >> timestamp;
      else
        timestamp = INVALID_TIMESTAMP;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

/*---------------------------------------------------------------
           getSensorPose
 ---------------------------------------------------------------*/
void CObservationRange::getSensorPose(CPose3D& out_sensorPose) const
{
  if (!sensedData.empty())
    out_sensorPose = CPose3D(sensedData[0].sensorPose);
  else
    out_sensorPose = CPose3D(0, 0, 0);
}

/*---------------------------------------------------------------
           setSensorPose
 ---------------------------------------------------------------*/
void CObservationRange::setSensorPose(const CPose3D& newSensorPose)
{
  for (auto& sd : sensedData) sd.sensorPose = newSensorPose.asTPose();
}

void CObservationRange::getDescriptionAsText(std::ostream& o) const
{
  using namespace std;
  CObservation::getDescriptionAsText(o);

  o << "minSensorDistance       = " << minSensorDistance << " m\n";
  o << "maxSensorDistance       = " << maxSensorDistance << " m\n";
  o << "sensorConeAperture     = " << RAD2DEG(sensorConeAperture) << " deg\n";

  // For each entry in this sequence:
  o << "  SENSOR_ID    RANGE (m)   STD_DEV (m)  SENSOR POSE (on the robot) "
       "\n";
  o << "-------------------------------------------------------\n";
  for (const auto& q : sensedData)
  {
    o << format("     %7u", (unsigned int)q.sensorID);
    o << format("    %4.03f   ", q.sensedDistance);
    o << format("    %4.03f   ", q.sensorNoiseStdDeviation);
    o << q.sensorPose << "\n";
  }
}
