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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/round.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>

#include <cmath>
#include <ostream>

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mrpt::maps::CLandmarksMap,landmarksMap", mrpt::maps::CLandmarksMap)

mrpt::maps::CLandmarksMap::TMapDefinition::TMapDefinition() = default;

void mrpt::maps::CLandmarksMap::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  const std::string sSectCreation = sectionNamePrefix + "_creationOpts";
  initialBeacons.clear();
  const int nBeacons = source.read_int(sSectCreation, "nBeacons", 0);
  for (int q = 1; q <= nBeacons; q++)
  {
    std::pair<mrpt::math::TPoint3D, CLandmark::TLandmarkID> b;
    b.second = source.read_int(sSectCreation, mrpt::format("beacon_%03i_ID", q), 0);
    b.first.x = source.read_double(sSectCreation, mrpt::format("beacon_%03i_x", q), 0);
    b.first.y = source.read_double(sSectCreation, mrpt::format("beacon_%03i_y", q), 0);
    b.first.z = source.read_double(sSectCreation, mrpt::format("beacon_%03i_z", q), 0);
    initialBeacons.push_back(b);
  }

  likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix + "_likelihoodOpts");
}

void mrpt::maps::CLandmarksMap::TMapDefinition::dumpToTextStream_map_specific(
    std::ostream& out) const
{
  out << mrpt::format("number of initial beacons = %u\n", unsigned(initialBeacons.size()));
  for (const auto& b : initialBeacons)
  {
    out << mrpt::format(
        "  ID=%03i (%8.03f,%8.03f,%8.03f)\n", static_cast<int>(b.second), b.first.x, b.first.y,
        b.first.z);
  }
  likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr mrpt::maps::CLandmarksMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const auto& def = dynamic_cast<const CLandmarksMap::TMapDefinition&>(_def);
  auto obj = CLandmarksMap::Create();

  for (const auto& b : def.initialBeacons)
  {
    CLandmark lm;
    lm.ID = b.second;
    lm.pose_mean = b.first;
    lm.pose_cov_11 = lm.pose_cov_22 = lm.pose_cov_33 = mrpt::square(0.01);
    obj->landmarks.push_back(lm);
  }

  obj->likelihoodOptions = def.likelihoodOpts;
  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CLandmarksMap, CMetricMap, mrpt::maps)

void mrpt::maps::CLandmarksMap::TLikelihoodOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& section)
{
  MRPT_LOAD_CONFIG_VAR(beaconRangesStd, float, source, section);
  MRPT_LOAD_CONFIG_VAR(beaconRangesUseObservationStd, bool, source, section);
}

void mrpt::maps::CLandmarksMap::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
  out << "\n----------- [CLandmarksMap::TLikelihoodOptions] ------------\n\n";
  LOADABLEOPTS_DUMP_VAR(beaconRangesStd, float);
  LOADABLEOPTS_DUMP_VAR(beaconRangesUseObservationStd, bool);
}

bool mrpt::maps::CLandmarksMap::internal_canComputeObservationLikelihood(
    const mrpt::obs::CObservation& obs) const
{
  return obs.GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationBeaconRanges);
}

double mrpt::maps::CLandmarksMap::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& robotPose) const
{
  const auto* o = dynamic_cast<const mrpt::obs::CObservationBeaconRanges*>(&obs);
  if (!o)
  {
    return 0;
  }

  const double sensorStd = likelihoodOptions.beaconRangesUseObservationStd
                               ? o->stdError
                               : likelihoodOptions.beaconRangesStd;
  ASSERT_GT_(sensorStd, .0);

  double ret = 0;
  for (const auto& meas : o->sensedData)
  {
    if (std::isnan(meas.sensedDistance))
    {
      continue;
    }

    const auto* lm = landmarks.getByBeaconID(meas.beaconID);
    if (lm)
    {
      // Gaussian likelihood of the sensed range:
      const auto sensor3D = robotPose + meas.sensorLocationOnRobot;
      const double expectedRange = sensor3D.distanceTo(lm->pose_mean);
      const double sensedDist = std::max<double>(.0, meas.sensedDistance);
      ret += -0.5 * mrpt::square((expectedRange - sensedDist) / sensorStd);
    }
    else if (o->maxSensorDistance > o->minSensorDistance)
    {
      // Unknown beacon: uniform likelihood
      ret += std::log(1.0 / (o->maxSensorDistance - o->minSensorDistance));
    }
  }
  return ret;
}

mrpt::maps::CLandmarksMap::CLandmarksMap() = default;

uint8_t mrpt::maps::CLandmarksMap::serializeGetVersion() const { return 0; }

void mrpt::maps::CLandmarksMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  const auto n = static_cast<uint32_t>(landmarks.size());
  out << n;
  for (uint32_t i = 0; i < n; i++)
  {
    const auto& lm = *landmarks.get(i);
    out << lm.ID;
    out << lm.pose_mean.x << lm.pose_mean.y << lm.pose_mean.z;
    out << lm.pose_cov_11 << lm.pose_cov_22 << lm.pose_cov_33;
    out << lm.pose_cov_12 << lm.pose_cov_13 << lm.pose_cov_23;
    out << static_cast<uint32_t>(lm.seenTimesCount);
  }
}

void mrpt::maps::CLandmarksMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      uint32_t n;
      in >> n;
      landmarks.clear();
      for (uint32_t i = 0; i < n; i++)
      {
        CLandmark lm;
        in >> lm.ID;
        in >> lm.pose_mean.x >> lm.pose_mean.y >> lm.pose_mean.z;
        in >> lm.pose_cov_11 >> lm.pose_cov_22 >> lm.pose_cov_33;
        in >> lm.pose_cov_12 >> lm.pose_cov_13 >> lm.pose_cov_23;
        uint32_t cnt;
        in >> cnt;
        lm.seenTimesCount = cnt;
        landmarks.push_back(lm);
      }
      break;
    }
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void mrpt::maps::CLandmarksMap::simulateRangeBearingReadings(
    const mrpt::poses::CPose3D& in_robotPose,
    const mrpt::poses::CPose3D& in_sensorLocationOnRobot,
    mrpt::obs::CObservationBearingRange& out_Observations,
    bool sensorDetectsIDs,
    double in_stdRange,
    double in_stdYaw,
    double in_stdPitch,
    std::vector<size_t>* out_real_associations,
    double spurious_count_mean,
    double spurious_count_std) const
{
  using mrpt::random::getRandomGenerator;

  if (out_real_associations) out_real_associations->clear();

  // Compute the 3D position of the sensor:
  const mrpt::poses::CPose3D sensorPose = in_robotPose + in_sensorLocationOnRobot;

  // Clear output data:
  out_Observations.validCovariances = false;
  out_Observations.sensor_std_range = static_cast<float>(in_stdRange);
  out_Observations.sensor_std_yaw = static_cast<float>(in_stdYaw);
  out_Observations.sensor_std_pitch = static_cast<float>(in_stdPitch);
  out_Observations.sensedData.clear();
  out_Observations.timestamp = mrpt::Clock::now();
  out_Observations.sensorLocationOnRobot = in_sensorLocationOnRobot;

  // For each landmark in the map:
  size_t idx = 0;
  for (const auto& lm : landmarks)
  {
    // Compute range, yaw, pitch from sensor pose to landmark:
    double range, yaw, pitch;
    sensorPose.sphericalCoordinates(lm.pose_mean, range, yaw, pitch);

    // Add noise:
    range += in_stdRange * getRandomGenerator().drawGaussian1D_normalized();
    yaw += in_stdYaw * getRandomGenerator().drawGaussian1D_normalized();
    pitch += in_stdPitch * getRandomGenerator().drawGaussian1D_normalized();

    yaw = mrpt::math::wrapToPi(yaw);
    range = std::max(0.0, range);

    if (range >= out_Observations.minSensorDistance &&
        range <= out_Observations.maxSensorDistance &&
        std::abs(yaw) <= 0.5f * out_Observations.fieldOfView_yaw &&
        std::abs(pitch) <= 0.5f * out_Observations.fieldOfView_pitch)
    {
      mrpt::obs::CObservationBearingRange::TMeasurement newMeas;
      newMeas.landmarkID =
          sensorDetectsIDs ? static_cast<int32_t>(lm.ID) : mrpt::obs::INVALID_LANDMARK_ID;
      newMeas.range = static_cast<float>(range);
      newMeas.yaw = static_cast<float>(yaw);
      newMeas.pitch = static_cast<float>(pitch);

      out_Observations.sensedData.push_back(newMeas);
      if (out_real_associations) out_real_associations->push_back(idx);
    }
    ++idx;
  }

  // Spurious detections:
  const double fSpurious =
      getRandomGenerator().drawGaussian1D(spurious_count_mean, spurious_count_std);
  size_t nSpurious = 0;
  if (spurious_count_std != 0 || spurious_count_mean != 0)
    nSpurious = static_cast<size_t>(std::max(0L, mrpt::round_long(fSpurious)));

  for (size_t i = 0; i < nSpurious; i++)
  {
    const double range = getRandomGenerator().drawUniform(
        out_Observations.minSensorDistance, out_Observations.maxSensorDistance);
    const double yaw =
        (out_Observations.sensor_std_yaw == 0)
            ? 0.0
            : getRandomGenerator().drawUniform(
                  -0.5 * out_Observations.fieldOfView_yaw, 0.5 * out_Observations.fieldOfView_yaw);
    const double pitch = (out_Observations.sensor_std_pitch == 0)
                             ? 0.0
                             : getRandomGenerator().drawUniform(
                                   -0.5 * out_Observations.fieldOfView_pitch,
                                   0.5 * out_Observations.fieldOfView_pitch);

    mrpt::obs::CObservationBearingRange::TMeasurement newMeas;
    newMeas.landmarkID = mrpt::obs::INVALID_LANDMARK_ID;
    newMeas.range = static_cast<float>(range);
    newMeas.yaw = static_cast<float>(yaw);
    newMeas.pitch = static_cast<float>(pitch);

    out_Observations.sensedData.push_back(newMeas);
    if (out_real_associations) out_real_associations->push_back(std::string::npos);
  }
}
