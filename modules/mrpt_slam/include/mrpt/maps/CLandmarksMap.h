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

#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <deque>
#include <vector>

namespace mrpt::maps
{
/** A minimal landmarks map: a container of CLandmark objects.
 *
 * This is a simplified version for MRPT v3.0, retaining only the
 * functionality needed and removing the computer vision part.
 *
 * \ingroup mrpt_slam_grp
 */
class CLandmarksMap : public mrpt::maps::CMetricMap
{
  DEFINE_SERIALIZABLE(CLandmarksMap, mrpt::maps)

 public:
  using landmark_type = CLandmark;

  CLandmarksMap() = default;

  /** Inner container with helper accessors. */
  struct TLandmarksList
  {
    std::deque<CLandmark> m_landmarks;

    void push_back(const CLandmark& lm) { m_landmarks.push_back(lm); }

    [[nodiscard]] CLandmark* get(size_t idx)
    {
      ASSERT_(idx < m_landmarks.size());
      return &m_landmarks[idx];
    }
    [[nodiscard]] const CLandmark* get(size_t idx) const
    {
      ASSERT_(idx < m_landmarks.size());
      return &m_landmarks[idx];
    }

    /** Find by beacon ID (returns nullptr if not found). */
    [[nodiscard]] const CLandmark* getByBeaconID(int64_t beaconID) const
    {
      for (auto& lm : m_landmarks)
        if (lm.ID == beaconID) return &lm;
      return nullptr;
    }

    [[nodiscard]] size_t size() const { return m_landmarks.size(); }
    void clear() { m_landmarks.clear(); }

    // Range-for support:
    auto begin() { return m_landmarks.begin(); }
    auto end() { return m_landmarks.end(); }
    auto begin() const { return m_landmarks.begin(); }
    auto end() const { return m_landmarks.end(); }
  };

  TLandmarksList landmarks;

  /** Simulate a range-bearing sensor observation from this landmark map.
   * \param[in]  robotPose    Global pose of the robot.
   * \param[in]  sensorOnRobot  Pose of the sensor relative to the robot.
   * \param[out] obs          Output observation (sensedData will be filled).
   *                          obs.fieldOfView_yaw, maxSensorDistance,
   *                          minSensorDistance must be set by the caller.
   * \param[in]  sensorDistinguishesLandmarks  If true, each observation has the
   *                          actual landmark ID; if false, they are all set to -1.
   * \param[in]  sigmaRange   Noise standard deviation for range (meters).
   * \param[in]  sigmaYaw     Noise standard deviation for yaw (radians).
   * \param[in]  sigmaPitch   Noise standard deviation for pitch (radians, use 0
   *                          for 2D).
   * \param[out] out_indices  If non-null, stores the GT landmark index for each
   *                          observation; spurious detections get index
   *                          std::string::npos.
   * \param[in]  spuriousMean Mean number of spurious detections.
   * \param[in]  spuriousStd  Std of the number of spurious detections.
   */
  void simulateRangeBearingReadings(
      const mrpt::poses::CPose3D& robotPose,
      const mrpt::poses::CPose3D& sensorOnRobot,
      mrpt::obs::CObservationBearingRange& obs,
      bool sensorDistinguishesLandmarks = true,
      double sigmaRange = 0,
      double sigmaYaw = 0,
      double sigmaPitch = 0,
      std::vector<size_t>* out_indices = nullptr,
      double spuriousMean = 0,
      double spuriousStd = 0) const;

  // ---- CMetricMap interface (minimal) ----
  [[nodiscard]] size_t size() const { return landmarks.size(); }

  [[nodiscard]] bool isEmpty() const override { return landmarks.size() == 0; }

  [[nodiscard]] std::string asString() const override { return "CLandmarksMap"; }

  void getVisualizationInto(mrpt::viz::CSetOfObjects&) const override {}

  void saveMetricMapRepresentationToFile(const std::string&) const override {}

 protected:
  void internal_clear() override { landmarks.clear(); }

  bool internal_insertObservation(
      const mrpt::obs::CObservation&, const std::optional<const mrpt::poses::CPose3D>&) override
  {
    return false;
  }

  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation&, const mrpt::poses::CPose3D&) const override
  {
    return 0;
  }

  bool internal_canComputeObservationLikelihood(const mrpt::obs::CObservation&) const override
  {
    return false;
  }
};

}  // namespace mrpt::maps
