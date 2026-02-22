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

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <deque>

namespace mrpt::maps
{
/** A minimal landmarks map: a container of CLandmark objects.
 *
 * This is a simplified version for MRPT v3.0, retaining only the
 * functionality needed by CGridMapAligner, COccupancyGridMapFeatureExtractor,
 * and related classes.
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
  };

  TLandmarksList landmarks;

  // ---- CMetricMap interface (minimal) ----
  [[nodiscard]] size_t size() const { return landmarks.size(); }

  [[nodiscard]] bool isEmpty() const override { return landmarks.size() == 0; }

  [[nodiscard]] std::string asString() const override { return "CLandmarksMap"; }

  void getVisualizationInto(mrpt::viz::CSetOfObjects&) const override {}

  void saveMetricMapRepresentationToFile(const std::string&) const override {}

 protected:
  void internal_clear() override { landmarks.clear(); }

  bool internal_insertObservation(
      const mrpt::obs::CObservation&,
      const std::optional<const mrpt::poses::CPose3D>&) override
  {
    return false;
  }

  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation&,
      const mrpt::poses::CPose3D&) const override
  {
    return 0;
  }

  bool internal_canComputeObservationLikelihood(
      const mrpt::obs::CObservation&) const override
  {
    return false;
  }
};

}  // namespace mrpt::maps
