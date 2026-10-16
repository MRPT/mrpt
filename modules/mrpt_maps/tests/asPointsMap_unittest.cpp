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

#include <gtest/gtest.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>

// A map that is not a points map, but exposes its contents as one, as
// keyframe or voxel maps in other libraries do.
// It must live directly in namespace mrpt::maps: the serialization macros
// qualify symbols with that literal namespace.
namespace mrpt::maps
{
class TestPointsViewMap : public CMetricMap
{
  DEFINE_SERIALIZABLE(TestPointsViewMap, mrpt::maps)

 public:
  CSimplePointsMap points;

  void internal_clear() override { points.clear(); }
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
  bool isEmpty() const override { return points.empty(); }
  void saveMetricMapRepresentationToFile(const std::string&) const override {}
  std::string asString() const override { return "TestPointsViewMap"; }
  void getVisualizationInto(mrpt::viz::CSetOfObjects&) const override {}

  const CPointsMap* asPointsMapView() const override { return &points; }
};

IMPLEMENTS_SERIALIZABLE(TestPointsViewMap, CMetricMap, mrpt::maps)
uint8_t TestPointsViewMap::serializeGetVersion() const { return 0; }
void TestPointsViewMap::serializeTo(mrpt::serialization::CArchive&) const {}
void TestPointsViewMap::serializeFrom(mrpt::serialization::CArchive&, uint8_t) {}
}  // namespace mrpt::maps

using mrpt::maps::asPointsMap;

TEST(asPointsMap, PointsMapIsItself)
{
  mrpt::maps::CSimplePointsMap m;
  EXPECT_EQ(asPointsMap(m), &m);
}

TEST(asPointsMap, NoPointsRepresentation)
{
  const mrpt::maps::COccupancyGridMap2D grid;
  EXPECT_EQ(asPointsMap(grid), nullptr);
}

TEST(asPointsMap, MapWithPointsView)
{
  mrpt::maps::TestPointsViewMap m;
  m.points.insertPoint(1.0f, 2.0f, 3.0f);
  m.points.insertPoint(4.0f, 5.0f, 6.0f);

  const auto* pts = asPointsMap(m);
  ASSERT_EQ(pts, &m.points);
  EXPECT_EQ(pts->size(), 2U);
}

// Matching against such a map works through its points view:
TEST(asPointsMap, MatchingAgainstPointsView)
{
  mrpt::maps::TestPointsViewMap other;
  mrpt::maps::CSimplePointsMap local;
  for (int i = 0; i < 10; i++)
  {
    const auto x = static_cast<float>(i);
    other.points.insertPoint(x, 0.0f, 0.0f);
    local.insertPoint(x, 0.0f, 0.0f);
  }

  mrpt::maps::TMatchingRatioParams params;
  params.maxDistForCorr = 0.1f;
  EXPECT_FLOAT_EQ(
      local.compute3DMatchingRatio(&other, mrpt::poses::CPose3D::Identity(), params), 1.0f);
}
