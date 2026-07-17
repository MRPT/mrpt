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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <test_mrpt_common.h>

#include <filesystem>

TEST(CMultiMetricMapTests, isEmpty)
{
  {
    mrpt::maps::CMultiMetricMap m;
    EXPECT_TRUE(m.isEmpty());
  }
}

namespace
{
mrpt::maps::CMultiMetricMap initializer1()
{
  mrpt::maps::TSetOfMetricMapInitializers map_inits;
  {
    {
      mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
      def.resolution = 0.05f;
      def.insertionOpts.maxOccupancyUpdateCertainty = 0.8f;
      def.insertionOpts.maxDistanceInsertion = 30;
      map_inits.push_back(def);
    }
    {
      mrpt::maps::CSimplePointsMap::TMapDefinition def;
      map_inits.push_back(def);
    }
    mrpt::maps::CMultiMetricMap m;
    m.setListOfMaps(map_inits);
    return m;
  }
}

mrpt::maps::CMultiMetricMap initializer2()
{
  mrpt::maps::CMultiMetricMap m;
  m.maps.push_back(mrpt::maps::COccupancyGridMap2D::Create());
  m.maps.push_back(mrpt::maps::CSimplePointsMap::Create());
  return m;
}
}  // namespace

TEST(CMultiMetricMapTests, initializers)
{
  {
    const auto m = initializer1();
    EXPECT_EQ(m.maps.size(), 2U);
    EXPECT_TRUE(IS_CLASS(*m.maps.at(0), mrpt::maps::COccupancyGridMap2D));
    EXPECT_TRUE(IS_CLASS(*m.maps.at(1), mrpt::maps::CSimplePointsMap));
  }
  {
    const auto m = initializer2();
    EXPECT_EQ(m.maps.size(), 2U);
    EXPECT_TRUE(IS_CLASS(*m.maps.at(0), mrpt::maps::COccupancyGridMap2D));
    EXPECT_TRUE(IS_CLASS(*m.maps.at(1), mrpt::maps::CSimplePointsMap));
  }
}

TEST(CMultiMetricMapTests, copyCtorOp)
{
  using mrpt::maps::CSimplePointsMap;

  auto m1 = initializer1();
  EXPECT_EQ(m1.maps.size(), 2U);

  m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

  // Test deep copy:
  {
    mrpt::maps::CMultiMetricMap m2 = m1;

    EXPECT_EQ(m1.mapByClass<CSimplePointsMap>()->size(), 1U);
    EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);

    m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

    EXPECT_EQ(m1.mapByClass<CSimplePointsMap>()->size(), 2U);
    EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);
  }
}

TEST(CMultiMetricMapTests, moveOp)
{
  using mrpt::maps::CSimplePointsMap;

  auto m1 = initializer1();
  EXPECT_EQ(m1.maps.size(), 2U);

  m1.mapByClass<CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

  mrpt::maps::CMultiMetricMap m2 = std::move(m1);

  EXPECT_EQ(m2.mapByClass<CSimplePointsMap>()->size(), 1U);
}

TEST(CMultiMetricMapTests, unknownMapType)
{
  {
    const mrpt::config::CConfigFileMemory cfg(R""""(
[map]
// Creation of maps:
occupancyGrid_count=1

[map_occupancyGrid_00_creationOpts]
min_x=-10
max_x= 10
min_y=-10
max_y= 10
)"""");

    mrpt::maps::TSetOfMetricMapInitializers map_inits;
    EXPECT_NO_THROW(map_inits.loadFromConfigFile(cfg, "map"));
    EXPECT_EQ(map_inits.size(), 1UL);
  }

  {
    const mrpt::config::CConfigFileMemory cfg(R""""(
[map]
// Creation of maps:
occupancyGrid_count=1
myUnregisteredMap_count=1

[map_occupancyGrid_00_creationOpts]
min_x=-10
max_x= 10
min_y=-10
max_y= 10
)"""");

    mrpt::maps::TSetOfMetricMapInitializers map_inits;
    EXPECT_ANY_THROW(map_inits.loadFromConfigFile(cfg, "map"));
  }
}

// =========================================================================
//  determineMatching2D(): requires exactly one CSimplePointsMap sub-map
// =========================================================================

TEST(CMultiMetricMapTests, DetermineMatching2DWithOnePointsMapWorks)
{
  auto m = initializer1();  // Has exactly 1 CSimplePointsMap sub-map.
  m.mapByClass<mrpt::maps::CSimplePointsMap>()->insertPoint(1.0f, 0.0f, 0.0f);

  mrpt::maps::CSimplePointsMap otherMap;
  otherMap.insertPoint(1.0f, 0.0f, 0.0f);

  mrpt::maps::TMatchingParams params;
  params.maxDistForCorrespondence = 0.5f;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  EXPECT_NO_THROW(m.determineMatching2D(
      &otherMap, mrpt::poses::CPose2D(0, 0, 0), correspondences, params, extraResults));
  EXPECT_FALSE(correspondences.empty());
}

TEST(CMultiMetricMapTests, DetermineMatching2DWithZeroPointsMapsThrows)
{
  mrpt::maps::CMultiMetricMap m;
  m.maps.push_back(mrpt::maps::COccupancyGridMap2D::Create());  // No points map.

  mrpt::maps::CSimplePointsMap otherMap;
  otherMap.insertPoint(1.0f, 0.0f, 0.0f);

  mrpt::maps::TMatchingParams params;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  EXPECT_ANY_THROW(m.determineMatching2D(
      &otherMap, mrpt::poses::CPose2D(0, 0, 0), correspondences, params, extraResults));
}

// =========================================================================
//  getAsSimplePointsMap(): 0, 1, and >1 points-map sub-map cases
// =========================================================================

TEST(CMultiMetricMapTests, GetAsSimplePointsMapZeroReturnsNull)
{
  mrpt::maps::CMultiMetricMap m;
  m.maps.push_back(mrpt::maps::COccupancyGridMap2D::Create());
  EXPECT_EQ(m.getAsSimplePointsMap(), nullptr);
}

TEST(CMultiMetricMapTests, GetAsSimplePointsMapOneReturnsIt)
{
  auto m = initializer1();
  EXPECT_NE(m.getAsSimplePointsMap(), nullptr);
}

TEST(CMultiMetricMapTests, GetAsSimplePointsMapMultipleThrows)
{
  mrpt::maps::CMultiMetricMap m;
  m.maps.push_back(mrpt::maps::CSimplePointsMap::Create());
  m.maps.push_back(mrpt::maps::CSimplePointsMap::Create());
  EXPECT_ANY_THROW(m.getAsSimplePointsMap());
}

// =========================================================================
//  asString() / mapByIndex()
// =========================================================================

TEST(CMultiMetricMapTests, AsStringListsChildMaps)
{
  auto m = initializer1();
  const auto s = m.asString();
  EXPECT_NE(s.find("children maps"), std::string::npos);
}

TEST(CMultiMetricMapTests, MapByIndexConstAndNonConst)
{
  auto m = initializer1();
  EXPECT_NE(m.mapByIndex(0), nullptr);

  const auto& mc = m;
  EXPECT_NE(mc.mapByIndex(0), nullptr);

  EXPECT_THROW(m.mapByIndex(99), std::exception);
}

// =========================================================================
//  saveMetricMapRepresentationToFile() / getVisualizationInto() /
//  auxParticleFilterCleanUp()
// =========================================================================

TEST(CMultiMetricMapTests, SaveMetricMapRepresentationToFile)
{
  auto m = initializer1();
  m.mapByClass<mrpt::maps::CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  const std::string prefix =
      (dir / ("mrpt_CMultiMetricMap_unittest_" + std::to_string(static_cast<long>(getpid())) + "_" +
              std::to_string(counter++)))
          .string();

  EXPECT_NO_THROW(m.saveMetricMapRepresentationToFile(prefix));

  // Clean up whatever per-submap files were generated (names include the
  // submap class name and index, see CMultiMetricMap.cpp):
  for (const auto& entry : std::filesystem::directory_iterator(dir))
  {
    const auto fn = entry.path().filename().string();
    if (fn.rfind(
            "mrpt_CMultiMetricMap_unittest_" + std::to_string(static_cast<long>(getpid())) + "_" +
                std::to_string(counter - 1),
            0) == 0)
      std::filesystem::remove(entry.path());
  }
}

TEST(CMultiMetricMapTests, GetVisualizationInto)
{
  auto m = initializer1();
  m.mapByClass<mrpt::maps::CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

  auto scene = mrpt::viz::CSetOfObjects::Create();
  EXPECT_NO_THROW(m.getVisualizationInto(*scene));
}

TEST(CMultiMetricMapTests, AuxParticleFilterCleanUp)
{
  auto m = initializer1();
  EXPECT_NO_THROW(m.auxParticleFilterCleanUp());
}

// =========================================================================
//  internal_computeObservationLikelihood / canComputeObservationLikelihood
// =========================================================================

TEST(CMultiMetricMapTests, ComputeObservationLikelihoodFinite)
{
  auto m = initializer1();

  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);
  m.insertObservation(scan1);

  EXPECT_TRUE(m.canComputeObservationLikelihood(scan1));
  const double lik = m.computeObservationLikelihood(scan1, mrpt::poses::CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(lik));
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CMultiMetricMapTests, SerializeRoundTrip)
{
  auto m1 = initializer1();
  m1.mapByClass<mrpt::maps::CSimplePointsMap>()->insertPoint(1.0f, 2.0f, 3.0f);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << m1;
  }
  buf.Seek(0);

  mrpt::maps::CMultiMetricMap m2;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar.ReadObject(&m2);
  }

  EXPECT_EQ(m2.maps.size(), m1.maps.size());
  EXPECT_EQ(m2.mapByClass<mrpt::maps::CSimplePointsMap>()->size(), 1u);
}

TEST(CMultiMetricMapTests, Compute3DMatchingRatio)
{
  auto m = initializer1();
  m.mapByClass<mrpt::maps::CSimplePointsMap>()->insertPoint(1.0f, 0.0f, 0.0f);

  mrpt::maps::CSimplePointsMap otherMap;
  otherMap.insertPoint(1.0f, 0.0f, 0.0f);

  mrpt::maps::TMatchingRatioParams params;
  const float ratio = m.compute3DMatchingRatio(&otherMap, mrpt::poses::CPose3D::Identity(), params);
  EXPECT_TRUE(std::isfinite(ratio));
}
