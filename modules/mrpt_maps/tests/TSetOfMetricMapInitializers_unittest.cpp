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

/** Unit tests for mrpt::maps::TSetOfMetricMapInitializers config-file I/O:
 *  what saveToConfigFile() writes must be readable back by
 *  loadFromConfigFile().
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>

#include <sstream>
#include <vector>

using mrpt::maps::TSetOfMetricMapInitializers;

TEST(TSetOfMetricMapInitializers, saveLoadRoundTrip)
{
  TSetOfMetricMapInitializers mapsIni;
  {
    mrpt::maps::CSimplePointsMap::TMapDefinition def;
    def.genericMapParams.enableObservationLikelihood = false;
    mapsIni.push_back(def);
  }
  {
    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
    def.genericMapParams.enableSaveAs3DObject = false;
    mapsIni.push_back(def);
  }
  {
    mrpt::maps::CSimplePointsMap::TMapDefinition def;
    mapsIni.push_back(def);
  }
  ASSERT_EQ(mapsIni.size(), 3U);

  mrpt::config::CConfigFileMemory cfg;
  mapsIni.saveToConfigFile(cfg, "MAPS");

  TSetOfMetricMapInitializers restored;
  restored.loadFromConfigFile(cfg, "MAPS");

  ASSERT_EQ(restored.size(), 3U);

  std::vector<mrpt::maps::TMetricMapInitializer::Ptr> got;
  for (const auto& mi : restored) got.push_back(mi);

  // Maps are grouped per class by the loader, hence the ordering below:
  EXPECT_EQ(
      std::string(got[0]->getMetricMapClassType()->className), "mrpt::maps::COccupancyGridMap2D");
  EXPECT_EQ(
      std::string(got[1]->getMetricMapClassType()->className), "mrpt::maps::CSimplePointsMap");
  EXPECT_EQ(
      std::string(got[2]->getMetricMapClassType()->className), "mrpt::maps::CSimplePointsMap");

  EXPECT_FALSE(got[0]->genericMapParams.enableSaveAs3DObject);
  // Only the generic params round trip: there is no saving counterpart to
  // loadFromConfigFile_map_specific(), so map-specific options stay at their
  // defaults.
  EXPECT_FALSE(got[1]->genericMapParams.enableObservationLikelihood);
  EXPECT_TRUE(got[2]->genericMapParams.enableObservationLikelihood);
}

TEST(TSetOfMetricMapInitializers, emptySetSavesNothing)
{
  TSetOfMetricMapInitializers mapsIni;

  mrpt::config::CConfigFileMemory cfg;
  mapsIni.saveToConfigFile(cfg, "MAPS");

  TSetOfMetricMapInitializers restored;
  restored.loadFromConfigFile(cfg, "MAPS");
  EXPECT_EQ(restored.size(), 0U);
}

TEST(TSetOfMetricMapInitializers, unknownMapClassThrows)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("MAPS", "thisIsNotAMapClass_count", 1);

  TSetOfMetricMapInitializers mapsIni;
  EXPECT_THROW(mapsIni.loadFromConfigFile(cfg, "MAPS"), std::exception);
}

TEST(TSetOfMetricMapInitializers, dumpToTextStream)
{
  TSetOfMetricMapInitializers mapsIni;
  mrpt::maps::CSimplePointsMap::TMapDefinition def;
  mapsIni.push_back(def);

  std::stringstream ss;
  mapsIni.dumpToTextStream(ss);
  EXPECT_NE(ss.str().find("CSimplePointsMap"), std::string::npos);
}
