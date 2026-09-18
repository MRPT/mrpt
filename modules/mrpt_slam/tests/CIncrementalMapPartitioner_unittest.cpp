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
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#include <algorithm>

TEST(CIncrementalMapPartitioner, test_dataset)
{
  mrpt::slam::CIncrementalMapPartitioner imp;

  // Options:
  imp.options.partitionThreshold = 0.5;
  imp.options.mrp.maxDistForCorr = 0.2f;
  imp.options.mrp.maxMahaDistForCorr = 10.0f;
  imp.options.simil_method = mrpt::slam::smMETRIC_MAP_MATCHING;

  const std::string map_file =
      mrpt::mrpt_data_dir() + std::string("/datasets/malaga-cs-fac-building.simplemap.gz");
  ASSERT_FILE_EXISTS_(map_file);

  mrpt::maps::CSimpleMap in_map, out_map;
  std::ignore = in_map.loadFromFile(map_file);

  for (const auto& [posePDF, sf, twist] : in_map) imp.addMapFrame(*sf, *posePDF);

  std::vector<std::vector<uint32_t>> parts;
  imp.updatePartitions(parts);

  EXPECT_EQ(parts.size(), 2UL);
  if (parts.size() != 2UL)
  {
    return;
  }
  // The two clusters are keyframes [0,26]+[62,98] and [27,61]. Which of them
  // the spectral partitioner returns first is not stable across platforms
  // (it depends on the sign of the eigenvector), so compare them in a
  // canonical order:
  std::vector<uint32_t> expected_a, expected_b;
  for (uint32_t i = 0; i <= 26; i++) expected_a.push_back(i);
  for (uint32_t i = 62; i <= 98; i++) expected_a.push_back(i);
  for (uint32_t i = 27; i <= 61; i++) expected_b.push_back(i);

  std::sort(
      parts.begin(), parts.end(),
      [](const auto& a, const auto& b) { return a.front() < b.front(); });

  EXPECT_EQ(parts[0], expected_a);
  EXPECT_EQ(parts[1], expected_b);
}
