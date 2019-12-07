/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

TEST(CIncrementalMapPartitioner, test_dataset)
{
	mrpt::slam::CIncrementalMapPartitioner imp;

	// Options:
	imp.options.partitionThreshold = 0.5;
	imp.options.mrp.maxDistForCorr = 0.2f;
	imp.options.mrp.maxMahaDistForCorr = 10.0f;
	imp.options.simil_method = mrpt::slam::smMETRIC_MAP_MATCHING;

	const std::string map_file =
		mrpt::UNITTEST_BASEDIR +
		std::string("/share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz");
	ASSERT_FILE_EXISTS_(map_file);

	mrpt::maps::CSimpleMap in_map, out_map;
	in_map.loadFromFile(map_file);

	for (const auto& pair : in_map)
	{
		const auto& [posePDF, sf] = pair;
		imp.addMapFrame(*sf, *posePDF);
	}

	std::vector<std::vector<uint32_t>> parts;
	imp.updatePartitions(parts);

	EXPECT_EQ(parts.size(), 2UL);
	if (parts.size() != 2UL) return;

	const std::vector<uint32_t>
		expected_p0 = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,
					   13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
					   26, 27, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72,
					   73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85,
					   86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98},
		expected_p1 = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
					   40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
					   52, 53, 54, 55, 56, 57, 58, 59, 60, 61};

	EXPECT_EQ(parts[0], expected_p0);
	EXPECT_EQ(parts[1], expected_p1);
}
