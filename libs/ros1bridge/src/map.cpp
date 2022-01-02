/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>	 // for fileExists()
#include <mrpt/system/string_utils.h>  // for lowerCase()
#include <mrpt/version.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

using namespace mrpt::config;
using namespace mrpt::io;
using mrpt::maps::CLogOddsGridMapLUT;
using mrpt::maps::CMultiMetricMap;
using mrpt::maps::COccupancyGridMap2D;
using mrpt::maps::CSimpleMap;

#ifndef INT8_MAX  // avoid duplicated #define's
#define INT8_MAX 0x7f
#define INT8_MIN (-INT8_MAX - 1)
#define INT16_MAX 0x7fff
#define INT16_MIN (-INT16_MAX - 1)
#endif	// INT8_MAX

namespace mrpt::ros1bridge
{
MapHdl::MapHdl()
{
	// MRPT -> ROS LUT:
	CLogOddsGridMapLUT<COccupancyGridMap2D::cellType> table;

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	const int i_min = INT8_MIN, i_max = INT8_MAX;
#else
	const int i_min = INT16_MIN, i_max = INT16_MAX;
#endif

	for (int i = i_min; i <= i_max; i++)
	{
		int8_t ros_val;
		if (i == 0)
		{
			// Unknown cell (no evidence data):
			ros_val = -1;
		}
		else
		{
			float p = 1.0 - table.l2p(i);
			ros_val = round(p * 100.);
			// printf("- cell -> ros = %4i -> %4i, p=%4.3f\n", i, ros_val, p);
		}

		lut_cellmrpt2ros[static_cast<int>(i) - i_min] = ros_val;
	}

	// ROS -> MRPT: [0,100] ->
	for (int i = 0; i <= 100; i++)
	{
		const float p = 1.0 - (i / 100.0);
		lut_cellros2mrpt[i] = table.p2l(p);

		// printf("- ros->cell=%4i->%4i p=%4.3f\n",i, lut_cellros2mrpt[i], p);
	}
}
MapHdl* MapHdl::instance()
{
	static MapHdl m;  // singeleton instance
	return &m;
}

bool fromROS(const nav_msgs::OccupancyGrid& src, COccupancyGridMap2D& des)
{
	MRPT_START
	if ((src.info.origin.orientation.x != 0) ||
		(src.info.origin.orientation.y != 0) ||
		(src.info.origin.orientation.z != 0) ||
		(src.info.origin.orientation.w != 1))
	{
		std::cerr << "[fromROS] Rotated maps are not supported!\n";
		return false;
	}
	float xmin = src.info.origin.position.x;
	float ymin = src.info.origin.position.y;
	float xmax = xmin + src.info.width * src.info.resolution;
	float ymax = ymin + src.info.height * src.info.resolution;

	des.setSize(xmin, xmax, ymin, ymax, src.info.resolution);
	auto inst = MapHdl::instance();
	for (unsigned int h = 0; h < src.info.height; h++)
	{
		COccupancyGridMap2D::cellType* pDes = des.getRow(h);
		const int8_t* pSrc = &src.data[h * src.info.width];
		for (unsigned int w = 0; w < src.info.width; w++)
			*pDes++ = inst->cellRos2Mrpt(*pSrc++);
	}
	return true;
	MRPT_END
}
bool toROS(
	const COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& des,
	const std_msgs::Header& header)
{
	des.header = header;
	return toROS(src, des);
}

bool toROS(const COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& des)
{
	des.info.width = src.getSizeX();
	des.info.height = src.getSizeY();
	des.info.resolution = src.getResolution();

	des.info.origin.position.x = src.getXMin();
	des.info.origin.position.y = src.getYMin();
	des.info.origin.position.z = 0;

	des.info.origin.orientation.x = 0;
	des.info.origin.orientation.y = 0;
	des.info.origin.orientation.z = 0;
	des.info.origin.orientation.w = 1;

	// I hope the data is always aligned
	des.data.resize(des.info.width * des.info.height);
	for (unsigned int h = 0; h < des.info.height; h++)
	{
		const COccupancyGridMap2D::cellType* pSrc = src.getRow(h);
		int8_t* pDes = &des.data[h * des.info.width];
		for (unsigned int w = 0; w < des.info.width; w++)
		{
			*pDes++ = MapHdl::instance()->cellMrpt2Ros(*pSrc++);
		}
	}
	return true;
}

bool MapHdl::loadMap(
	CMultiMetricMap& _metric_map, const CConfigFileBase& _config_file,
	const std::string& _map_file, const std::string& _section_name, bool _debug)
{
	using namespace mrpt::maps;

	TSetOfMetricMapInitializers mapInitializers;
	mapInitializers.loadFromConfigFile(_config_file, _section_name);

	CSimpleMap simpleMap;

	// Load the set of metric maps to consider in the experiments:
	_metric_map.setListOfMaps(mapInitializers);
	if (_debug) mapInitializers.dumpToConsole();

	if (_debug)
		printf(
			"%s, _map_file.size() = %zu\n", _map_file.c_str(),
			_map_file.size());
	// Load the map (if any):
	if (_map_file.size() < 3)
	{
		if (_debug) printf("No mrpt map file!\n");
		return false;
	}
	else
	{
		ASSERT_(mrpt::system::fileExists(_map_file));

		// Detect file extension:
		std::string mapExt =
			mrpt::system::lowerCase(mrpt::system::extractFileExtension(
				_map_file, true));	// Ignore possible .gz extensions

		if (!mapExt.compare("simplemap"))
		{
			// It's a ".simplemap":
			if (_debug) printf("Loading '.simplemap' file...");
			CFileGZInputStream f(_map_file);
			mrpt::serialization::archiveFrom(f) >> simpleMap;
			printf("Ok\n");

			ASSERTMSG_(
				simpleMap.size() > 0,
				"Simplemap was aparently loaded OK, but it is empty!");

			// Build metric map:
			if (_debug) printf("Building metric map(s) from '.simplemap'...");
			_metric_map.loadFromSimpleMap(simpleMap);
			if (_debug) printf("Ok\n");
		}
		else if (!mapExt.compare("gridmap"))
		{
			// It's a ".gridmap":
			if (_debug) printf("Loading gridmap from '.gridmap'...");
			ASSERTMSG_(
				_metric_map.countMapsByClass<COccupancyGridMap2D>() == 1,
				"Error: Trying to load a gridmap into a multi-metric map "
				"requires 1 gridmap member.");
			CFileGZInputStream fm(_map_file);
			mrpt::serialization::archiveFrom(fm) >>
				(*_metric_map.mapByClass<COccupancyGridMap2D>());
			if (_debug) printf("Ok\n");
		}
		else
		{
			THROW_EXCEPTION(mrpt::format(
				"Map file has unknown extension: '%s'", mapExt.c_str()));
			return false;
		}
	}
	return true;
}

}  // namespace mrpt::ros1bridge
