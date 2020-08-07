/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdint>
#include <string>

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** @name Maps, Occupancy Grid Maps: ROS <-> MRPT
 *  @{ */

/** Methods to convert between ROS msgs and MRPT objects for map datatypes.
 * @brief the map class is implemented as singeleton use map::instance
 * ()->fromROS ...
 */
class MapHdl
{
   private:
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	int lut_cellmrpt2ros[0x100];  // lookup table for entry convertion
#else
	int lut_cellmrpt2ros[0xFFFF];  // lookup table for entry convertion
#endif
	int lut_cellros2mrpt[101];  // lookup table for entry convertion

	MapHdl();
	MapHdl(const MapHdl&);
	~MapHdl() = default;

   public:
	/**
	 * @return returns singeleton instance
	 * @brief it creates a instance with some look up table to speed up the
	 * conversions
	 */
	static MapHdl* instance();

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	int8_t cellMrpt2Ros(int8_t i)
	{
		return lut_cellmrpt2ros[static_cast<int>(i) - INT8_MIN];
	}
#else
	int16_t cellMrpt2Ros(int16_t i)
	{
		return lut_cellmrpt2ros[static_cast<int>(i) - INT16_MIN];
	}
#endif
	int8_t cellRos2Mrpt(int8_t i)
	{
		if (i < 0)
		{
			// unobserved cells: no log-odds information
			return 0;
		}
		ASSERT_LE_(i, 100);
		return lut_cellros2mrpt[i];
	}

	/**
	 * loads a mprt map
	 * @return true on sucess.
	 * @param _metric_map
	 * @param _config_file
	 * @param _map_file  default: map.simplemap
	 * @param _section_name default: metricMap
	 * @param _debug default: false
	 */
	static bool loadMap(
		mrpt::maps::CMultiMetricMap& _metric_map,
		const mrpt::config::CConfigFileBase& _config_file,
		const std::string& _map_file = "map.simplemap",
		const std::string& _section_name = "metricMap", bool _debug = false);
};

/**
 * converts ros msg to mrpt object
 * @return true on sucessful conversion, false on any error.
 * @param src
 * @param des
 */
bool fromROS(
	const nav_msgs::OccupancyGrid& src, mrpt::maps::COccupancyGridMap2D& des);

/**
 * converts mrpt object to ros msg and updates the msg header
 * @return true on sucessful conversion, false on any error.
 * @param src
 * @param header
 */
bool toROS(
	const mrpt::maps::COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& msg,
	const std_msgs::Header& header);
/**
 * converts mrpt object to ros msg
 * @return true on sucessful conversion, false on any error.
 */
bool toROS(
	const mrpt::maps::COccupancyGridMap2D& src, nav_msgs::OccupancyGrid& msg);

/** @}
 * @}
 */

}  // namespace mrpt::ros1bridge
