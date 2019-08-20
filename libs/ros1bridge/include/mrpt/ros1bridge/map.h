/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
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
 * ()->ros2mrpt ...
 */
class MapHdl
{
   private:
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	int8_t lut_cellmrpt2ros[0xFF];  // lookup table for entry convertion
	int8_t* lut_cellmrpt2rosPtr;  // pointer to the center of the lookup table
// neede to work with neg. indexes
#else
	int8_t lut_cellmrpt2ros[0xFFFF];  // lookup table for entry convertion
	int8_t* lut_cellmrpt2rosPtr;  // pointer to the center of the lookup table
// neede to work with neg. indexes
#endif
	int8_t lut_cellros2mrpt[0xFF];  // lookup table for entry convertion
	int8_t* lut_cellros2mrptPtr;  // pointer to the center of the lookup table
	// neede to work with neg. indexes
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
	int8_t cellMrpt2Ros(int i) { return lut_cellmrpt2rosPtr[i]; }
#else
	int16_t cellMrpt2Ros(int i) { return lut_cellmrpt2rosPtr[i]; }
#endif
	int8_t cellRos2Mrpt(int i) { return lut_cellros2mrptPtr[i]; }
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
