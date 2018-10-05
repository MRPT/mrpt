# Occupancy grid cellsize in bits: 8 or 16
# ===================================================
set(MRPT_OCCUPANCY_GRID_CELLSIZE 8 CACHE STRING "Cell size for mrpt::maps::COccupancyGridMap2D (either 8 or 16 bits)")
if(NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 8 AND NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 16)
	message("MRPT_OCCUPANCY_GRID_CELLSIZE can have the values 8 or 16 only.")
endif(NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 8 AND NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 16)
