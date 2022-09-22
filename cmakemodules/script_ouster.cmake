# Support for OUSTER lidars:
# ===================================================

find_package(jsoncpp QUIET)
find_package(CURL QUIET)

if (jsoncpp_FOUND AND CURL_FOUND)
	set(def_ ON)
else()
	set(def_ OFF)
endif()

set(MRPT_WITH_OUSTER_LIDAR ${def_} CACHE BOOL "Build support for OUSTER Lidar sensors")

if (MRPT_WITH_OUSTER_LIDAR AND 
	((NOT jsoncpp_FOUND) OR (NOT CURL_FOUND)))
	message(ERROR "Building OUSTER requires jsoncpp and CURL libraries. Install them or disable MRPT_WITH_OUSTER_LIDAR")
endif()

# Create config vars:
set(CMAKE_MRPT_HAS_OUSTER 0)
if(MRPT_WITH_OUSTER_LIDAR)
	set(CMAKE_MRPT_HAS_OUSTER 1)
endif()
