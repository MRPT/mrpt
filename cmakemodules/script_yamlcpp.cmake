# Check for yaml-cpp libray.
# It has a *-config.cmake files, but they are not present in Ubuntu 12.04 LTS, 
# so we use pkg-config there.
# ===================================================
set(CMAKE_MRPT_HAS_YAMLCPP 0)
set(CMAKE_MRPT_HAS_YAMLCPP_SYSTEM 0)

# DISABLE_YAMLCPP
# ---------------------
option(DISABLE_YAMLCPP "Force not using yamlcpp" "OFF")
mark_as_advanced(DISABLE_YAMLCPP)
if(NOT DISABLE_YAMLCPP)
	# Invoke pkg-config for getting the configuration:
	if(PKG_CONFIG_FOUND)
		PKG_CHECK_MODULES(LIBYAMLCPP QUIET yaml-cpp)
		if (LIBYAMLCPP_FOUND)
			set(CMAKE_MRPT_HAS_YAMLCPP 1)
			set(CMAKE_MRPT_HAS_YAMLCPP_SYSTEM 1)

			include_directories(${LIBYAMLCPP_INCLUDE_DIRS})
			if($ENV{VERBOSE})
				message(STATUS "LIBYAMLCPP_LIBRARIES    : ${LIBYAMLCPP_LIBRARIES}")
				message(STATUS "LIBYAMLCPP_INCLUDE_DIRS : ${LIBYAMLCPP_INCLUDE_DIRS}")
				message(STATUS "LIBYAMLCPP_VERSION      : ${LIBYAMLCPP_VERSION}")
			endif()
		endif()
	endif()

endif()

