
set(CMAKE_MRPT_HAS_PCL 0)
set(CMAKE_MRPT_HAS_PCL_SYSTEM 0)

SET_PROPERTY(GLOBAL PROPERTY CMAKE_MRPT_HAS_PCL "${CMAKE_MRPT_HAS_PCL}")
SET_PROPERTY(GLOBAL PROPERTY CMAKE_MRPT_HAS_PCL_SYSTEM "${CMAKE_MRPT_HAS_PCL_SYSTEM}")

# Leave at the user's choice to disable the SWR libs:
option(DISABLE_PCL "Forces NOT using PCL, even if it could be found by CMake" "OFF")
mark_as_advanced(DISABLE_PCL)

if(NOT DISABLE_PCL)
	# Avoid warning about env var PCL_ROOT:
	if(POLICY CMP0074)
	  cmake_policy(SET CMP0074 NEW)
	endif()


	# PCL library:
	# --------------------------------------------
	find_package(PCL COMPONENTS io common registration visualization segmentation surface QUIET)
	if(PCL_FOUND AND PCL_IO_FOUND AND PCL_COMMON_FOUND AND PCL_REGISTRATION_FOUND AND PCL_VISUALIZATION_FOUND AND PCL_SURFACE_FOUND)

		set(CMAKE_MRPT_HAS_PCL 1)
		set(CMAKE_MRPT_HAS_PCL_SYSTEM 1)

		if (NOT Boost_FOUND)
			message(FATAL_ERROR "PCL requires Boost. Either disable PCL (with DISABLE_PCL=ON) or, to fix the error, create the entries BOOST_ROOT and BOOST_LIBRARYDIR and set them to the correct values")
		endif (NOT Boost_FOUND)

		# Filter empty strings in flags (lead to errors since they end up as `" "` in gcc flags)
		list(FILTER PCL_DEFINITIONS EXCLUDE REGEX "^[ ]+")

		if($ENV{VERBOSE})
			message(STATUS "PCL:")
			message(STATUS " Include dirs: ${PCL_INCLUDE_DIRS}")
			message(STATUS " Library dirs: ${PCL_LIBRARY_DIRS}")
			message(STATUS " Definitions : ${PCL_DEFINITIONS}")
			message(STATUS " Libraries   : ${PCL_LIBRARIES}")
		endif()

		add_library(imp_pcl INTERFACE IMPORTED GLOBAL)
		set_target_properties(imp_pcl
			PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
			INTERFACE_COMPILE_OPTIONS "${PCL_DEFINITIONS}"
			# We cannot add link libs here since they contain "optimized/debug" which are not permitted here.
			#INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
			)
	endif()
endif()

SET_PROPERTY(GLOBAL PROPERTY CMAKE_MRPT_HAS_PCL "${CMAKE_MRPT_HAS_PCL}")
SET_PROPERTY(GLOBAL PROPERTY CMAKE_MRPT_HAS_PCL_SYSTEM "${CMAKE_MRPT_HAS_PCL_SYSTEM}")
SET_PROPERTY(GLOBAL PROPERTY PCL_LIBRARIES "${PCL_LIBRARIES}")
