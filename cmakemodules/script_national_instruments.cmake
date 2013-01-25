# National Instruments C API 
# -----------------------------------------
# "NIEXTCCOMPILERSUPP" should have the path to the NI headers & libs:
SET(NI_ROOT_DIR $ENV{NIEXTCCOMPILERSUPP} CACHE PATH "National-Instruments C-support directory (as read from env.var NIEXTCCOMPILERSUPP)")
MARK_AS_ADVANCED(NI_ROOT_DIR)

SET(NI_AUTODETECTED "OFF")
IF(NOT "" STREQUAL ${NI_ROOT_DIR})
	SET(NI_AUTODETECTED "ON")
ENDIF(NOT "" STREQUAL ${NI_ROOT_DIR})

SET(MRPT_HAS_NATIONAL_INSTRUMENTS ${NI_AUTODETECTED} CACHE BOOL "Build hwdrivers interfaces to NI devices")

SET(CMAKE_MRPT_HAS_NATIONAL_INSTRUMENTS 0)
IF (MRPT_HAS_NATIONAL_INSTRUMENTS)
	MARK_AS_ADVANCED(CLEAR NI_ROOT_DIR)

	# Check minimum directories:
	IF (EXISTS "${NI_ROOT_DIR}/include/")
		# OK, we have NI:
		SET(CMAKE_MRPT_HAS_NATIONAL_INSTRUMENTS 1)
		MARK_AS_ADVANCED(CLEAR NI_ROOT_DIR)

		INCLUDE_DIRECTORIES("${NI_ROOT_DIR}/include/")
		link_directories("${NI_ROOT_DIR}/lib${CMAKE_MRPT_WORD_SIZE}/msvc/")

		# Now check for specific hardware headers:	
		SET(CMAKE_MRPT_HAS_NI845x 0)
		IF (EXISTS "${NI_ROOT_DIR}/include/ni845x.h")
			SET(CMAKE_MRPT_HAS_NI845x 1)
		ENDIF (EXISTS "${NI_ROOT_DIR}/include/ni845x.h")
		
		
	ELSE(EXISTS "${NI_ROOT_DIR}/include/")
		# Error:
		MESSAGE("Error: Include path not found (correct NI_ROOT_DIR or disable MRPT_HAS_NATIONAL_INSTRUMENTS): '${NI_ROOT_DIR}/include/'")
	ENDIF(EXISTS "${NI_ROOT_DIR}/include/")

	

	

ENDIF (MRPT_HAS_NATIONAL_INSTRUMENTS)

