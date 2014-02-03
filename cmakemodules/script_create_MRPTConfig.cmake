# ----------------------------------------------------------------------------
#   Generate the MRPTConfig.cmake & configure files
# ----------------------------------------------------------------------------
# Create the code fragment: "DECLARE_LIBS_DEPS", for usage below while
#  generating "MRPTConfig.cmake"
SET(DECLARE_LIBS_DEPS "")
FOREACH(_LIB ${ALL_MRPT_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	SET(DECLARE_LIBS_DEPS "${DECLARE_LIBS_DEPS} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_DEPS\" \"${_LIB_DEP}\")\n")
ENDFOREACH(_LIB)

# Create the code fragment: "DECLARE_LIBS_HDR_ONLY", for usage below while
#  generating "MRPTConfig.cmake"
SET(DECLARE_LIBS_HDR_ONLY "")
FOREACH(_LIB ${ALL_MRPT_LIBS})
	get_property(_LIB_HDR_ONLY GLOBAL PROPERTY "${_LIB}_LIB_IS_HEADERS_ONLY")
	SET(DECLARE_LIBS_HDR_ONLY "${DECLARE_LIBS_HDR_ONLY} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_IS_HEADERS_ONLY\" \"${_LIB_HDR_ONLY}\")\n")
ENDFOREACH(_LIB)

# ----------------------------------------------------------------------------
#   Generate the MRPTConfig.cmake file
# ----------------------------------------------------------------------------
SET(THE_MRPT_SOURCE_DIR "${MRPT_SOURCE_DIR}")
SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/libs")
SET(THE_CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}")
SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "${MRPT_CONFIG_FILE_INCLUDE_DIR}")
SET(MRPT_CONFIGFILE_IS_INSTALL 0)

CONFIGURE_FILE(
	"${MRPT_SOURCE_DIR}/parse-files/MRPTConfig.cmake.in"
    "${MRPT_BINARY_DIR}/MRPTConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding MRPT, e.g. find_package(MRPT 1.0.0 EXACT)
CONFIGURE_FILE(
	"${MRPT_SOURCE_DIR}/parse-files/MRPTConfig-version.cmake.in" 
	"${CMAKE_BINARY_DIR}/MRPTConfig-version.cmake" IMMEDIATE @ONLY)

# ----------------------------------------------------------------------------
#   Generate the MRPTConfig.cmake file for unix
#      installation in CMAKE_INSTALL_PREFIX
# ----------------------------------------------------------------------------
SET(MRPT_CONFIGFILE_IS_INSTALL 1)
IF(WIN32)
	SET(THE_MRPT_SOURCE_DIR "\${THIS_MRPT_CONFIG_PATH}")
	SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/libs")
	SET(THE_CMAKE_BINARY_DIR "\${THIS_MRPT_CONFIG_PATH}")
	SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "\${THIS_MRPT_CONFIG_PATH}/include/mrpt/mrpt-config/")
ELSE(WIN32)
	# Unix install. This .cmake file will end up in /usr/share/mrpt/MRPTConfig.cmake :
	IF (CMAKE_MRPT_USE_DEB_POSTFIXS)
		# We're building a .deb package: DESTDIR is NOT the final installation directory:
		SET(THE_MRPT_SOURCE_DIR "/usr")
		SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/include/mrpt")
		SET(THE_CMAKE_BINARY_DIR "/usr")
		SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "/usr/include/mrpt/mrpt-config/")
	ELSE(CMAKE_MRPT_USE_DEB_POSTFIXS)
		# Normal case: take the desired installation directory
		SET(THE_MRPT_SOURCE_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/include/mrpt")
		SET(THE_CMAKE_BINARY_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/mrpt/mrpt-config/")
	ENDIF(CMAKE_MRPT_USE_DEB_POSTFIXS)
ENDIF(WIN32)

CONFIGURE_FILE(
	"${MRPT_SOURCE_DIR}/parse-files/MRPTConfig.cmake.in"  
	"${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding MRPT, e.g. find_package(MRPT 1.0.0 EXACT)
CONFIGURE_FILE(
	"${MRPT_SOURCE_DIR}/parse-files/MRPTConfig-version.cmake.in" 
	"${MRPT_BINARY_DIR}/unix-install/MRPTConfig-version.cmake" IMMEDIATE @ONLY)
