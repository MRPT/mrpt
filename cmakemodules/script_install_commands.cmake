# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------
FILE(GLOB MRPT_PKGCONFIG_PC_FILES "${MRPT_BINARY_DIR}/pkgconfig/mrpt-*.pc")

IF(EXISTS "${MRPT_BINARY_DIR}/pkgconfig/mrpt-base.pc" AND NOT IS_DEBIAN_DBG_PKG)
	INSTALL(
		FILES ${MRPT_PKGCONFIG_PC_FILES}
		DESTINATION ${libmrpt_dev_INSTALL_PREFIX}lib${LIB_SUFFIX}/pkgconfig )
ENDIF(EXISTS "${MRPT_BINARY_DIR}/pkgconfig/mrpt-base.pc" AND NOT IS_DEBIAN_DBG_PKG)

# CMake will look for MRPTConfig.cmake at: /usr/share|lib/mrpt
IF(WIN32)
	INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ./ )
ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}share/mrpt )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# Docs, examples and the rest of files:
IF(WIN32)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc" DESTINATION ./ PATTERN ".svn" EXCLUDE)

	IF (PACKAGE_INCLUDES_SOURCES)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/cmakemodules" DESTINATION ./ PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs" DESTINATION ./ PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples"  DESTINATION ./ COMPONENT Examples PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/scripts" DESTINATION ./  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/swig" DESTINATION ./ PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/tests" DESTINATION ./  PATTERN ".svn" EXCLUDE)
	ENDIF (PACKAGE_INCLUDES_SOURCES)
		
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/parse-files" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share" DESTINATION ./  PATTERN ".svn" EXCLUDE)

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------
	# wxWidgets:
	IF (EXISTS "${wxWidgets_LIB_DIR}")
		FILE(GLOB_RECURSE EXTRA_DLLS "${wxWidgets_LIB_DIR}/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${wxWidgets_LIB_DIR}")

	# OpenCV:
	IF (EXISTS "${OpenCV_DIR}/bin/Release")
		FILE(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${OpenCV_DIR}/bin/Release")

        # Intel TBB dlls
        if(CMAKE_MRPT_HAS_TBB)
            string(REGEX REPLACE "/lib" "/bin" TBB_DLL_DIR "${TBB_LIB_DIR}")
            install(PROGRAMS "${TBB_DLL_DIR}/tbb.dll" DESTINATION bin COMPONENT main)
            install(PROGRAMS "${TBB_DLL_DIR}/tbb_debug.dll" DESTINATION bin COMPONENT main)
        endif(CMAKE_MRPT_HAS_TBB)

	# ffmpeg:
	IF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		FILE(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")

	# libfreenect for windows: Copy the libusb.dll file:
	IF (NOT UNIX AND CMAKE_MRPT_HAS_FREENECT)
		IF (NOT "${LIBUSB_1_DLL_FOR_INSTALL}" STREQUAL "")
			INSTALL(FILES "${LIBUSB_1_DLL_FOR_INSTALL}/libusb0.dll" DESTINATION bin)
		ENDIF (NOT "${LIBUSB_1_DLL_FOR_INSTALL}" STREQUAL "")
	ENDIF (NOT UNIX AND CMAKE_MRPT_HAS_FREENECT)

	# Extra optional DLLs to be installed in the "bin" folder:
	file(TO_CMAKE_PATH "$ENV{MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL)
	IF (NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
			FILE(STRINGS "${MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL_FILES)
			FOREACH(XFIL ${MRPT_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				INSTALL(FILES "${XFIL2}" DESTINATION bin)
			ENDFOREACH(XFIL)
		endif (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
	ENDIF(NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")

	# My own debug DLLs:
	FILE(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Debug/*.dll")
	FOREACH(F ${EXTRA_DLLS})
		INSTALL(FILES "${F}" DESTINATION bin)
	ENDFOREACH(F)
	FILE(GLOB_RECURSE EXTRA_LIBS "${MRPT_BINARY_DIR}/lib/Debug/*.lib")
	FOREACH(F ${EXTRA_LIBS})
		INSTALL(FILES "${F}" DESTINATION lib)
	ENDFOREACH(F)

ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc/html" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  PATTERN ".svn" EXCLUDE)
		INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/mrpt_example1.tar.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")

		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/srba-guide/srba-guide.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/srba-guide/srba-guide.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/srba-guide/srba-guide.ps.gz")

		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")

		# applications config files
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/applications" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share PATTERN ".svn" EXCLUDE)
		#INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/menu" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mrpt" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/pixmaps" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)

	 	# Mime types go to the mrpt-core package
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mime" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# The headers of all the MRPT libs:
# (in win32 the /libs/* tree is install entirely, not only the headers):
IF (PACKAGE_INCLUDES_SOURCES)
	IF (UNIX AND NOT IS_DEBIAN_DBG_PKG)
		FOREACH(_LIB ${ALL_MRPT_LIBS})
			STRING(REGEX REPLACE "mrpt-(.*)" "\\1" _LIB ${_LIB})
			SET(SRC_DIR "${MRPT_SOURCE_DIR}/libs/${_LIB}/include/")
			IF (EXISTS "${SRC_DIR}")  # This is mainly to avoid a crash with "mrpt-core", which is a "virtual" MRPT module.
				INSTALL(DIRECTORY "${SRC_DIR}" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}include/mrpt/${_LIB}/include/  PATTERN ".svn" EXCLUDE)
			ENDIF (EXISTS "${SRC_DIR}")
		ENDFOREACH(_LIB)
	ENDIF(UNIX AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)


# Config-dependent headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
		INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)

# If using embedded version, install embedded version as part of mrpt-base's headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
		IF(WIN32)
			# Eigen headers must end up in /Program Files/MRPT-X.Y.Z/libs/base/...
			SET(MRPT_INSTALL_EIGEN_PREFIX "libs/base/include/")
		ELSE(WIN32)
			# Eigen headers must end up in /usr/...
			SET(MRPT_INSTALL_EIGEN_PREFIX "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/base/include/")
		ENDIF(WIN32)

		INSTALL(
			DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs/eigen3/Eigen"
			DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" PATTERN ".svn" EXCLUDE)
		INSTALL(
			DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs/eigen3/unsupported"
			DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" PATTERN ".svn" EXCLUDE)
	ENDIF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)
