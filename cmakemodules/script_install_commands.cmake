# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------

# CMake will look for MRPTConfig.cmake at: /usr/share|lib/mrpt
IF(WIN32)
	INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ./ )
	INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig-version.cmake" DESTINATION ./ )
ELSE()
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}share/mrpt )
		INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig-version.cmake" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}share/mrpt )
        install(EXPORT MRPTTargets DESTINATION share/mrpt)
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF()

# Docs, examples and the rest of files:
IF(WIN32)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc" DESTINATION ./ )

	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/cmakemodules" DESTINATION ./ )
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs" DESTINATION ./ )
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples"  DESTINATION ./
		COMPONENT Examples
		PATTERN ".gitignore" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/scripts" DESTINATION ./  )
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/tests" DESTINATION ./  )

	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/parse-files" DESTINATION ./ )
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share"
		DESTINATION ./
		PATTERN ".gitignore" EXCLUDE
	)

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------
	# wxWidgets:
	IF (EXISTS "${wxWidgets_LIB_DIR}")
		FILE(GLOB_RECURSE EXTRA_DLLS "${wxWidgets_LIB_DIR}/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${wxWidgets_LIB_DIR}")

	# Qt5:
	IF (${CMAKE_MRPT_HAS_Qt5})
		FOREACH(F ${Qt5_COMPONENTS_LIBS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF ()

	# OpenCV:
	IF (EXISTS "${OpenCV_DIR}/bin/Release")
		FILE(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${OpenCV_DIR}/bin/Release")

	# ffmpeg:
	IF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		FILE(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")

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
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc/html" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  )
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  )
		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")

		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")

		# applications config files
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/applications" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mrpt" DESTINATION ${mrpt_common_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/pixmaps" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/metainfo" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)

	 	# Mime types go to the mrpt-core package
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mime" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# Config-dependent headers:
IF (NOT IS_DEBIAN_DBG_PKG)
	INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt" )
	INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt" )
ENDIF()

# Using embedded version of libraries that need public headers?
IF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
	IF(WIN32)
		# Eigen headers must end up in /Program Files/MRPT-X.Y.Z/libs/base/...
		SET(MRPT_INSTALL_EIGEN_PREFIX "libs/base/include/")
	ELSE(WIN32)
		# Eigen headers must end up in /usr/...
		SET(MRPT_INSTALL_EIGEN_PREFIX "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/base/include/")
	ENDIF(WIN32)

	INSTALL(
		DIRECTORY "${MRPT_BINARY_DIR}/otherlibs/eigen3/Eigen"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" )
	INSTALL(
		DIRECTORY "${MRPT_BINARY_DIR}/otherlibs/eigen3/unsupported"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" )
ENDIF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
