# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------

# CMake will look for MRPTConfig.cmake at: /usr/share|lib/mrpt
if(WIN32)
	install(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ./ )
	install(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig-version.cmake" DESTINATION ./ )
else()
	if (NOT IS_DEBIAN_DBG_PKG)
		install(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ${libmrpt_common_dev_INSTALL_PREFIX}share/mrpt )
		install(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig-version.cmake" DESTINATION ${libmrpt_common_dev_INSTALL_PREFIX}share/mrpt )
	endif(NOT IS_DEBIAN_DBG_PKG)
endif()

# Docs, examples and the rest of files:
if(WIN32)
	install(DIRECTORY "${MRPT_SOURCE_DIR}/doc" DESTINATION ./ )

	install(DIRECTORY "${MRPT_SOURCE_DIR}/cmakemodules" DESTINATION ./ )
	install(DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs" DESTINATION ./ )
	install(DIRECTORY "${MRPT_SOURCE_DIR}/samples"  DESTINATION ./
		COMPONENT Examples
		PATTERN ".gitignore" EXCLUDE)
	install(DIRECTORY "${MRPT_SOURCE_DIR}/scripts" DESTINATION ./  )
	install(DIRECTORY "${MRPT_SOURCE_DIR}/tests" DESTINATION ./  )

	install(DIRECTORY "${MRPT_SOURCE_DIR}/parse-files" DESTINATION ./ )
	install(DIRECTORY "${MRPT_SOURCE_DIR}/share"
		DESTINATION ./
		PATTERN ".gitignore" EXCLUDE
	)

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------
	# wxWidgets:
	if (EXISTS "${wxWidgets_LIB_DIR}")
		file(GLOB_RECURSE EXTRA_DLLS "${wxWidgets_LIB_DIR}/*.dll")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach(F)
	endif (EXISTS "${wxWidgets_LIB_DIR}")

	# Qt5:
	if (${CMAKE_MRPT_HAS_Qt5})
		foreach(F ${Qt5_COMPONENTS_LIBS})
			install(FILES "${F}" DESTINATION bin)
		endforeach(F)
	endif ()

	# OpenCV:
	if (EXISTS "${OpenCV_DIR}/bin/Release")
		file(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach(F)
	endif (EXISTS "${OpenCV_DIR}/bin/Release")

	# ffmpeg:
	if (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		file(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach(F)
	endif (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")

	# Extra optional DLLs to be installed in the "bin" folder:
	file(TO_CMAKE_PATH "$ENV{MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL)
	if (NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
			file(STRINGS "${MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL_FILES)
			foreach(XFIL ${MRPT_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				install(FILES "${XFIL2}" DESTINATION bin)
			endforeach(XFIL)
		endif (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
	endif(NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")

	# My own debug DLLs:
	file(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Debug/*.dll")
	foreach(F ${EXTRA_DLLS})
		install(FILES "${F}" DESTINATION bin)
	endforeach(F)
	file(GLOB_RECURSE EXTRA_LIBS "${MRPT_BINARY_DIR}/lib/Debug/*.lib")
	foreach(F ${EXTRA_LIBS})
		install(FILES "${F}" DESTINATION lib)
	endforeach(F)

else(WIN32)
	if (NOT IS_DEBIAN_DBG_PKG)
		install(DIRECTORY "${MRPT_SOURCE_DIR}/doc/html" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  )
		install(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  )
		if(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")
			install(FILES "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		endif(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")

		if(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")
			install(FILES "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		endif(EXISTS "${MRPT_SOURCE_DIR}/doc/pbmap-guide/pbmap-guide.ps.gz")

		# applications config files
		install(DIRECTORY "${MRPT_SOURCE_DIR}/share/applications" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		install(DIRECTORY "${MRPT_SOURCE_DIR}/share/mrpt" DESTINATION ${mrpt_common_INSTALL_PREFIX}share)
		install(DIRECTORY "${MRPT_SOURCE_DIR}/share/pixmaps" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		install(DIRECTORY "${MRPT_SOURCE_DIR}/share/metainfo" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)

	 	# Mime types go to the mrpt-core package
		install(DIRECTORY "${MRPT_SOURCE_DIR}/share/mime" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share )
	endif(NOT IS_DEBIAN_DBG_PKG)
endif(WIN32)

# Config-dependent headers:
if (NOT IS_DEBIAN_DBG_PKG)
	install(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h" DESTINATION "${libmrpt_common_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
	install(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h" DESTINATION "${libmrpt_common_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
endif()

# Using embedded version of libraries that need public headers?
if(EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
	if(WIN32)
		# Eigen headers must end up in /Program Files/MRPT-X.Y.Z/libs/base/...
		set(MRPT_INSTALL_EIGEN_PREFIX "libs/math/include/")
	else(WIN32)
		# Eigen headers must end up in /usr/...
		set(MRPT_INSTALL_EIGEN_PREFIX "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/math/include/")
	endif(WIN32)

	install(
		DIRECTORY "${MRPT_BINARY_DIR}/otherlibs/eigen3/Eigen"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" )
	install(
		DIRECTORY "${MRPT_BINARY_DIR}/otherlibs/eigen3/unsupported"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" )
endif()
