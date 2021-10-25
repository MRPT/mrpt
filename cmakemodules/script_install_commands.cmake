# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------

# MRPTConfig.cmake: backwards-compatible file as it was named in mrpt v1.x
if(WIN32)
	set(cfg_ver_destdir "./")
else()
	set(cfg_ver_destdir "share/mrpt")
endif()
install(
	FILES
		"${MRPT_SOURCE_DIR}/parse-files/mrpt-config.cmake"
		"${CMAKE_BINARY_DIR}/mrpt-config-version.cmake"
	DESTINATION
		${cfg_ver_destdir}
	)

# Docs, examples and the rest of files:
if(WIN32)
	install(DIRECTORY
			"${MRPT_SOURCE_DIR}/doc"
			"${MRPT_SOURCE_DIR}/cmakemodules"
			"${MRPT_SOURCE_DIR}/3rdparty"
			"${MRPT_SOURCE_DIR}/scripts"
			"${MRPT_SOURCE_DIR}/tests"
			"${MRPT_SOURCE_DIR}/parse-files"
			"${MRPT_SOURCE_DIR}/share"
		DESTINATION ./)
	install(DIRECTORY "${MRPT_SOURCE_DIR}/samples"  DESTINATION ./
		COMPONENT Examples
		PATTERN ".gitignore" EXCLUDE)

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------

	# Extra optional DLLs to be installed in the "bin" folder:
	file(TO_CMAKE_PATH "$ENV{MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL)
	if (NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
			file(STRINGS "${MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL_FILES)
			foreach(XFIL ${MRPT_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				install(FILES "${XFIL2}" DESTINATION bin)
			endforeach()
		endif ()
	endif()

	# My own debug DLLs:
	file(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Debug/*.dll" "${MRPT_BINARY_DIR}/bin/Release/*.dll")
	foreach(F ${EXTRA_DLLS})
		install(FILES "${F}" DESTINATION bin)
	endforeach()
	file(GLOB_RECURSE EXTRA_LIBS "${MRPT_BINARY_DIR}/lib/Debug/*.lib" "${MRPT_BINARY_DIR}/lib/Release/*.lib" )
	foreach(F ${EXTRA_LIBS})
		install(FILES "${F}" DESTINATION lib)
	endforeach()

else(WIN32)
	install(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION share/doc/mrpt-doc/)

	# applications config files
	install(
		DIRECTORY
			"${MRPT_SOURCE_DIR}/share/applications"
			"${MRPT_SOURCE_DIR}/share/pixmaps"
			"${MRPT_SOURCE_DIR}/share/metainfo"
			"${MRPT_SOURCE_DIR}/share/mime"
		DESTINATION
			share
		)

		install(
			DIRECTORY
				"${MRPT_SOURCE_DIR}/share/mrpt"
			DESTINATION
				share
			)
endif()

# Config-dependent headers: to /usr/include/mrpt root dir.
install(
	FILES
		"${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h"
		"${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h"
		"${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/mrpt_paths_config.h"
	DESTINATION
		"include/mrpt/"
	)
