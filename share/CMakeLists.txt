project(mrpt_share_dir)

# install targets:
IF(WIN32)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share"
		DESTINATION ./
		PATTERN ".gitignore" EXCLUDE
	)
ELSE()
	IF (NOT IS_DEBIAN_DBG_PKG)
		# mrpt-common pkg:
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mrpt" DESTINATION ${mrpt_common_INSTALL_PREFIX}share)

		# mrpt-apps pkg:
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/applications" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/pixmaps" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/metainfo" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mime" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share )
	ENDIF()
ENDIF()
