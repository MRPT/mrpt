INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
# Targets for building the documentation of the MRPT
# ----------------------------------------------------------------------------

PROJECT(documentation)

IF(UNIX)
	add_subdirectory(man-pages)
ENDIF(UNIX)

FIND_PACKAGE(Doxygen)

IF(DOXYGEN_EXECUTABLE)
	# --------------------------------
	#   documentation_html
	# --------------------------------
	IF(WIN32)
		SET(CMD build_docs-HTML.bat)
	ELSE(WIN32)
		SET(CMD "build_docs.sh")
		SET(PARAM "-h")
	ENDIF(WIN32)
	add_custom_target(documentation_html
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (html)"
		)

	# --------------------------------
	#   documentation_chm
	# --------------------------------
	IF(WIN32)
		SET(CMD build_docs-CHM.bat)
	ELSE(WIN32)
		SET(CMD "build_docs.sh")
		SET(PARAM "-c")
	ENDIF(WIN32)
	add_custom_target(documentation_chm
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (chm)"
		)

	# --------------------------------
	#   documentation_html_chm
	# --------------------------------
	IF(WIN32)
		SET(CMD build_docs-HTML-CHM.bat)
	ELSE(WIN32)
		SET(CMD "build_docs.sh")
		SET(PARAM "-ch")
	ENDIF(WIN32)
	add_custom_target(documentation_html_chm
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (html+chm)"
		)
		
ENDIF(DOXYGEN_EXECUTABLE)
