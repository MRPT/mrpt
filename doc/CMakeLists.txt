INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
# Targets for building the documentation of the MRPT
# ----------------------------------------------------------------------------

PROJECT(documentation)

IF(UNIX)
	add_subdirectory(man-pages)
ENDIF(UNIX)

FIND_PACKAGE(Doxygen)

# --------------------------------
# documentation_performance_html
# --------------------------------
if(TARGET mrpt-perfdata2html)
	GET_TARGET_PROPERTY(PERF2HTML_GENERATED_EXE mrpt-perfdata2html LOCATION)
	add_custom_target(documentation_performance_html
		COMMAND 		${PERF2HTML_GENERATED_EXE} "${MRPT_SOURCE_DIR}/doc/perf-data/"
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating performance HTML pages"
		)
	add_dependencies(documentation_performance_html mrpt-perfdata2html)
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(documentation_performance_html PROPERTIES FOLDER "documentation")
	endif()
endif()

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
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(documentation_html PROPERTIES FOLDER "documentation")
	endif(ENABLE_SOLUTION_FOLDERS)

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
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(documentation_chm PROPERTIES FOLDER "documentation")
	endif(ENABLE_SOLUTION_FOLDERS)

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
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(documentation_html_chm PROPERTIES FOLDER "documentation")
	endif(ENABLE_SOLUTION_FOLDERS)
		
ENDIF(DOXYGEN_EXECUTABLE)


# Compile Latex targets:
IF(NOT WIN32)
find_program(LATEXMK_PROGRAM latexmk)
mark_as_advanced(LATEXMK_PROGRAM)
IF(LATEXMK_PROGRAM)
	# The main (empty) target:
	add_custom_target(documentation_psgz_guides)

	add_custom_target(documentation_pbmap-guide
		COMMAND 		make -C ${MRPT_SOURCE_DIR}/doc/pbmap-guide/ pbmap-guide.ps.gz
		COMMENT 		"Generating pbmap-guide (ps.gz)"
		)
	add_dependencies(documentation_psgz_guides documentation_pbmap-guide)
	
ENDIF(LATEXMK_PROGRAM)
ENDIF(NOT WIN32)

