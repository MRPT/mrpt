include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory


# Dummy documentation target for showing grouped files in IDEs:
# ----------------------------------------------------------------------------
file(GLOB DOX_HDRS "${CMAKE_SOURCE_DIR}/doc/doxygen-pages/*.h")
set(DOC_INDIV_FILES
	"${CMAKE_SOURCE_DIR}/README.md"
	)
set(all_docs_files ${DOX_HDRS} ${DOC_INDIV_FILES})
add_custom_target(DocumentationFiles DEPENDS ${all_docs_files} SOURCES ${all_docs_files})
set_target_properties(DocumentationFiles PROPERTIES FOLDER "documentation")

# ----------------------------------------------------------------------------
# Targets for building the documentation of the MRPT
# ----------------------------------------------------------------------------

project(documentation)

if(UNIX)
	add_subdirectory(man-pages)
endif()

find_package(Doxygen)

if(DOXYGEN_EXECUTABLE)
	# --------------------------------
	#   documentation_html
	# --------------------------------
	if(WIN32)
		set(CMD build_docs-HTML.bat)
	else()
		set(CMD "build_docs.sh")
		set(PARAM "-h")
	endif()
	add_custom_target(documentation_html
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (html)"
		)
	set_target_properties(documentation_html PROPERTIES FOLDER "documentation")

	# --------------------------------
	#   documentation_chm
	# --------------------------------
	if(WIN32)
		set(CMD build_docs-CHM.bat)
	else()
		set(CMD "build_docs.sh")
		set(PARAM "-c")
	endif()
	add_custom_target(documentation_chm
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (chm)"
		)
	set_target_properties(documentation_chm PROPERTIES FOLDER "documentation")

	# --------------------------------
	#   documentation_html_chm
	# --------------------------------
	if(WIN32)
		set(CMD build_docs-HTML-CHM.bat)
	else()
		set(CMD "build_docs.sh")
		set(PARAM "-ch")
	endif()
	add_custom_target(documentation_html_chm
		COMMAND 		${MRPT_SOURCE_DIR}/scripts/${CMD} ${PARAM}
		WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
		COMMENT 		"Generating MRPT documentation (html+chm)"
		)
	set_target_properties(documentation_html_chm PROPERTIES FOLDER "documentation")
endif()


# Compile Latex targets:
if(NOT WIN32)
find_program(LATEXMK_PROGRAM latexmk)
mark_as_advanced(LATEXMK_PROGRAM)
if(LATEXMK_PROGRAM)
	# The main (empty) target:
	add_custom_target(documentation_psgz_guides)

    # pbmap-guide
	add_custom_target(documentation_pbmap-guide
		COMMAND 		make -C ${MRPT_SOURCE_DIR}/doc/pbmap-guide/ pbmap-guide.ps.gz
		COMMENT 		"Generating pbmap-guide (ps.gz)"
		)
	add_dependencies(documentation_psgz_guides documentation_pbmap-guide)

	# graphslam-engine-guide
	add_custom_target(documentation_graphslam-engine-guide
		COMMAND 		make -C ${MRPT_SOURCE_DIR}/doc/graphslam-engine-guide/ graphslam-engine-guide.ps.gz
		COMMENT 		"Generating graphslam-engine-guide (ps.gz)"
		)
	add_dependencies(documentation_psgz_guides documentation_graphslam-engine-guide)

endif()
endif()
