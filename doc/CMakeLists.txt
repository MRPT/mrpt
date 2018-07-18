include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory


# Dummy documentation target for showing grouped files in IDEs:
# ----------------------------------------------------------------------------
file(GLOB DOX_HDRS "${CMAKE_SOURCE_DIR}/doc/doxygen-pages/*.h")
file(GLOB_RECURSE RST_FILES "${CMAKE_SOURCE_DIR}/doc/*.rst")
set(DOC_INDIV_FILES
	"${CMAKE_SOURCE_DIR}/README.md"
	)
set(all_docs_files ${DOX_HDRS} ${RST_FILES} ${DOC_INDIV_FILES})
add_custom_target(DocumentationFiles DEPENDS ${all_docs_files} SOURCES ${all_docs_files})
set_target_properties(DocumentationFiles PROPERTIES FOLDER "documentation")

# ----------------------------------------------------------------------------
# Targets for building the documentation of the MRPT
# ----------------------------------------------------------------------------

project(documentation)

if(UNIX)
	add_subdirectory(man-pages)
endif()

if(DOXYGEN_EXECUTABLE)
	# --------------------------------
	#   documentation_html
	# --------------------------------
	add_custom_target(documentation_html
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build.py ${MRPT_VERSION}
		COMMENT "Generating MRPT documentation (html)"
		SOURCES build.py conf.py _templates/layout.html ${RST_FILES}
		)
	set_target_properties(documentation_html PROPERTIES FOLDER "documentation")
endif()


# Compile Latex targets:
if(NOT WIN32)
find_program(LATEXMK_PROGRAM latexmk)
mark_as_advanced(LATEXMK_PROGRAM)
if(LATEXMK_PROGRAM)
	# The main (empty) target:
	add_custom_target(documentation_psgz_guides)

	# graphslam-engine-guide
	add_custom_target(documentation_graphslam-engine-guide
		COMMAND 		make -C ${MRPT_SOURCE_DIR}/doc/graphslam-engine-guide/ graphslam-engine-guide.ps.gz
		COMMENT 		"Generating graphslam-engine-guide (ps.gz)"
		)
	add_dependencies(documentation_psgz_guides documentation_graphslam-engine-guide)

endif()
endif()
