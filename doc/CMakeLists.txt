include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

project(documentation)

# Dummy documentation target for showing grouped files in IDEs:
# ----------------------------------------------------------------------------
file(GLOB_RECURSE RST_FILES "${CMAKE_SOURCE_DIR}/doc/*.rst")
file(GLOB_RECURSE H_FILES "${CMAKE_SOURCE_DIR}/doc/sources/*.h")
set(DOC_INDIV_FILES
	"${CMAKE_SOURCE_DIR}/README.md"
	)
set(all_docs_files ${RST_FILES} ${H_FILES} ${DOC_INDIV_FILES})
add_custom_target(DocumentationFiles DEPENDS ${all_docs_files} SOURCES ${all_docs_files})
set_target_properties(DocumentationFiles PROPERTIES FOLDER "documentation")

# Manpages
# ----------------------------------------------------------------------------
if(UNIX)
	add_subdirectory(man-pages)
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
