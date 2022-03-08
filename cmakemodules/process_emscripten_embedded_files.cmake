
# Parse all .cpp sources of a given `target`, and for each comment like:
#  //! JS_PRELOAD_FILE <name>
# it will append a link flag like: 
#  --preload-file <name>
# Only if building for Emscripten. It has no effects otherwise.
#
# 07/March/2022, Jose Luis Blanco Claraco
#
function(process_emscripten_embedded_files targetName_)
	if (NOT "${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten")
		return()
	endif()

	get_target_property(targetCWD ${targetName_} BINARY_DIR)

	get_target_property(MY_SOURCES ${targetName_} SOURCES)
	foreach(fil_ ${MY_SOURCES})
		#message(STATUS "Processing: ${fil_}")
		file(STRINGS "${fil_}" content_)
		string(REGEX MATCHALL "//! JS_PRELOAD_FILE.+\\|([-A-Za-z0-9_\\./]+)\\|" myMatches_ "${content_}")
		foreach(match_ ${myMatches_})
			message(STATUS "match_ 1: ${match_}")
			string(REGEX REPLACE "<(.+)>" "\\1" f_ "${match_}")
			message(STATUS "f: ${f_}")
			if (NOT f_)
				continue()
			endif()
			message(STATUS "[emscripten] Embedding file '${f_}' into '${targetName_}'")

			if(NOT COMMAND target_link_options)
				message("Warning: This feature requires target_link_options() [cmake >=3.13]")
				return()
			endif()

			set(absPath ${MRPT_SOURCE_DIR}/${f_})
			set(relativePath ${f_})

			# Create copies (symlinks dont work...) to access the files via relative paths:
			execute_process(COMMAND bash -c "mkdir -p \$(dirname ${relativePath})" WORKING_DIRECTORY ${targetCWD})
			execute_process(COMMAND bash -c "cp ${absPath} ${relativePath}" WORKING_DIRECTORY ${targetCWD})

			target_link_options(${targetName_} PRIVATE --preload-file ${relativePath})
		endforeach()
	endforeach()

	# Make the .data file accessible to the emscripten linker:
	execute_process(COMMAND bash -c "ln -s ${EXECUTABLE_OUTPUT_PATH}/${targetName_}.data ${targetCWD}/${targetName_}.data"  OUTPUT_QUIET ERROR_QUIET)

endfunction()
