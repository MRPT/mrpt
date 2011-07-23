INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
PROJECT(tests)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# -----------------------------
# Add the "make test" target:
# -----------------------------
ADD_CUSTOM_TARGET(test)

# Tests based on Google gtest:
# -----------------------------

# Include gtest from embeded lib:
IF (NOT CMAKE_MRPT_HAS_GTEST_SYSTEM)
	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/gtest-1.5.0/fused-src/")
ENDIF(NOT CMAKE_MRPT_HAS_GTEST_SYSTEM)

# If using system library, add C++ flags:
IF (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")
	ADD_DEFINITIONS(${CMAKE_GTEST_CFLAGS})
ENDIF (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")


get_property(LST_LIB_TESTS GLOBAL PROPERTY "MRPT_TEST_LIBS")
FOREACH(_TSTLIB ${LST_LIB_TESTS})
	STRING(REGEX REPLACE "mrpt_(.*)" "mrpt-\\1" _TSTLIB_DASH ${_TSTLIB})
	get_property(lstfiles GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_FILES")

	MESSAGE(STATUS "Generating tests for ${_TSTLIB} -> ${_TSTLIB_DASH}")
	#MESSAGE(STATUS "  Files: ${lstfiles}")

	# Test project:
	ADD_EXECUTABLE( test_${_TSTLIB} test_main.cpp ${lstfiles})

	# Add the required libraries for linking:
IF (CMAKE_MRPT_HAS_GTEST_SYSTEM)
	# System vesion:
	TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} gtest)
ELSE (CMAKE_MRPT_HAS_GTEST_SYSTEM)
	# Own vesion:
	TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} mrptgtest) 
	ADD_DEPENDENCIES(test_${_TSTLIB} mrptgtest)  # Assure the MRPT library is updated:
ENDIF (CMAKE_MRPT_HAS_GTEST_SYSTEM)


	set_target_properties(test_${_TSTLIB} PROPERTIES "COMPILE_DEFINITIONS" "MRPT_TEST_LIB=${_TSTLIB}")

	# Deps:
	get_property(lst_deps GLOBAL PROPERTY "${_TSTLIB_DASH}_LIB_DEPS")
	set(lst_deps ${_TSTLIB_DASH} ${lst_deps}) # The order in this list is very important for linking!
	FOREACH(_DEPLIB ${lst_deps})
		# Include dirs:
		STRING(REGEX REPLACE "mrpt-(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
		include_directories("${MRPT_LIBS_ROOT}/${_DEPLIB_DASH}/include")

		# Linking deps:
		ADD_DEPENDENCIES(test_${_TSTLIB} ${_DEPLIB})
		TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${_DEPLIB})
	ENDFOREACH(_DEPLIB)
	

	
	# Run it:
	GET_TARGET_PROPERTY(GENERATED_EXE test_${_TSTLIB} LOCATION)
	ADD_CUSTOM_TARGET(run_tests_${_TSTLIB} COMMAND "${GENERATED_EXE}" "${MRPT_SOURCE_DIR}")
	ADD_DEPENDENCIES(run_tests_${_TSTLIB} test_${_TSTLIB})
	ADD_DEPENDENCIES(test run_tests_${_TSTLIB})

ENDFOREACH(_TSTLIB)


