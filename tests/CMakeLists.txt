INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
PROJECT(tests)

SET(EXECUTABLE_OUTPUT_PATH "${MRPT_BINARY_DIR}/bin")

# -----------------------------
# Add the "make test" target:
# -----------------------------
ADD_CUSTOM_TARGET(test_legacy)
ADD_CUSTOM_TARGET(tests_build_all)
set_target_properties(test_legacy PROPERTIES FOLDER "unit tests")
set_target_properties(tests_build_all PROPERTIES FOLDER "unit tests")
IF (UNIX)
	ADD_CUSTOM_TARGET(test_gdb)
ENDIF()


ADD_DEFINITIONS(-DCMAKE_MRPT_GLOBAL_UNITTEST_SRC_DIR="${MRPT_SOURCE_DIR}")

# Allow tests to include sources from examples directory:
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}")
# Include tests-specific hdrs:
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/tests/include")

# Tests based on Google gtest:
# -----------------------------

# Include gtest from embeded lib:
IF (NOT CMAKE_MRPT_HAS_GTEST_SYSTEM)
	INCLUDE_DIRECTORIES("${CMAKE_MRPT_GTEST_SRC_DIR}/include/")
ENDIF()

# If using system library, add C++ flags:
IF (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")
	ADD_DEFINITIONS(${CMAKE_GTEST_CFLAGS})
ENDIF ()

# Fix issue #367: undefined reference to symbol on tests linking
IF(NOT MSVC)
    SET(CMAKE_THREAD_LIBS_INIT ${CMAKE_THREAD_LIBS_INIT} "-lpthread")
ENDIF()

if(MSVC)
	# avoid MSVC pedantic warning on tr1. See: https://github.com/google/googletest/issues/1111
	ADD_DEFINITIONS(/D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)
endif()


get_property(LST_LIB_TESTS GLOBAL PROPERTY "MRPT_TEST_LIBS")
FOREACH(_TSTLIB ${LST_LIB_TESTS})
	STRING(REGEX REPLACE "mrpt_(.*)" "mrpt-\\1" _TSTLIB_DASH ${_TSTLIB})
	get_property(lstfiles GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_FILES")

	#MESSAGE(STATUS "Generating tests for ${_TSTLIB} -> ${_TSTLIB_DASH}")
	#MESSAGE(STATUS "  Files: ${lstfiles}")

	# Test project:
	ADD_EXECUTABLE( test_${_TSTLIB} test_main.cpp ${lstfiles})
	ADD_TEST(test_${_TSTLIB} ${EXECUTABLE_OUTPUT_PATH}/test_${_TSTLIB})
	add_coverage(test_${_TSTLIB})

	# Add the required libraries for linking:
	IF (CMAKE_MRPT_HAS_GTEST_SYSTEM)
		# System vesion:
		TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} gtest ${CMAKE_THREAD_LIBS_INIT})
	ELSE ()
		# Own version:
		TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} mrptgtest ${CMAKE_THREAD_LIBS_INIT})
		ADD_DEPENDENCIES(test_${_TSTLIB} mrptgtest)  # Assure the MRPT library is updated:
	ENDIF ()

	set_target_properties(test_${_TSTLIB} PROPERTIES "COMPILE_DEFINITIONS" "MRPT_TEST_LIB=${_TSTLIB}")

	# Deps:
	get_property(lst_deps GLOBAL PROPERTY "${_TSTLIB_DASH}_LIB_DEPS")
	# extra deps:
	get_property(extra_deps GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_EXTRA_DEPS")
	set(lst_deps ${lst_deps} ${extra_deps})
	
	set(lst_deps ${_TSTLIB_DASH} ${lst_deps}) # The order in this list is very important for linking!
	FOREACH(_DEPLIB ${lst_deps})
		# Include dirs:
		STRING(REGEX REPLACE "mrpt-(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
		# This link command automatically imports all required "include directories"
		# and other PUBLIC dependencies of the linked target:
		TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${_DEPLIB})
	ENDFOREACH(_DEPLIB)

	# Run it:
	SET(GENERATED_EXE "$<TARGET_FILE:test_${_TSTLIB}>")
	ADD_CUSTOM_TARGET(run_tests_${_TSTLIB} COMMAND "${GENERATED_EXE}")
	ADD_DEPENDENCIES(run_tests_${_TSTLIB} test_${_TSTLIB})
	ADD_DEPENDENCIES(test_legacy run_tests_${_TSTLIB})
	ADD_DEPENDENCIES(tests_build_all test_${_TSTLIB})

	IF (UNIX)
		# test_gdb
		ADD_CUSTOM_TARGET(run_tests_${_TSTLIB}_gdb
		COMMAND "gdb" "-batch" "-ex" "\"run\"" "-ex" "\"bt\"" "-return-child-result" "${GENERATED_EXE}")
		ADD_DEPENDENCIES(run_tests_${_TSTLIB}_gdb test_${_TSTLIB})
		ADD_DEPENDENCIES(test_gdb run_tests_${_TSTLIB}_gdb)
	ENDIF()

	set_target_properties(test_${_TSTLIB} PROPERTIES FOLDER "unit tests")
	set_target_properties(run_tests_${_TSTLIB} PROPERTIES FOLDER "unit tests")

ENDFOREACH(_TSTLIB)
