include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
project(tests)

set(EXECUTABLE_OUTPUT_PATH "${MRPT_BINARY_DIR}/bin")

# -----------------------------
# Add the "make test" target:
# -----------------------------
add_custom_target(test_legacy)
add_custom_target(tests_build_all)
set_target_properties(test_legacy PROPERTIES FOLDER "unit tests")
set_target_properties(tests_build_all PROPERTIES FOLDER "unit tests")
if (UNIX)
	add_custom_target(test_gdb)
endif()


add_definitions(-DCMAKE_MRPT_GLOBAL_UNITTEST_SRC_DIR="${MRPT_SOURCE_DIR}")

# Allow tests to include sources from examples directory:
include_directories("${MRPT_SOURCE_DIR}")
# Include tests-specific hdrs:
include_directories("${MRPT_SOURCE_DIR}/tests/include")

# Tests based on Google gtest:
# -----------------------------

# Include gtest from embeded lib:
if (NOT CMAKE_MRPT_HAS_GTEST_SYSTEM)
	include_directories("${CMAKE_MRPT_GTEST_SRC_DIR}/include/")
endif()

# If using system library, add C++ flags:
if (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")
	add_definitions(${CMAKE_GTEST_CFLAGS})
endif ()

# Fix issue #367: undefined reference to symbol on tests linking
if(NOT MSVC)
    set(CMAKE_THREAD_LIBS_INIT ${CMAKE_THREAD_LIBS_INIT} "-lpthread")
endif()

if(MSVC)
	# avoid MSVC pedantic warning on tr1. See: https://github.com/google/googletest/issues/1111
	add_definitions(/D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING)
endif()


get_property(LST_LIB_TESTS GLOBAL PROPERTY "MRPT_TEST_LIBS")
foreach(_TSTLIB ${LST_LIB_TESTS})
	string(REGEX REPLACE "mrpt_(.*)" "mrpt-\\1" _TSTLIB_DASH ${_TSTLIB})
	get_property(lstfiles GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_FILES")

	#message(STATUS "Generating tests for ${_TSTLIB} -> ${_TSTLIB_DASH}")
	#message(STATUS "  Files: ${lstfiles}")

	# Test project:
	add_executable( test_${_TSTLIB} test_main.cpp ${lstfiles})
	add_test(test_${_TSTLIB}_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_${_TSTLIB})
	add_test(test_${_TSTLIB}_run ${EXECUTABLE_OUTPUT_PATH}/test_${_TSTLIB})
	set_tests_properties(test_${_TSTLIB}_run PROPERTIES DEPENDS test_${_TSTLIB}_build)

	add_coverage(test_${_TSTLIB})

	# Add the required libraries for linking:
	if (CMAKE_MRPT_HAS_GTEST_SYSTEM)
		# System vesion:
		target_link_libraries(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} gtest ${CMAKE_THREAD_LIBS_INIT})
	else ()
		# Own version:
		target_link_libraries(test_${_TSTLIB} ${MRPT_LINKER_LIBS_RELorDEB} mrptgtest ${CMAKE_THREAD_LIBS_INIT})
		add_dependencies(test_${_TSTLIB} mrptgtest)  # Assure the MRPT library is updated:
	endif ()

	set_target_properties(test_${_TSTLIB} PROPERTIES "COMPILE_DEFINITIONS" "MRPT_TEST_LIB=${_TSTLIB}")

	# Deps:
	get_property(lst_deps GLOBAL PROPERTY "${_TSTLIB_DASH}_LIB_DEPS")
	# extra deps:
	get_property(extra_deps GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_EXTRA_DEPS")
	set(lst_deps ${lst_deps} ${extra_deps})

	set(lst_deps ${_TSTLIB_DASH} ${lst_deps}) # The order in this list is very important for linking!
	foreach(_DEPLIB ${lst_deps})
		# Include dirs:
		string(REGEX REPLACE "mrpt-(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
		# This link command automatically imports all required "include directories"
		# and other PUBLIC dependencies of the linked target:
		target_link_libraries(test_${_TSTLIB} ${_DEPLIB})
	endforeach(_DEPLIB)

	# Run it:
	set(GENERATED_EXE "$<TARGET_FILE:test_${_TSTLIB}>")
	add_custom_target(run_tests_${_TSTLIB} COMMAND "${GENERATED_EXE}")
	add_dependencies(run_tests_${_TSTLIB} test_${_TSTLIB})
	add_dependencies(test_legacy run_tests_${_TSTLIB})
	add_dependencies(tests_build_all test_${_TSTLIB})

	if (UNIX)
		# test_gdb
		add_custom_target(run_tests_${_TSTLIB}_gdb
		COMMAND "gdb" "-batch" "-ex" "\"run\"" "-ex" "\"bt\"" "-return-child-result" "${GENERATED_EXE}")
		add_dependencies(run_tests_${_TSTLIB}_gdb test_${_TSTLIB})
		add_dependencies(test_gdb run_tests_${_TSTLIB}_gdb)
	endif()

	set_target_properties(test_${_TSTLIB} PROPERTIES FOLDER "unit tests")
	set_target_properties(run_tests_${_TSTLIB} PROPERTIES FOLDER "unit tests")

endforeach(_TSTLIB)
