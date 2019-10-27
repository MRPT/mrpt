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
	add_custom_target(test_valgrind)
endif()


add_definitions(-DCMAKE_UNITTEST_BASEDIR="${MRPT_SOURCE_DIR}")

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
		set(gtest_lib_name gtest)
	else ()
		# Own version:
		set(gtest_lib_name mrptgtest)
	endif ()

	target_link_libraries(test_${_TSTLIB} ${gtest_lib_name} Threads::Threads)

	# Deps:
	get_property(lst_deps GLOBAL PROPERTY "${_TSTLIB_DASH}_LIB_DEPS")
	# extra deps:
	get_property(extra_deps GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_EXTRA_DEPS")
	set(lst_deps ${lst_deps} ${extra_deps})

	# The order in this list is very important for linking!
	# Otherwise, one may have the weird linking error:
	# "error adding symbols: DSO missing from command line"
	set(lst_deps ${_TSTLIB_DASH} ${lst_deps})
	foreach(_DEPLIB ${lst_deps})
		# Include dirs:
		if (_DEPLIB MATCHES "^mrpt-?:*(.*)")
			string(REGEX REPLACE "mrpt-?:*(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
			# This link command automatically imports all required "include directories"
			# and other PUBLIC dependencies of the linked target:
			target_link_libraries(test_${_TSTLIB} mrpt::${_DEPLIB_DASH})
		else()
			target_link_libraries(test_${_TSTLIB} ${_DEPLIB})
		endif()
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
			COMMAND "gdb" "-batch" "-ex" "\"run\"" "-ex" "\"bt\"" "-return-child-result" "${GENERATED_EXE}"
			)
		add_dependencies(run_tests_${_TSTLIB}_gdb test_${_TSTLIB})
		add_dependencies(test_gdb run_tests_${_TSTLIB}_gdb)

		# test_valgrind
		add_custom_target(run_tests_${_TSTLIB}_valgrind
			COMMAND "valgrind" "--error-exitcode=1" "${GENERATED_EXE}"
			)
		add_dependencies(run_tests_${_TSTLIB}_valgrind test_${_TSTLIB})
		add_dependencies(test_valgrind run_tests_${_TSTLIB}_valgrind)
	endif()

	set_target_properties(test_${_TSTLIB} PROPERTIES FOLDER "unit tests")
	set_target_properties(run_tests_${_TSTLIB} PROPERTIES FOLDER "unit tests")

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(test_${_TSTLIB} PRIVATE "-Wno-abi")
	endif()

endforeach()

# Special dependencies:
if (TARGET test_mrpt_img)
    if(CMAKE_MRPT_HAS_OPENCV)
	target_link_libraries(test_mrpt_img imp_opencv)
    endif()
endif()
