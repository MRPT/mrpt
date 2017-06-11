INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
PROJECT(tests)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif()

# -----------------------------
# Add the "make test" target:
# -----------------------------
ADD_CUSTOM_TARGET(test)
ADD_CUSTOM_TARGET(tests_build_all)
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test PROPERTIES FOLDER "unit tests")
	set_target_properties(tests_build_all PROPERTIES FOLDER "unit tests")
endif()
IF (UNIX)
	ADD_CUSTOM_TARGET(test_gdb)
ENDIF()


ADD_DEFINITIONS(-DCMAKE_MRPT_GLOBAL_UNITTEST_SRC_DIR="${MRPT_SOURCE_DIR}")

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


get_property(LST_LIB_TESTS GLOBAL PROPERTY "MRPT_TEST_LIBS")
FOREACH(_TSTLIB ${LST_LIB_TESTS})
	STRING(REGEX REPLACE "mrpt_(.*)" "mrpt-\\1" _TSTLIB_DASH ${_TSTLIB})
	get_property(lstfiles GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_FILES")

	#MESSAGE(STATUS "Generating tests for ${_TSTLIB} -> ${_TSTLIB_DASH}")
	#MESSAGE(STATUS "  Files: ${lstfiles}")

	# Test project:
	ADD_EXECUTABLE( test_${_TSTLIB} test_main.cpp ${lstfiles})

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
	set(lst_deps ${_TSTLIB_DASH} ${lst_deps}) # The order in this list is very important for linking!
	FOREACH(_DEPLIB ${lst_deps})
		# Include dirs:
		STRING(REGEX REPLACE "mrpt-(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
		include_directories("${MRPT_LIBS_ROOT}/${_DEPLIB_DASH}/include")

		# Deps:
		ADD_DEPENDENCIES(test_${_TSTLIB} ${_DEPLIB})
		# Linking deps:
		get_property(_LIB_HDRONLY GLOBAL PROPERTY "${_DEPLIB}_LIB_IS_HEADERS_ONLY")
		IF(NOT _LIB_HDRONLY)
			TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${_DEPLIB})
		ENDIF(NOT _LIB_HDRONLY)
	ENDFOREACH(_DEPLIB)



	# Run it:
	SET(GENERATED_EXE "$<TARGET_FILE:test_${_TSTLIB}>")
	ADD_CUSTOM_TARGET(run_tests_${_TSTLIB} COMMAND "${GENERATED_EXE}")
	ADD_DEPENDENCIES(run_tests_${_TSTLIB} test_${_TSTLIB})
	ADD_DEPENDENCIES(test run_tests_${_TSTLIB})
	ADD_DEPENDENCIES(tests_build_all test_${_TSTLIB})

IF (UNIX)
	# test_gdb
	ADD_CUSTOM_TARGET(run_tests_${_TSTLIB}_gdb
	COMMAND "gdb" "-batch" "-ex" "\"run\"" "-ex" "\"bt\"" "-return-child-result" "${GENERATED_EXE}")
	ADD_DEPENDENCIES(run_tests_${_TSTLIB}_gdb test_${_TSTLIB})
	ADD_DEPENDENCIES(test_gdb run_tests_${_TSTLIB}_gdb)
ENDIF()

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test_${_TSTLIB} PROPERTIES FOLDER "unit tests")
	set_target_properties(run_tests_${_TSTLIB} PROPERTIES FOLDER "unit tests")
endif()

ENDFOREACH(_TSTLIB)
