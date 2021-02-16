# ===================================================
# nanoflann
# ===================================================
set(CMAKE_MRPT_HAS_NANOFLANN 1) # nanoflann is a mandatory dependency

find_package(nanoflann CONFIG QUIET)

if ($ENV{VERBOSE})
    message(STATUS "nanoflann_DIR: ${nanoflann_DIR}")
endif()

if (nanoflann_FOUND AND (NOT "${nanoflann_DIR}" STREQUAL "${MRPT_BINARY_DIR}/3rdparty/nanoflann"))
    # system library:
    set(CMAKE_MRPT_HAS_NANOFLANN_SYSTEM 1)

    # Target nanoflann::nanoflann is now already well defined.
    # Define this alias so we can list "nanoflann" as a direct dependency
    # of mrpt-math and it gets mapped into a find_dependency(nanoflann) command
    # in mrpt-math-config.cmake:
    add_library(nanoflann ALIAS nanoflann::nanoflann)
else()
    # embedded copy:
    set(CMAKE_MRPT_HAS_NANOFLANN_SYSTEM 0)

    set(NANOFLANN_BUILD_EXAMPLES "OFF" CACHE BOOL "" FORCE)
    set(NANOFLANN_BUILD_BENCHMARKS "OFF" CACHE BOOL "" FORCE)
    set(NANOFLANN_BUILD_TESTS "OFF" CACHE BOOL "" FORCE)
    mark_as_advanced(NANOFLANN_BUILD_EXAMPLES)
    mark_as_advanced(NANOFLANN_BUILD_BENCHMARKS)
    mark_as_advanced(NANOFLANN_BUILD_TESTS)

    add_subdirectory(3rdparty/nanoflann)
    set_target_properties(nanoflann PROPERTIES FOLDER "3rd party")
endif()
