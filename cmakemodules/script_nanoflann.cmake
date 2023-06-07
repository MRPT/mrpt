# ===================================================
# nanoflann
# ===================================================
set(CMAKE_MRPT_HAS_NANOFLANN 1) # nanoflann is a mandatory dependency

# Where the embedded version would be built:
set(nanoflann_EMBEDDED_BUILD_DIR "${MRPT_BINARY_DIR}/3rdparty/nanoflann")

find_package(nanoflann CONFIG QUIET)

if (NOT nanoflann_FOUND)
    message(STATUS "--- Running CMake on external submodule 'nanoflann'...")
    file(MAKE_DIRECTORY "${nanoflann_EMBEDDED_BUILD_DIR}")
    if(NOT ${CMAKE_VERSION} VERSION_LESS "3.15")
        set(echo_flag COMMAND_ECHO STDOUT)
    endif()
    execute_process(COMMAND
        ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" "${MRPT_SOURCE_DIR}/3rdparty/nanoflann"
        -DNANOFLANN_BUILD_EXAMPLES=OFF
        -DNANOFLANN_BUILD_TESTS=OFF
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${nanoflann_EMBEDDED_BUILD_DIR}"
      ${echo_flag}
      )
    if(result)
      message(FATAL_ERROR "CMake step for nanoflann failed: ${result}")
    endif()
    message(STATUS "--- End running CMake")

    # Search again:
    set(nanoflann_DIR "${nanoflann_EMBEDDED_BUILD_DIR}" CACHE PATH "Path to nanoflann CMake config file" FORCE)
    mark_as_advanced(nanoflann_DIR)
    find_package(nanoflann CONFIG QUIET)
endif()

if (NOT nanoflann_FOUND)
    message(FATAL_ERROR "nanoflann not found, neither as system library nor git submodule. Check error messages above for possible reasons.")
endif()

# system library?:
if (NOT "${nanoflann_DIR}" STREQUAL "${nanoflann_EMBEDDED_BUILD_DIR}")
    set(CMAKE_MRPT_HAS_NANOFLANN_SYSTEM 1)
else()
    set(CMAKE_MRPT_HAS_NANOFLANN_SYSTEM 0)

    # install the embedded copy too (we need nanoflann-config.cmake, etc.)
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build \"${nanoflann_EMBEDDED_BUILD_DIR}\" --target install)")
endif()

if ($ENV{VERBOSE})
    message(STATUS "nanoflann_DIR: ${nanoflann_DIR}")
endif()
