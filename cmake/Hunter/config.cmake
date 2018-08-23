# Copyright (c) 2013-2017, Lukas Solanka
# All rights reserved.

# Do not place header guards here

# Unset:
#   * ${PACKAGE_NAME}_ROOT (CMake variable)
#   * ${PACKAGE_NAME}_ROOT (CMake cache variable)
#   * ${PACKAGE_NAME}_ROOT (environment variable)

# Set CMake variables:
#   * HUNTER_${PACKAGE_NAME}_VERSION
#   * HUNTER_${PACKAGE_NAME}_CMAKE_ARGS (optionally)

# Usage:
#   hunter_config(Foo VERSION 1.0.0)
#   hunter_config(Boo VERSION 1.2.3z CMAKE_ARGS BOO_WITH_A=ON)

# Wiki:
#   * https://github.com/ruslo/hunter/wiki/dev.modules#hunter_config

include(hunter_config)
include(hunter_user_error)

# NOTE: no names with spaces!

hunter_config(OpenCV VERSION ${HUNTER_OpenCV_VERSION}
    CMAKE_ARGS
        WITH_OPENEXR=OFF
        WITH_IPP=OFF
        WITH_VTK=OFF
)

hunter_config(Assimp VERSION ${HUNTER_Assimp_VERSION}
    CMAKE_ARGS
    ASSIMP_BUILD_ASSIMP_TOOLS=OFF
    ASSIMP_BUILD_SAMPLES=OFF
    ASSIMP_BUILD_STATIC_LIB=ON
    ASSIMP_BUILD_TESTS=OFF
    ASSIMP_LIBRARY_SUFFIX=-mrpt
    #CMAKE_LIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
    #LIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
    #CMAKE_RUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
    #RUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
    CMAKE_DEBUG_POSTFIX=${CMAKE_DEBUG_POSTFIX}
)
