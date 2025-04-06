# Build python bindings?:
# ===================================================

# disabled on start
set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)

# Leave at the user's choice to disable the python bindings:
option(MRPT_DISABLE_PYTHON_BINDINGS "Disable the build (if possible) of Python bindings" "OFF")
if(MRPT_DISABLE_PYTHON_BINDINGS)
    set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)
endif()

if (NOT MRPT_DISABLE_PYTHON_BINDINGS AND (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"))
    set(MRPT_DISABLE_PYTHON_BINDINGS ON CACHE BOOL "Disable python wrappers" FORCE)
    message(STATUS "*WARNING* Disabling Python wrappers: not supported if built using clang (it leads to pybind11-generated code errors)")
endif()


if(UNIX AND NOT MRPT_DISABLE_PYTHON_BINDINGS)
	# Requires CMake 3.13+
    if(NOT ${CMAKE_VERSION} VERSION_LESS "3.12.0")
        find_package(Python3 COMPONENTS Interpreter Development)
    else()
        # When cmake 3.12 is available everywhere, delete this branch of the if()
        find_program(Python3_EXECUTABLE NAMES python3)
        set(Python3_VERSION_MAJOR ${Python_VERSION_MAJOR})
        set(Python3_VERSION_MINOR ${Python_VERSION_MINOR})
    endif()

    if (Python3_FOUND)
        if ("${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}" VERSION_LESS "3.8")
            message(ERROR "Disable Python wrappers: requires Python version >=3.8, but found: ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
        endif()
        
        # Enforce using python3:
        set(Python_ADDITIONAL_VERSIONS ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR})
        set(PYBIND11_PYTHON_VERSION ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR})
        find_package(pybind11 CONFIG)
        
        # The PYTHON build dir must be exactly at the same level than the root binary dir
        # in order for the different build tools (debuild,...) pass the correct relative 
        # path where to install the built python packages (e.g. under <build>/debian/tmp )
        set(MRPT_PYTHON_BUILD_DIRECTORY ${MRPT_BINARY_DIR})
    
        if (pybind11_FOUND AND pybind11_VERSION VERSION_LESS 2.2)
            message(ERROR "Disable Python bindings: pybind11 >=2.2 is required but only found version ${pybind11_VERSION}")
            set(pybind11_FOUND OFF)
        endif()
    
        if (pybind11_FOUND)
            # build python bindings if we have all requirements
            set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 1)
        endif()
    endif()
endif()
