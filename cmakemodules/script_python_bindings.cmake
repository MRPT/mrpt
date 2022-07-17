# Build python bindings?:
# ===================================================

# disabled on start
set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)

# Leave at the user's choice to disable the python bindings:
option(MRPT_DISABLE_PYTHON_BINDINGS "Disable the build (if possible) of Python bindings" "OFF")
if(MRPT_DISABLE_PYTHON_BINDINGS)
    set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)
endif()

if(UNIX AND NOT MRPT_DISABLE_PYTHON_BINDINGS)
	# Requires CMake 3.13+
	find_package(Python3 COMPONENTS Development QUIET)

    if (Python3_FOUND)
        string(REGEX MATCHALL "[0-9]+" MY_PYTHON3_VERSION_PARTS "${Python3_VERSION}")

        list(GET MY_PYTHON3_VERSION_PARTS 0 MY_PYTHON3_MAJOR_VERSION)
        list(GET MY_PYTHON3_VERSION_PARTS 1 MY_PYTHON3_MINOR_VERSION)

        set(BOOST_PYTHON_MODULE_NAME python${MY_PYTHON3_MAJOR_VERSION}${MY_PYTHON3_MINOR_VERSION})
    
        # find packages quiet
        find_package(Boost COMPONENTS ${BOOST_PYTHON_MODULE_NAME} QUIET)
    endif()

    # build python bindings if we have all requirements
    if(Boost_FOUND AND Python3_FOUND)
        set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 1)
    add_subdirectory(python)
    endif()
endif()
