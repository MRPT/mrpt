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
    #set( BUILD_PY_BINDINGS OFF CACHE BOOL "If you want to build the MRPT python bindings, enable this.")
    # find packages quiet
    find_package(Boost QUIET COMPONENTS python3)

	# Requires CMake 3.13+
	find_package (Python3 COMPONENTS Development QUIET)

    # build python bindings if we have all requirements
	if(Boost_FOUND AND Python3_FOUND)
        set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 1)
        add_subdirectory(python)
    endif()
endif()
