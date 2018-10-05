# Build python bindings?:
# ===================================================

# disabled on start
set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)

# Leave at the user's choice to disable the python bindings:
option(DISABLE_PYTHON_BINDINGS "Disable the build (if possible) of Python bindings" "OFF")
if(DISABLE_PYTHON_BINDINGS)
    set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)
endif(DISABLE_PYTHON_BINDINGS)

if(UNIX AND NOT DISABLE_PYTHON_BINDINGS)
    #set( BUILD_PY_BINDINGS OFF CACHE BOOL "If you want to build the MRPT python bindings, enable this.")
    # find packages quiet
    find_package(Boost QUIET COMPONENTS python)
    find_package(PythonLibs 2.7 QUIET)

    # build python bindings if we have all requirements
    if(Boost_FOUND AND PYTHONLIBS_FOUND)
        set(CMAKE_MRPT_HAS_PYTHON_BINDINGS 1)
        add_subdirectory(python)
    endif()
endif(UNIX AND NOT DISABLE_PYTHON_BINDINGS)
