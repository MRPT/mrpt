# Build python bindings?:
# ===================================================

# disabled on start
SET(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)

# Leave at the user's choice to disable the python bindings:
OPTION(DISABLE_PYTHON_BINDINGS "Disable the build (if possible) of Python bindings" "OFF")
IF(DISABLE_PYTHON_BINDINGS)
    SET(CMAKE_MRPT_HAS_PYTHON_BINDINGS 0)
ENDIF(DISABLE_PYTHON_BINDINGS)

IF(UNIX AND NOT DISABLE_PYTHON_BINDINGS)
    #SET( BUILD_PY_BINDINGS OFF CACHE BOOL "If you want to build the MRPT python bindings, enable this.")
    # find packages quiet
    FIND_PACKAGE(Boost QUIET COMPONENTS python)
    FIND_PACKAGE(PythonLibs 2.7 QUIET)

    # build python bindings if we have all requirements
    IF(Boost_FOUND AND PYTHONLIBS_FOUND)
        SET(CMAKE_MRPT_HAS_PYTHON_BINDINGS 1)
        ADD_SUBDIRECTORY(python)
    ENDIF()
ENDIF(UNIX AND NOT DISABLE_PYTHON_BINDINGS)
