# ===================================================
# nanogui
# ===================================================
set(CMAKE_MRPT_HAS_NANOGUI 0)
set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)

set(BUILD_NANOGUI ON CACHE BOOL "Build an embedded version of nanogui (OpenGL GUIs)")

# TODO: Build from sources for Windows
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW glfw3)

if (BUILD_NANOGUI AND
    ((NOT CMAKE_MRPT_HAS_OPENGL_GLUT) OR (NOT GLFW_FOUND)))
    message(STATUS "Warning: Disabling BUILD_NANOGUI since requirements were not found.")
    set(BUILD_NANOGUI OFF CACHE BOOL "" FORCE)
endif()

if (NOT BUILD_NANOGUI)
return()
endif()

add_subdirectory(3rdparty/nanogui)

set(CMAKE_MRPT_HAS_NANOGUI 1)
set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)
