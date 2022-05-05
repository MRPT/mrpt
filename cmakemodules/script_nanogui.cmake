# ===================================================
# nanogui
# ===================================================
set(CMAKE_MRPT_HAS_NANOGUI 0)
set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)

if (APPLE)
  set(def_ OFF)
else()
  set(def_ ON)
endif()
set(MRPT_WITH_NANOGUI ${def_} CACHE BOOL "Build an embedded version of nanogui (OpenGL GUIs)")
unset(def_)

find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
  pkg_search_module(GLFW glfw3)
endif()

if (MRPT_WITH_NANOGUI AND
    (NOT CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_EGL))
    message(STATUS "Warning: Disabling MRPT_WITH_NANOGUI since requirements were not found.")
    set(MRPT_WITH_NANOGUI OFF CACHE BOOL "" FORCE)
endif()

if (NOT MRPT_WITH_NANOGUI)
return()
endif()

add_subdirectory(3rdparty/nanogui)

set_target_properties(mrpt-nanogui PROPERTIES FOLDER "3rd party")
if (TARGET glfw)
  set_target_properties(glfw PROPERTIES FOLDER "3rd party")
endif()
if (TARGET glfw_objects)
  set_target_properties(glfw_objects PROPERTIES FOLDER "3rd party")
endif()


set(CMAKE_MRPT_HAS_NANOGUI 1)
set(CMAKE_MRPT_HAS_NANOGUI_SYSTEM 0)
