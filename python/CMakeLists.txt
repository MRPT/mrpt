project(pymrpt)

# BINDER
# Wrapper generated with binder git commit 95938b8a6d7afd564ee5f0d3430964bd8ea3df1a
#

#find_package(PythonInterp)

# Note: relative paths are relative to ${CMAKE_INSTALL_PREFIX} (defaults to /usr/local on Ubuntu)
#
# According to debian policy https://debian-python.readthedocs.io/en/latest/debian-policy.html#module-path
# this should be: /usr/lib/python3/dist-packages
#
#set(PYMRPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/")
#install(TARGETS pymrpt LIBRARY DESTINATION ${PYMRPT_INSTALL_DIR})

if (NOT CMAKE_MRPT_HAS_PYTHON_BINDINGS)
  return()
endif()

set(PY_SRCS_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

# these "all" files can be generated like:
#  ls -1 src/nanogui/*.cpp | xargs -I FIL echo "#include \"FIL\"" > all_nanogui.cpp

#file(GLOB_RECURSE PY_ALL_SRCS ${PY_SRCS_DIR}/all_*.cpp)
file(GLOB_RECURSE PY_ALL_SRCS all_*.cpp)

pybind11_add_module(pymrpt
  ${PY_SRCS_DIR}/pymrpt.cpp
  ${PY_ALL_SRCS}
  # -- manually crafted files --
  pymrpt_internals.cpp
  )

# Remove LTO from pybind flags:
option(MRPT_PYBIND11_WITH_LTO "If disabled, MRPT will remove the LTO build flags added by pybind for pymrpt. Default is ON, so enabling LTO" "ON")
if (NOT MRPT_PYBIND11_WITH_LTO)
  get_target_property(_opts pymrpt LINK_LIBRARIES)
  string(REPLACE "pybind11::lto" "" _opts "${_opts}")
  string(REPLACE "pybind11::thin_lto" "" _opts "${_opts}")
  set_target_properties(pymrpt PROPERTIES LINK_LIBRARIES ${_opts})
else()
  # Add parallel lto:
  set(MRPT_PYTHON_LTO_PARALLEL_JOBS "3" CACHE STRING "Number of parallel threads for link-time -flto in pymrpt. Setting it to a large number may lead to RAM swapping. Default=3. Set to 'auto' for automatic core detection.")
  target_link_options(pymrpt PRIVATE -flto=${MRPT_PYTHON_LTO_PARALLEL_JOBS})
endif()

# enable separate build of pymrpt for ros packages:
if(NOT TARGET mrpt::apps AND NOT TARGET mrpt-apps)
  find_package(mrpt-apps REQUIRED)
endif()
if(NOT TARGET mrpt::nav AND NOT TARGET mrpt-nav)
  find_package(mrpt-nav REQUIRED)
endif()
if(NOT TARGET mrpt::slam AND NOT TARGET mrpt-slam)
  find_package(mrpt-slam REQUIRED)
endif()
if(NOT TARGET mrpt::gui AND NOT TARGET mrpt-gui)
  find_package(mrpt-gui REQUIRED)
endif()

# Link:
target_link_libraries(pymrpt PUBLIC 
  mrpt::apps
  mrpt::slam
  mrpt::gui
  mrpt::nav
)

target_include_directories(pymrpt PRIVATE ".")

#target_compile_definitions(pymrpt PRIVATE 
#  PYBIND11_MAJOR_VERSION=${PYBIND11_MAJOR_VERSION}
#  PYBIND11_MINOR_VERSION=${PYBIND11_MINOR_VERSION}
#  PYBIND11_PATCH_VERSION=${PYBIND11_PATCH_VERSION}
#  )

# Fix for pybind 2.4.x:
# override_caster_t  was overload_caster_t
if ("${PYBIND11_MAJOR_VERSION}.${PYBIND11_MINOR_VERSION}" VERSION_LESS "2.9")
  target_compile_definitions(pymrpt PRIVATE override_caster_t=overload_caster_t)
endif()

target_compile_definitions(pymrpt PRIVATE MRPT_BUILDING_PYTHON_WRAPPER)

# Python files:
file(MAKE_DIRECTORY ${MRPT_PYTHON_BUILD_DIRECTORY}/mrpt/)
add_custom_command(TARGET pymrpt POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pymrpt>
  ${MRPT_PYTHON_BUILD_DIRECTORY}/mrpt/
)
file(WRITE ${MRPT_PYTHON_BUILD_DIRECTORY}/mrpt/__init__.py
  "from . import *")

# Required if we provide the stub files: ( https://peps.python.org/pep-0561/ )
file(WRITE ${MRPT_PYTHON_BUILD_DIRECTORY}/mrpt/py.typed "")

# install stub files too:
file(COPY
  ${CMAKE_CURRENT_LIST_DIR}/stubs-out/
  DESTINATION ${MRPT_PYTHON_BUILD_DIRECTORY}/
)

# copy ROS stuff
file(COPY
  ${CMAKE_CURRENT_LIST_DIR}/ros_bridge.py
  DESTINATION ${MRPT_PYTHON_BUILD_DIRECTORY}/mrpt/
)

# Generate setup.py
configure_file(${MRPT_SOURCE_DIR}/parse-files/setup.py.in
  ${MRPT_PYTHON_BUILD_DIRECTORY}/setup.py)

if (NOT DEFINED PYTHON_INSTALL_DIRECTORY)
  set(PYTHON_INSTALL_DIRECTORY ${CMAKE_INSTALL_PREFIX} CACHE PATH "Install prefix for python modules with 'make python-install'")
endif()

mrpt_ament_cmake_python_get_python_install_dir() # Gets PYTHON_INSTALL_DIR

# We need to do the install via a script run during "make install" so we can 
# catch the variable DESTDIR, used in debian builds:
configure_file(
  ${MRPT_SOURCE_DIR}/parse-files/install-python.cmake.in
  ${MRPT_BINARY_DIR}/install-python.cmake
  @ONLY
)

add_custom_target(python-install
    COMMAND ${CMAKE_COMMAND} -P ${MRPT_BINARY_DIR}/install-python.cmake
    COMMENT "Running 'python-install'...")

add_dependencies(python-install pymrpt)

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build \"${MRPT_BINARY_DIR}\" --target python-install)")

# -------------------------------------
# Special handle for ROS builds
# -------------------------------------
# See: https://github.com/ros-infrastructure/rosdep/issues/724#issuecomment-643015997


