# Dummy file: only used if using Embedded Eigen3 version

set(EP_eigen3_FOUND TRUE)

# Guard against duplicate target creation: this config file is re-included by
# find_dependency(EP_eigen3) once per consuming project. When several consumers
# are configured in a single CMake session (e.g. the 12 add_subdirectory() apps
# in mrpt_apps_cli), the second inclusion would otherwise fail with
# "add_library cannot create target 'Eigen' because another target ... already
# exists". INTERFACE targets are global, so a NOT TARGET guard makes this
# idempotent.
if(NOT TARGET Eigen)
  add_library(Eigen INTERFACE)
  #target_link_libraries(Eigen INTERFACE mrpt::math)
endif()

# Dummy:
if(NOT TARGET glfw)
  add_library(glfw INTERFACE)
endif()
