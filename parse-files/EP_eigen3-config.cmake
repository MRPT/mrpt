# Dummy file: only used if using Embedded Eigen3 version

set(EP_eigen3_FOUND TRUE)

add_library(Eigen INTERFACE)
#target_link_libraries(Eigen INTERFACE mrpt::math)

# Dummy:
add_library(glfw INTERFACE)
