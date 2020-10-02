# =========================================================================
#  The Mobile Robot Programming Toolkit (MRPT) CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Please, read:
#  https://docs.mrpt.org/reference/devel/mrpt_from_cmake.html

#   In your CMakeLists.txt, add these lines:
#
#    find_package(MRPT REQUIRED
#       COMPONENTS slam nav
#       OPTIONAL_COMPONENTS  vision)
#    target_link_libraries(MY_TARGET_NAME ${MRPT_LIBRARIES})
#
#   or do it individually:
#    find_package(mrpt-slam)
#    find_package(mrpt-nav)
#    target_link_libraries(MY_TARGET_NAME mrpt::slam mrpt::nav)
# =========================================================================
include(CMakeFindDependencyMacro)

set(MRPT_LIBRARIES "")
foreach(_comp ${MRPT_FIND_COMPONENTS})
  # If not set, use current directory as guess:
  if (NOT mrpt-${_comp}_DIR)
    set(mrpt-${_comp}_DIR  ${CMAKE_CURRENT_LIST_DIR})
  endif()

  if (MRPT_FIND_REQUIRED_${_comp})
    find_dependency(mrpt-${_comp})
  else()
    find_package(mrpt-${_comp} CONFIG QUIET)
  endif()
  if (mrpt-${_comp}_FOUND)
    list(APPEND MRPT_LIBRARIES mrpt::${_comp})
  endif()
endforeach()

# backwards-compatibility var name:
set(MRPT_LIBS ${MRPT_LIBRARIES})
