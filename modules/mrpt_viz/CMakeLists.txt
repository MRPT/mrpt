# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_viz LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_poses REQUIRED)
find_package(mrpt_img REQUIRED)

include(./cmake/script_assimp.cmake REQUIRED)      # Check for system assimp lib (3D models)

# Assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
	# ASSIMP_ROOT_DIR - the root directory where the installation can be found
	# ASSIMP_CXX_FLAGS - extra flags for compilation
	# ASSIMP_LINK_FLAGS - extra flags for linking
	# ASSIMP_INCLUDE_DIRS - include directories
	# ASSIMP_LIBRARY_DIRS - link directories
	# ASSIMP_LIBRARIES - libraries to link plugins with
	if (NOT ${ASSIMP_CXX_FLAGS} STREQUAL "")
		add_definitions("${ASSIMP_CXX_FLAGS}")
	endif()
	if (NOT "${ASSIMP_INCLUDE_DIRS}" STREQUAL "")
		include_directories("${ASSIMP_INCLUDE_DIRS}")
	endif()
	if(NOT "${ASSIMP_VERSION}" STREQUAL "")
		string(SUBSTRING "${ASSIMP_VERSION}" 0 1 MRPT_ASSIMP_VERSION_MAJOR)
		add_definitions(-DMRPT_ASSIMP_VERSION_MAJOR=${MRPT_ASSIMP_VERSION_MAJOR})
	endif()
endif ()


# define lib:
set(LIB_SOURCES
  src/CArrow.cpp
  src/CAssimpModel.cpp
  src/CAxis.cpp
  src/CBox.cpp
  src/CCamera.cpp
  src/CColorBar.cpp
  src/CCylinder.cpp
  src/CDisk.cpp
  src/CEllipsoid2D.cpp
  src/CEllipsoid3D.cpp
  src/CEllipsoidInverseDepth2D.cpp
  src/CEllipsoidInverseDepth3D.cpp
  src/CEllipsoidRangeBearing2D.cpp
  src/CFrustum.cpp
  src/CGeneralizedEllipsoidTemplate.cpp
  src/CGridPlaneXY.cpp
  src/CGridPlaneXZ.cpp
  src/CMesh3D.cpp
  src/CMesh.cpp
  src/CMeshFast.cpp
  src/COctoMapVoxels.cpp
  src/CPointCloudColoured.cpp
  src/CPointCloud.cpp
  src/CPolyhedron.cpp
  src/CSetOfLines.cpp
  src/CSetOfObjects.cpp
  src/CSetOfTexturedTriangles.cpp
  src/CSetOfTriangles.cpp
  src/CSimpleLine.cpp
  src/CSkyBox.cpp
  src/CSphere.cpp
  src/CText3D.cpp
  src/CText.cpp
  src/CTextMessageCapable.cpp
  src/CTexturedPlane.cpp
  src/CVectorField2D.cpp
  src/CVectorField3D.cpp
  src/CVisualObject.cpp
  src/PLY_import_export.cpp
  src/pose_pdfs.cpp
  src/registerAllClasses.cpp
  src/Scene.cpp
  src/StockObjects.cpp
  src/TLightParameters.cpp
  src/TTriangle.cpp
  src/Viewport.cpp
  src/Visualizable.cpp
  # tests ----
  tests/bimap_serialize_unittest.cpp
  tests/CFileGZStreams_unittest.cpp
  tests/CSerializable_unittest.cpp
  tests/serializations_unittest.cpp
  tests/stl_serialize_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/viz/opengl_fonts.h
  include/mrpt/viz/CVisualObject.h
  include/mrpt/viz/CBox.h
  include/mrpt/viz/CTextMessageCapable.h
  include/mrpt/viz/CAxis.h
  include/mrpt/viz/pose_pdfs.h
  include/mrpt/viz/COctoMapVoxels.h
  include/mrpt/viz/CVectorField3D.h
  include/mrpt/viz/Scene.h
  include/mrpt/viz/CText3D.h
  include/mrpt/viz/CText.h
  include/mrpt/viz/graph_tools.h
  include/mrpt/viz/registerAllClasses.h
  include/mrpt/viz/CPointCloudColoured.h
  include/mrpt/viz/graph_tools_impl.h
  include/mrpt/viz/CGridPlaneXZ.h
  include/mrpt/viz/CArrow.h
  include/mrpt/viz/CEllipsoid2D.h
  include/mrpt/viz/TLightParameters.h
  include/mrpt/viz/CDisk.h
  include/mrpt/viz/CGeneralizedEllipsoidTemplate.h
  include/mrpt/viz/CSimpleLine.h
  include/mrpt/viz/Visualizable.h
  include/mrpt/viz/CSetOfTriangles.h
  include/mrpt/viz/CEllipsoid3D.h
  include/mrpt/viz/CGridPlaneXY.h
  include/mrpt/viz/CMeshFast.h
  include/mrpt/viz/Viewport.h
  include/mrpt/viz/CSphere.h
  include/mrpt/viz/pointcloud_adapters.h
  include/mrpt/viz/viz_frwds.h
  include/mrpt/viz/CAssimpModel.h
  include/mrpt/viz/TTriangle.h
  include/mrpt/viz/CCamera.h
  include/mrpt/viz/CCylinder.h
  include/mrpt/viz/CPointCloud.h
  include/mrpt/viz/CUBE_TEXTURE_FACE.h
  include/mrpt/viz/PLY_import_export.h
  include/mrpt/viz/CTexturedPlane.h
  include/mrpt/viz/CSetOfLines.h
  include/mrpt/viz/CEllipsoidInverseDepth3D.h
  include/mrpt/viz/stock_objects.h
  include/mrpt/viz/CEllipsoidInverseDepth2D.h
  include/mrpt/viz/CMesh.h
  include/mrpt/viz/CSetOfTexturedTriangles.h
  include/mrpt/viz/CFrustum.h
  include/mrpt/viz/CMesh3D.h
  include/mrpt/viz/CSetOfObjects.h
  include/mrpt/viz/CSkyBox.h
  include/mrpt/viz/CEllipsoidRangeBearing2D.h
  include/mrpt/viz/CColorBar.h
  include/mrpt/viz/CVectorField2D.h
  include/mrpt/viz/CPolyhedron.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_poses
    mrpt::mrpt_img
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_poses
    mrpt_img
)


# Link against assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
  if(TARGET "EP_assimp")
    add_dependencies(${PROJECT_NAME} EP_assimp)
  endif()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${ASSIMP_LIBRARIES})
endif ()

# Python wrapper:
mrpt_add_python_module(viz
  python_bindings/${PROJECT_NAME}_py.cpp
)
