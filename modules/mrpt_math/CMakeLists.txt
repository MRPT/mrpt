# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_math LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_system REQUIRED)
find_package(mrpt_serialization REQUIRED)
find_package(mrpt_random REQUIRED)

include(./cmake/script_eigen.cmake REQUIRED)       # Eigen3
include(./cmake/script_nanoflann.cmake REQUIRED)   # Check for nanoflann

# eigen3:
if (MRPT_EIGEN_USE_EMBEDDED_VERSION)
	set(eigen_dep EP_eigen3)
else()
	set(eigen_dep Eigen3) # find_package() lib name
endif()

# define lib:
set(LIB_SRCS
  src/A_MatrixBase_instantiate_CMatrixDynamic_d.cpp
  src/A_MatrixBase_instantiate_CMatrixDynamic_f.cpp
  src/CAtan2LookUpTable.cpp
  src/CAtan2LookUpTable_unittest.cpp
  src/CHistogram.cpp
  src/CLevenbergMarquardt_unittest.cpp
  src/CMatrixB.cpp
  src/CMatrixD.cpp
  src/CMatrixDynamic.cpp
  src/CMatrixDynamic_unittest.cpp
  src/CMatrixF.cpp
  src/CMatrixFixed.cpp
  src/CMatrixFixed_unittest.cpp
  src/container_ops_unittest.cpp
  src/CPolygon.cpp
  src/CSparseMatrix.cpp
  src/CSparseMatrix_unittest.cpp
  src/CSplineInterpolator1D.cpp
  src/CVectorDynamic.cpp
  src/CVectorFixed.cpp
  src/distributions_unittest.cpp
  src/EigenAlignment_unittest.cpp
  src/filters.cpp
  src/fourier.cpp
  src/fresnel.cpp
  src/fresnel_unittest.cpp
  src/geometry.cpp
  src/geometry_unittest.cpp
  src/KDTreeCapable_unittest.cpp
  src/kmeans.cpp
  src/kmeans++/KMeans.cpp
  src/kmeans++/KmTree.cpp
  src/kmeans++/KmUtils.cpp
  src/math.cpp
  src/MatrixBase_instantiate_CMatrixFixed_22_d.cpp
  src/MatrixBase_instantiate_CMatrixFixed_22_f.cpp
  src/MatrixBase_instantiate_CMatrixFixed_33_d.cpp
  src/MatrixBase_instantiate_CMatrixFixed_33_f.cpp
  src/matrix_ops1_unittest.cpp
  src/matrix_ops2_unittest.cpp
  src/matrix_ops3_unittest.cpp
  src/matrix_ops4_unittest.cpp
  src/matrix_ops5_unittest.cpp
  src/matrix_ops_unittest.cpp
  src/MatrixVectorBase_instantiate_CMatrixDynamic_d.cpp
  src/MatrixVectorBase_instantiate_CMatrixDynamic_f.cpp
  src/MatrixVectorBase_instantiate_CMatrixDynamic_ints.cpp
  src/MatrixVectorBase_instantiate_CMatrixFixed.cpp
  src/MatrixVectorBase_instantiate_CVectorDynamic.cpp
  src/MatrixVectorBase_instantiate_CVectorFixed.cpp
  src/matrix_yaml_unittest.cpp
  src/model_search.cpp
  src/poly_roots.cpp
  src/poly_roots_unittest.cpp
  src/ransac_applications.cpp
  src/registerAllClasses.cpp
  src/robust_kernels_unittest.cpp
  src/slerp.cpp
  src/slerp_unittest.cpp
  src/S_MatrixBase_instantiate_CMatrixFixed_66_d.cpp
  src/S_MatrixBase_instantiate_CMatrixFixed_66_f.cpp
  src/S_MatrixBase_instantiate_CMatrixFixed_77_d.cpp
  src/TBoundingBox.cpp
  src/TBoundingBox_unittest.cpp
  src/TLine2D.cpp
  src/TLine3D.cpp
  src/TObject2D.cpp
  src/TObject3D.cpp
  src/TPlane.cpp
  src/TPoint2D.cpp
  src/TPoint3D.cpp
  src/TPolygon2D.cpp
  src/TPolygon3D.cpp
  src/TPolygonWithPlane.cpp
  src/TPose2D.cpp
  src/TPose3D.cpp
  src/TPose3DQuat.cpp
  src/TPoseOrPoint_unittest.cpp
  src/TSegment2D.cpp
  src/TSegment3D.cpp
  src/Twist2D.cpp
  src/Twist3D.cpp
  src/wrap2pi_unittest.cpp
  src/Z_MatrixBase_instantiate_CMatrixFixed_44_d.cpp
  src/Z_MatrixBase_instantiate_CMatrixFixed_44_f.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/3rdparty/CSparse/cs.h
  include/mrpt/math/CAtan2LookUpTable.h
  include/mrpt/math/CBinaryRelation.h
  include/mrpt/math/CHistogram.h
  include/mrpt/math/CLevenbergMarquardt.h
  include/mrpt/math/CMatrixB.h
  include/mrpt/math/CMatrixD.h
  include/mrpt/math/CMatrixDynamic.h
  include/mrpt/math/CMatrixF.h
  include/mrpt/math/CMatrixFixed.h
  include/mrpt/math/CMonteCarlo.h
  include/mrpt/math/CPolygon.h
  include/mrpt/math/CProbabilityDensityFunction.h
  include/mrpt/math/CQuaternion.h
  include/mrpt/math/CSparseMatrixTemplate.h
  include/mrpt/math/CSplineInterpolator1D.h
  include/mrpt/math/CVectorDynamic.h
  include/mrpt/math/CVectorFixed.h
  include/mrpt/math/data_utils.h
  include/mrpt/math/distributions.h
  include/mrpt/math/eigen_extensions.h
  include/mrpt/math/epsilon.h
  include/mrpt/math/filters.h
  include/mrpt/math/fourier.h
  include/mrpt/math/fresnel.h
  include/mrpt/math/geometry.h
  include/mrpt/math/gtsam_wrappers.h
  include/mrpt/math/homog_matrices.h
  include/mrpt/math/interp_fit.h
  include/mrpt/math/KDTreeCapable.h
  include/mrpt/math/kmeans.h
  include/mrpt/math/mat2eig.h
  include/mrpt/math/math_frwds.h
  include/mrpt/math/matrix_adaptors.h
  include/mrpt/math/MatrixBase.h
  include/mrpt/math/MatrixBlockSparseCols.h
  include/mrpt/math/matrix_serialization.h
  include/mrpt/math/matrix_size_t.h
  include/mrpt/math/MatrixVectorBase.h
  include/mrpt/math/model_search.h
  include/mrpt/math/model_search_impl.h
  include/mrpt/math/num_jacobian.h
  include/mrpt/math/ops_containers.h
  include/mrpt/math/ops_matrices.h
  include/mrpt/math/ops_vectors.h
  include/mrpt/math/point_poses2vectors.h
  include/mrpt/math/poly_roots.h
  include/mrpt/math/ransac_applications.h
  include/mrpt/math/ransac.h
  include/mrpt/math/ransac_impl.h
  include/mrpt/math/registerAllClasses.h
  include/mrpt/math/robust_kernels.h
  include/mrpt/math/slerp.h
  include/mrpt/math/TBoundingBox.h
  include/mrpt/math/TLine2D.h
  include/mrpt/math/TLine3D.h
  include/mrpt/math/TObject2D.h
  include/mrpt/math/TObject3D.h
  include/mrpt/math/TPlane.h
  include/mrpt/math/TPoint2D.h
  include/mrpt/math/TPoint3D.h
  include/mrpt/math/TPolygon2D.h
  include/mrpt/math/TPolygon3D.h
  include/mrpt/math/TPolygonWithPlane.h
  include/mrpt/math/TPose2D.h
  include/mrpt/math/TPose3D.h
  include/mrpt/math/TPose3DQuat.h
  include/mrpt/math/TPoseOrPoint.h
  include/mrpt/math/transform_gaussian.h
  include/mrpt/math/TSegment2D.h
  include/mrpt/math/TSegment3D.h
  include/mrpt/math/TTwist2D.h
  include/mrpt/math/TTwist3D.h
  include/mrpt/math/utils.h
  include/mrpt/math/utils_matlab.h
  include/mrpt/math/wrap2pi.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_system
    mrpt::mrpt_serialization
    mrpt::mrpt_random
    Eigen3::Eigen
    nanoflann::nanoflann
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_system
    mrpt_serialization
    mrpt_random
  # Other imported targets:
    nanoflann # find_package() lib name
    ${eigen_dep}
)

# extra dependencies required by unit tests in this module:
#set_property(GLOBAL PROPERTY mrpt_math_UNIT_TEST_EXTRA_DEPS mrpt-io)


# Ignore precompiled headers in some sources:
#if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
#  set_source_files_properties(
#    ${math/CSparse_FILES}
#    ${math/kmeans_FILES}
#    PROPERTIES COMPILE_FLAGS "/Y-"
#    )
#endif()

# Still needed?
#if (MSVC)
#  # This is to prevent the error (in Debug builds):
#  # LNK1189：library limit of 65535 objects exceeded
#  target_compile_options(math PRIVATE /Ob2)
#endif()
