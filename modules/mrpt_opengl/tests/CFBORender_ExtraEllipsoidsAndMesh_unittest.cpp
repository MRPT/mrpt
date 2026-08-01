/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// Off-screen (FBO) render regression test that exercises the remaining
// mrpt::viz drawing primitives not already covered by CFBORender_unittest.cpp,
// CFBORender_SolidPrimitives_unittest.cpp, or
// CFBORender_LinePointPrimitives_unittest.cpp: the inverse-depth/range-bearing
// ellipsoid parameterizations, CGridPlaneXZ, and CMesh3D. The rendered RGB
// frame is compared against a small, low-resolution ground-truth PNG checked
// into the repo.
//
// Note: CSkyBox is intentionally NOT exercised here either, for the same
// flakiness reason documented in CFBORender_LinePointPrimitives_unittest.cpp.

#include <gtest/gtest.h>

// mrpt::opengl (rendering)
#include <mrpt/opengl/CFBORender.h>

// mrpt::viz (scene graph)
#include <mrpt/core/bits_math.h>
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/Scene.h>
#include <test_mrpt_common.h>

#include <Eigen/Dense>

#if MRPT_HAS_OPENGL && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

// See CFBORender_unittest.cpp for the rationale of these arch exclusions:
#if defined(__mips__) || defined(__mips)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
#if defined(__powerpc) || defined(__powerpc__) || defined(__powerpc64__) ||               \
    defined(__POWERPC__) || defined(__ppc__) || defined(__PPC__) || defined(_ARCH_PPC) || \
    defined(__PPC64__) || defined(__ppc64__) || defined(_ARCH_PPC64)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
#if defined(__arm__) && MRPT_WORD_SIZE == 32
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
#if defined(__riscv)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
#if defined(__loongarch__)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif

namespace
{
float imageDiff(const mrpt::img::CImage& im1, const mrpt::img::CImage& im2)
{
  const auto [r1, g1, b1] = im1.getAsRGBMatricesFloat();
  const auto [r2, g2, b2] = im2.getAsRGBMatricesFloat();

  return (r1 - r2).asEigen().array().abs().sum() + (g1 - g2).asEigen().array().abs().sum() +
         (b1 - b2).asEigen().array().abs().sum();
}

void test_opengl_extraEllipsoidsAndMesh()
{
  using namespace mrpt;       // _deg
  using namespace mrpt::viz;  // Scene graph classes

  const std::string expected_RGB_img_file =
      UNITTEST_BASEDIR() + "/tests/CFBORender_expected_extraEllipsoidsAndMesh.png";

  auto scene = mrpt::viz::Scene::Create();

  // Background grid, on the XZ plane, behind the row of objects:
  {
    auto obj = mrpt::viz::CGridPlaneXZ::Create(-6, 6, -1, 4, -3, 2);
    obj->setColor(0.4f, 0.4f, 0.4f);
    scene->insert(obj);
  }

  // Row 1 (y=0): CEllipsoidInverseDepth2D, CEllipsoidInverseDepth3D,
  // CEllipsoidRangeBearing2D. Parameterizations follow the same
  // (rho=1/range, angle) convention used in mrpt_examples_cpp's
  // opengl_objects_demo, since a zero-mean covariance is a degenerate,
  // effectively invisible input for these parameter spaces.
  {
    auto obj = mrpt::viz::CEllipsoidInverseDepth2D::Create();
    const double max_dist = 3.0;
    const double min_dist = 1.0;
    const double rho_mean = 0.5 * (1.0 / min_dist + 1.0 / max_dist);
    const double rho_std = (1.0 / 6.0) * (1.0 / min_dist - 1.0 / max_dist);
    mrpt::math::CMatrixFixed<double, 2, 2> cov;
    cov.setZero();
    cov(0, 0) = rho_std * rho_std;
    cov(1, 1) = mrpt::square(2.0_deg);
    mrpt::math::CMatrixFixed<double, 2, 1> mean;
    mean(0, 0) = rho_mean;
    mean(1, 0) = 90.0_deg;
    obj->setCovMatrixAndMean(cov, mean);
    obj->setQuantiles(2.0f);
    obj->setNumberOfSegments(60);
    obj->setUnderflowMaxRange(10.0);
    obj->setColor(1.0f, 0.4f, 0.2f);
    obj->setLocation(-3, 0, -1.5);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CEllipsoidInverseDepth3D::Create();
    const double max_dist = 3.0;
    const double min_dist = 1.0;
    const double rho_mean = 0.5 * (1.0 / min_dist + 1.0 / max_dist);
    const double rho_std = (1.0 / 6.0) * (1.0 / min_dist - 1.0 / max_dist);
    mrpt::math::CMatrixFixed<double, 3, 3> cov;
    cov.setZero();
    cov(0, 0) = rho_std * rho_std;
    cov(1, 1) = mrpt::square(2.0_deg);
    cov(2, 2) = mrpt::square(2.0_deg);
    mrpt::math::CMatrixFixed<double, 3, 1> mean;
    mean(0, 0) = rho_mean;
    mean(1, 0) = 90.0_deg;
    mean(2, 0) = 0.0_deg;
    obj->setCovMatrixAndMean(cov, mean);
    obj->setQuantiles(2.0f);
    obj->setUnderflowMaxRange(10.0f);
    obj->setColor(0.2f, 1.0f, 0.4f);
    obj->setLocation(0, 0, -1.5);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CEllipsoidRangeBearing2D::Create();
    mrpt::math::CMatrixFixed<double, 2, 2> cov;
    cov.setZero();
    cov(0, 0) = 0.05;
    cov(1, 1) = 0.02;
    mrpt::math::CMatrixFixed<double, 2, 1> mean;
    mean(0, 0) = 1.5;
    mean(1, 0) = 0.0;
    obj->setCovMatrixAndMean(cov, mean);
    obj->setQuantiles(2.0f);
    obj->setColor(0.3f, 0.6f, 1.0f);
    obj->setLocation(3, 0, -1.5);
    scene->insert(obj);
  }

  // Row 2 (y=3): CMesh3D (a single quad and a single triangle)
  {
    auto obj = mrpt::viz::CMesh3D::Create();
    obj->enableShowEdges(true);
    obj->enableShowFaces(true);
    obj->enableShowVertices(true);
    obj->enableFaceNormals(true);
    obj->setEdgeColor(0.f, 0.f, 0.f);
    obj->setFaceColor(0.8f, 0.6f, 0.1f);
    obj->setVertColor(1.f, 1.f, 1.f);

    // clang-format off
    float vert_coords[] = {
        -0.6f, -0.6f, 0.0f,
         0.6f, -0.6f, 0.0f,
         0.6f,  0.6f, 0.5f,
        -0.6f,  0.6f, 0.5f,
         0.0f,  1.2f, 1.0f,
    };
    int verts_per_face[] = {4, 3};
    int face_verts[] = {
        0, 1, 2, 3,  // quad
        2, 3, 4,     // triangle
    };
    // clang-format on
    obj->loadMesh(5, 2, verts_per_face, face_verts, vert_coords);
    obj->setLocation(-2, 3, 0);
    scene->insert(obj);
  }

  int width = 320;
  int height = 200;
  const float camera_fov_deg = 90.0f;

  mrpt::opengl::CFBORender renderer(width, height);
  mrpt::img::CImage frame(width, height, mrpt::img::CH_RGB);
  mrpt::math::CMatrixFloat depth;

  auto viewport = scene->getViewport();
  viewport->setCustomBackgroundColor({0.25f, 0.25f, 0.25f, 1.0f});
  viewport->setViewportClipDistances(0.1f, 30.0f);

  mrpt::viz::CCamera camera;
  camera.setProjectiveFOVdeg(camera_fov_deg);
  camera.set6DOFMode(true);

  auto robotPose =
      mrpt::poses::CPose3D(-1, 0, 6.5, 0.0_deg /*yaw*/, 90.0_deg /*pitch*/, 0.0_deg /*roll*/);
  auto camPose = robotPose + mrpt::poses::CPose3D::FromYawPitchRoll(
                                 90.0_deg /*yaw*/, 0.0_deg /*pitch*/, 90.0_deg /*roll*/);
  camera.setPose(camPose);
  renderer.setCamera(camera);

  renderer.render_RGBD(*scene, frame, depth);

  mrpt::img::CImage gt_frame;
  bool readOk_rgb = gt_frame.loadFromFile(expected_RGB_img_file);
  EXPECT_TRUE(readOk_rgb);

  const float rgb_diff = imageDiff(gt_frame, frame);
  std::cout << "rgb_diff=" << rgb_diff << "\n";
  EXPECT_LT(rgb_diff, 5000.0f);
}

}  // namespace

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, CFBORender_extraEllipsoidsAndMesh)
#else
TEST(OpenGL, DISABLED_CFBORender_extraEllipsoidsAndMesh)
#endif
{
  try
  {
    test_opengl_extraEllipsoidsAndMesh();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}
