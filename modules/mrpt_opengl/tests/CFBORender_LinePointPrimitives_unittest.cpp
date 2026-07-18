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

// Off-screen (FBO) render regression test that puts together, in a single
// scene, every line/point/text/vector-field mrpt::viz drawing primitive not
// already exercised by CFBORender_unittest.cpp or
// CFBORender_SolidPrimitives_unittest.cpp, so their render() paths get unit
// test coverage. The rendered RGB frame is compared against a small,
// low-resolution ground-truth PNG checked into the repo.
//
// Note: CSkyBox is intentionally NOT exercised here. It was found to render
// non-deterministically (visible in some process runs, silently skipped in
// others) when driven through CFBORender, seemingly a texture upload/first-
// draw synchronization issue rather than anything in this test; it deserves
// a dedicated investigation rather than a flaky regression test.

#include <gtest/gtest.h>

// mrpt::opengl (rendering)
#include <mrpt/opengl/CFBORender.h>

// mrpt::viz (scene graph)
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/Scene.h>
#include <test_mrpt_common.h>

#include <Eigen/Dense>
#include <cmath>

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

void test_opengl_linePointPrimitives()
{
  using namespace mrpt;       // _deg
  using namespace mrpt::viz;  // Scene graph classes

  const std::string expected_RGB_img_file =
      UNITTEST_BASEDIR() + "/tests/CFBORender_expected_linePointPrimitives.png";

  auto scene = mrpt::viz::Scene::Create();

  // Row 1 (y=0): CArrow, CAxis, CColorBar, CEllipsoid2D
  {
    auto obj = mrpt::viz::CArrow::Create(
        mrpt::math::TPoint3Df(0, 0, 0), mrpt::math::TPoint3Df(0, 0, 1.2f), 0.25f, 0.05f, 0.12f);
    obj->setColor(1.0f, 0.3f, 0.3f);
    obj->setLocation(-6, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CAxis::Create(-1, -1, -1, 1, 1, 1, 1.0f, 2.0f, true);
    obj->setColor(0.9f, 0.9f, 0.9f);
    obj->setLocation(-3, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CColorBar::Create(
        mrpt::img::cmJET, 0.3, 1.2, 0.0f, 1.0f, 0.0f, 1.0f, "%4.1f", 0.08f);
    obj->setLocation(0, 0, -0.5);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CEllipsoid2D::Create();
    mrpt::math::CMatrixDouble22 cov;
    cov.setIdentity();
    cov(0, 0) = 0.6 * 0.6;
    cov(1, 1) = 0.3 * 0.3;
    obj->setCovMatrix(cov);
    obj->setQuantiles(2.5f);
    obj->set2DsegmentsCount(24);
    obj->setColor(0.2f, 1.0f, 0.8f);
    obj->setLocation(3, 0, 0);
    scene->insert(obj);
  }

  // Row 2 (y=3): CPointCloud, CPointCloudColoured, CSetOfLines, CSimpleLine
  {
    auto obj = mrpt::viz::CPointCloud::Create();
    obj->setPointSize(4.0f);
    obj->setColor(1.0f, 1.0f, 1.0f);
    for (int i = 0; i < 20; i++)
    {
      const float ang = static_cast<float>(i) * 0.31415f;
      obj->insertPoint(0.6f * std::cos(ang), 0.6f * std::sin(ang), 0.05f * static_cast<float>(i));
    }
    obj->setLocation(-6, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CPointCloudColoured::Create();
    obj->setPointSize(4.0f);
    for (int i = 0; i < 20; i++)
    {
      const float ang = static_cast<float>(i) * 0.31415f;
      obj->push_back(
          0.6f * std::cos(ang), 0.6f * std::sin(ang), 0.05f * static_cast<float>(i),
          static_cast<float>(i) / 20.0f, 1.0f - static_cast<float>(i) / 20.0f, 0.5f);
    }
    obj->setLocation(-3, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSetOfLines::Create();
    obj->setColor(1.0f, 0.8f, 0.0f);
    obj->setLineWidth(2.0f);
    obj->appendLine(-0.6, -0.6, 0, 0.6, -0.6, 0.8);
    obj->appendLine(0.6, -0.6, 0.8, 0, 0.6, 0);
    obj->appendLine(0, 0.6, 0, -0.6, -0.6, 0);
    obj->setLocation(0, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSimpleLine::Create(-0.6f, 0, 0, 0.6f, 0, 1.0f, 3.0f);
    obj->setColor(0.2f, 0.6f, 1.0f);
    obj->setLocation(3, 3, 0);
    scene->insert(obj);
  }

  // Row 3 (y=-3): CText, CVectorField2D, CVectorField3D
  {
    auto obj = mrpt::viz::CText::Create("MRPT");
    obj->setColor(1.0f, 1.0f, 1.0f);
    obj->setLocation(-5, -3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CVectorField2D::Create();
    mrpt::math::CMatrixFloat vx(3, 3), vy(3, 3);
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        vx(r, c) = 0.15f * static_cast<float>(c - 1);
        vy(r, c) = 0.15f * static_cast<float>(r - 1);
      }
    }
    obj->setVectorField(vx, vy);
    obj->setGridCenterAndCellSize(0, 0, 0.4f, 0.4f);
    obj->setPointColor(1.0f, 1.0f, 0.0f);
    obj->setVectorFieldColor(0.0f, 1.0f, 1.0f);
    obj->setLocation(-1, -3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CVectorField3D::Create();
    mrpt::math::CMatrixFloat vx(2, 2), vy(2, 2), vz(2, 2), px(2, 2), py(2, 2), pz(2, 2);
    for (int r = 0; r < 2; r++)
    {
      for (int c = 0; c < 2; c++)
      {
        px(r, c) = 0.4f * static_cast<float>(c);
        py(r, c) = 0.4f * static_cast<float>(r);
        pz(r, c) = 0;
        vx(r, c) = 0.1f;
        vy(r, c) = 0.1f;
        vz(r, c) = 0.2f;
      }
    }
    obj->setPointCoordinates(px, py, pz);
    obj->setVectorField(vx, vy, vz);
    obj->setColor(1.0f, 0.4f, 0.8f);
    obj->setLocation(2, -3, 0);
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
      mrpt::poses::CPose3D(-1.5, 0, 6.5, 0.0_deg /*yaw*/, 90.0_deg /*pitch*/, 0.0_deg /*roll*/);
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
TEST(OpenGL, CFBORender_linePointPrimitives)
#else
TEST(OpenGL, DISABLED_CFBORender_linePointPrimitives)
#endif
{
  try
  {
    test_opengl_linePointPrimitives();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}
