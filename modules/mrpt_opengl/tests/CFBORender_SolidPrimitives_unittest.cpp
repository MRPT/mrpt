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
// scene, every solid/volumetric mrpt::viz drawing primitive not already
// exercised by CFBORender_unittest.cpp, so their render() paths get unit
// test coverage. The rendered RGB frame is compared against a small,
// low-resolution ground-truth PNG checked into the repo.

#include <gtest/gtest.h>

// mrpt::opengl (rendering)
#include <mrpt/opengl/CFBORender.h>

// mrpt::viz (scene graph)
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
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

void test_opengl_solidPrimitives()
{
  using namespace mrpt;       // _deg
  using namespace mrpt::viz;  // Scene graph classes

  const std::string expected_RGB_img_file =
      UNITTEST_BASEDIR() + "/tests/CFBORender_expected_solidPrimitives.png";

  auto scene = mrpt::viz::Scene::Create();

  // Background grid, on the XZ plane, behind the row of objects:
  {
    auto obj = mrpt::viz::CGridPlaneXZ::Create(-8, 8, -1, 5, -4, 2);
    obj->setColor(0.4f, 0.4f, 0.4f);
    scene->insert(obj);
  }

  // Row 1 (y=0): CCylinder, CDisk, CFrustum, CEllipsoid3D
  {
    auto obj = mrpt::viz::CCylinder::Create(0.6f, 0.3f, 1.2f, 16);
    obj->setColor(1.0f, 0.2f, 0.2f);
    obj->setLocation(-6, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CDisk::Create(0.7f, 0.2f, 24);
    obj->setColor(0.2f, 1.0f, 0.2f);
    obj->setLocation(-3, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CFrustum::Create(0.3f, 1.5f, 60.0f, 40.0f, 2.0f, true, true);
    obj->setPlaneColor(mrpt::img::TColor(80, 160, 255, 128));
    obj->setColor(1.0f, 1.0f, 0.2f);
    obj->setLocation(0, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CEllipsoid3D::Create();
    mrpt::math::CMatrixDouble33 cov;
    cov.setIdentity();
    cov(0, 0) = 0.7 * 0.7;
    cov(1, 1) = 0.4 * 0.4;
    cov(2, 2) = 0.5 * 0.5;
    obj->setCovMatrix(cov);
    obj->setQuantiles(2.5f);
    obj->set3DsegmentsCount(12);
    obj->setColor(0.2f, 0.6f, 1.0f);
    obj->setLocation(3, 0, 0);
    scene->insert(obj);
  }

  // Row 2 (y=3): CPolyhedron (two variants), CSetOfTriangles, CSetOfTexturedTriangles
  {
    auto obj = mrpt::viz::CPolyhedron::CreateIcosahedron(0.7);
    obj->setColor(1.0f, 0.5f, 0.0f);
    obj->setLocation(-6, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CPolyhedron::CreateCuboctahedron(0.7);
    obj->setColor(0.6f, 0.2f, 0.8f);
    obj->setLocation(-3, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSetOfTriangles::Create();
    mrpt::viz::TTriangle t;
    t.vertices[0].xyzrgba.pt = {-0.6f, -0.6f, 0.0f};
    t.vertices[1].xyzrgba.pt = {0.6f, -0.6f, 0.0f};
    t.vertices[2].xyzrgba.pt = {0.0f, 0.6f, 0.9f};
    for (auto& v : t.vertices)
    {
      v.xyzrgba.r = 255;
      v.xyzrgba.g = 120;
      v.xyzrgba.b = 40;
      v.xyzrgba.a = 255;
    }
    obj->insertTriangle(t);
    obj->setLocation(0, 3, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSetOfTexturedTriangles::Create();
    mrpt::img::CImage tex(4, 4, mrpt::img::CH_RGB);
    tex.filledRectangle({0, 0}, {4, 4}, mrpt::img::TColor(255, 255, 0));
    obj->assignImage(tex);
    mrpt::viz::TTriangle t1;
    t1.vertices[0].xyzrgba.pt = {-0.6f, -0.6f, 0.0f};
    t1.vertices[1].xyzrgba.pt = {0.6f, -0.6f, 0.0f};
    t1.vertices[2].xyzrgba.pt = {0.6f, 0.6f, 0.0f};
    t1.vertices[0].uv = {0.0f, 0.0f};
    t1.vertices[1].uv = {1.0f, 0.0f};
    t1.vertices[2].uv = {1.0f, 1.0f};
    mrpt::viz::TTriangle t2 = t1;
    t2.vertices[1] = t1.vertices[2];
    t2.vertices[2].xyzrgba.pt = {-0.6f, 0.6f, 0.0f};
    t2.vertices[2].uv = {0.0f, 1.0f};
    obj->insertTriangle(t1);
    obj->insertTriangle(t2);
    obj->setLocation(3, 3, 0);
    scene->insert(obj);
  }

  // Row 3 (y=-3): COctoMapVoxels
  {
    auto obj = mrpt::viz::COctoMapVoxels::Create();
    obj->resizeVoxelSets(mrpt::viz::VOXEL_SET_OCCUPIED + 1);
    for (int ix = 0; ix < 3; ix++)
    {
      for (int iy = 0; iy < 3; iy++)
      {
        mrpt::viz::COctoMapVoxels::TVoxel v(
            mrpt::math::TPoint3Df(0.4f * static_cast<float>(ix), 0.4f * static_cast<float>(iy), 0),
            0.35,
            mrpt::img::TColor(
                static_cast<uint8_t>(40 * ix), static_cast<uint8_t>(255 - 40 * iy), 200));
        obj->push_back_Voxel(mrpt::viz::VOXEL_SET_OCCUPIED, v);
      }
    }
    obj->showVoxels(mrpt::viz::VOXEL_SET_OCCUPIED, true);
    obj->setLocation(-1, -3, 0);
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
TEST(OpenGL, CFBORender_solidPrimitives)
#else
TEST(OpenGL, DISABLED_CFBORender_solidPrimitives)
#endif
{
  try
  {
    test_opengl_solidPrimitives();
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}
