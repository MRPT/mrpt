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

#include <gtest/gtest.h>

// mrpt::opengl (rendering)
#include <mrpt/opengl/CFBORender.h>

// mrpt::viz (scene graph)
#include <mrpt/opengl/config.h>  // for MRPT_HAS_*
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>
#include <test_mrpt_common.h>

#include <Eigen/Dense>

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_EGL
#define RUN_OFFSCREEN_RENDER_TESTS
#endif

// In MIPS, these tests crash in autobuilders, skip them:
#if defined(__mips__) || defined(__mips)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
// Idem with PowerPC:
#if defined(__powerpc) || defined(__powerpc__) || defined(__powerpc64__) ||               \
    defined(__POWERPC__) || defined(__ppc__) || defined(__PPC__) || defined(_ARCH_PPC) || \
    defined(__PPC64__) || defined(__ppc64__) || defined(_ARCH_PPC64)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
// Idem with 32bit ARM archs:
#if defined(__arm__) && MRPT_WORD_SIZE == 32
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
// Idem with RISCV64 arch:
#if defined(__riscv)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif
// Idem with Loongarch arch:
#if defined(__loongarch__)
#undef RUN_OFFSCREEN_RENDER_TESTS
#endif

namespace
{
float imageDiff(const mrpt::img::CImage& im1, const mrpt::img::CImage& im2)
{
  mrpt::math::CMatrixFloat r1, g1, b1;
  im1.getAsRGBMatrices(r1, g1, b1);

  mrpt::math::CMatrixFloat r2, g2, b2;
  im2.getAsRGBMatrices(r2, g2, b2);

  return (r1 - r2).asEigen().array().abs().sum() + (g1 - g2).asEigen().array().abs().sum() +
         (b1 - b2).asEigen().array().abs().sum();
}

void test_opengl_CFBORender(const bool useCameraFromIntrinsics)
{
  using namespace mrpt;                  // _deg
  using namespace std::string_literals;  // s
  using namespace mrpt::viz;             // Scene graph classes

  const std::string expected_RGB_img_file =
      UNITTEST_BASEDIR() + "/../../tests/CFBORender_expected_rgb_"s +
      (useCameraFromIntrinsics ? "camInt"s : "camFOV"s) + ".png"s;

  const std::string expected_depth_img_file =
      UNITTEST_BASEDIR() + "/../../tests/CFBORender_expected_depth_"s +
      (useCameraFromIntrinsics ? "camInt"s : "camFOV"s) + ".png"s;

  // Create a viz::Scene (abstract scene graph, no OpenGL dependency)
  auto scene = mrpt::viz::Scene::Create();

  // Add objects to the scene
  {
    auto obj = mrpt::viz::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 5);
    obj->setColor(0.4f, 0.4f, 0.4f);
    scene->insert(obj);
  }
  {
    auto obj =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(.1, .1, .1));
    obj->setColor(1.0f, 0.f, 0.f);
    obj->setName("x");
    obj->enableShowName(true);
    obj->setLocation(1.0, 0, 0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CTexturedPlane::Create();
    obj->setPlaneCorners(-10, 10, -10, 10);
    obj->setColor_u8(0x00, 0xff, 0xff, 0xff);
    obj->setLocation(0, 0, -14);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSphere::Create();
    obj->setColor(0, 0, 1);
    obj->setRadius(1.0f);
    obj->setLocation(0, 1, 0);
    obj->setName("ball_1");
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CText3D::Create("Hi there!");
    obj->setLocation(5.0, 3.0, 1.0);
    scene->insert(obj);
  }
  {
    auto obj = mrpt::viz::stock_objects::CornerXYZ(2.0f);
    obj->setLocation(-3, -2, 0.5);
    scene->insert(obj);
  }

  // CMeshFast, CMesh:
  {
    double off_x = -5.0, STEP_X = 1.0;
    auto obj1 = mrpt::viz::CMeshFast::Create();
    auto obj2 = mrpt::viz::CMeshFast::Create();
    auto obj3 = mrpt::viz::CMesh::Create();
    auto obj4 = mrpt::viz::CMesh::Create();

    obj1->setXBounds(-1, 1);
    obj1->setYBounds(-1, 1);

    const int W = 128, H = 128;

    mrpt::math::CMatrixDynamic<float> Z(H, W);

    for (int r = 0; r < H; r++)
    {
      for (int c = 0; c < W; c++)
      {
        Z(r, c) = std::sin(0.05 * (c + r) - 0.5) * cos(0.9 - 0.03 * r);
      }
    }

    const std::string texture_file =
        mrpt::system::getShareMRPTDir() + "datasets/sample-texture-terrain.jpg"s;

    mrpt::img::CImage im;

    // obj1:
    obj1->setZ(Z);
    obj1->enableColorFromZ(true);
    obj1->setPointSize(2.0);
    obj1->setLocation(off_x, 0, 0);
    scene->insert(obj1);

    // obj2:
    if (im.loadFromFile(texture_file))
    {
      obj2->assignImageAndZ(im, Z);
      obj2->setPointSize(2.0);
      obj2->setLocation(off_x, 3, 0);
      scene->insert(obj2);
    }

    off_x += STEP_X;

    // obj3:
    obj3->setZ(Z);
    obj3->enableColorFromZ(true, mrpt::img::cmJET);
    obj3->enableWireFrame(true);
    obj3->setLocation(off_x, 0, 0);
    obj3->cullFaces(mrpt::viz::TCullFace::BACK);
    scene->insert(obj3);

    // obj4:
    if (im.getWidth() > 1)
    {
      obj4->assignImageAndZ(im, Z);
      obj4->setLocation(off_x, 3, 0);
      obj4->cullFaces(mrpt::viz::TCullFace::BACK);
      scene->insert(obj4);
    }
  }

  int width = 500, height = 400;
  const double cameraFOVdeg = 120.0;

  ::setenv("MRPT_FBORENDER_SHOW_DEVICES", "1", 1);

  // Create the FBO renderer (mrpt::opengl)
  mrpt::opengl::CFBORender renderer(width, height);
  mrpt::img::CImage frame(width, height, mrpt::img::CH_RGB);
  mrpt::math::CMatrixFloat depth;

  // Configure viewport
  auto viewport = scene->getViewport();
  viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
  const float clipMax = 25.0f;
  viewport->setViewportClipDistances(0.1, clipMax);

  // Configure camera
  {
    // Create camera and set it as override in the renderer
    mrpt::viz::CCamera camera;

    if (useCameraFromIntrinsics)
    {
      mrpt::img::TCamera c1;
      c1.ncols = width;
      c1.nrows = height;
      c1.fx(width * 0.5);
      c1.fy(width * 0.5);
      c1.cx(width / 2);
      c1.cy(height / 2);

      camera.setProjectiveFromPinhole(c1);
    }
    else
    {
      camera.setProjectiveFOVdeg(cameraFOVdeg);
    }

    // Defined by setPose() instead of orbit values:
    camera.set6DOFMode(true);

    // Reference camera pose:
    auto robotPose =
        mrpt::poses::CPose3D(0, 0, 10.0, 0.0_deg /*yaw*/, 90.0_deg /*pitch*/, 0.0_deg /*roll*/);

    // Convert to +Z pointing forward camera axes:
    auto camPose = robotPose + mrpt::poses::CPose3D::FromYawPitchRoll(
                                   90.0_deg /*yaw*/, 0.0_deg /*pitch*/, 90.0_deg /*roll*/);

    camera.setPose(camPose);

    // Set camera override in renderer
    renderer.setCamera(camera);
  }

  // Render the scene
  renderer.render_RGBD(*scene, frame, depth);

  // Compare with ground truth
  mrpt::img::CImage gt_frame;
  bool readOk_rgb = gt_frame.loadFromFile(expected_RGB_img_file);

  EXPECT_TRUE(readOk_rgb);

  const float rgb_diff = imageDiff(gt_frame, frame);
  std::cout << "rgb_diff=" << rgb_diff << "\n";
  EXPECT_LT(rgb_diff, 5000.0f);

  {
    mrpt::img::CImage imDepth;
    depth *= (1.0f / clipMax);
    imDepth.setFromMatrix(depth, true);

    mrpt::img::CImage gt_imDepth;
    bool readOk_depth = gt_imDepth.loadFromFile(expected_depth_img_file);

    EXPECT_TRUE(readOk_depth);

    const float depth_diff = imageDiff(gt_imDepth, imDepth);
    std::cout << "depth_diff=" << depth_diff << "\n";
    EXPECT_LT(depth_diff, 3000.0f);
  }
}

}  // namespace

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, CFBORender_camera_intrinsics)
#else
TEST(OpenGL, DISABLED_CFBORender_camera_intrinsics)
#endif
{
  try
  {
    test_opengl_CFBORender(true);
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}

#if defined(RUN_OFFSCREEN_RENDER_TESTS)
TEST(OpenGL, CFBORender_camera_fov)
#else
TEST(OpenGL, DISABLED_CFBORender_camera_fov)
#endif
{
  try
  {
    test_opengl_CFBORender(false);
  }
  catch (const std::exception& e)
  {
    std::cerr << "***** WARNING ****: Ignoring exception in test, likely due to limited "
                 "rendering capabilities on this device (?):\n"
              << e.what() << "\n";
  }
}