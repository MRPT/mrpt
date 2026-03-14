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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>

#include <iostream>
#include <thread>

namespace
{
// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
  using namespace std;
  using namespace mrpt;
  using namespace mrpt::gui;
  using namespace mrpt::math;
  using namespace mrpt::viz;
  using namespace mrpt::img;
  using namespace mrpt::system;

  CDisplayWindow3D win("Example of 3D Scene Visualization - MRPT", 640, 480);

  Scene::Ptr& theScene = win.get3DSceneAndLock();

  // Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
  {
    Viewport::Ptr vi = theScene->createViewport("myClone");
    vi->setViewportPosition(0.7, 0.05, 0.28, 0.28);
    vi->setCloneView("main");
    vi->setTransparent(true);
    vi->getCamera().setAzimuthDegrees(45);
    vi->getCamera().setElevationDegrees(45);
    vi->getCamera().setZoomDistance(10);
  }

  // Another clone viewport, using absolute coordinates
  {
    Viewport::Ptr vi = theScene->createViewport("myClone2");
    vi->setViewportPosition(
        /*x px*/ -250, /*y px*/ -250, /*width px*/ 250,
        /*height px*/ 200);  // x,y negative means pixels from the
    // top/right, instead of from the bottom/left.
    vi->setCloneView("main");
    vi->setTransparent(false);
    vi->getCamera().setAzimuthDegrees(-95);
    vi->getCamera().setElevationDegrees(30);
    vi->getCamera().setZoomDistance(8);
  }

  // And another transparent viewport just to show 3D text:
  {
    mrpt::viz::CText::Ptr txt1 = mrpt::viz::CText::Create();
    auto vi = theScene->createViewport("flat_viewport");
    vi->setViewportPosition(0, 0, 0.3, 0.3);
    vi->setTransparent(true);
    vi->setBorderSize(0);
    vi->getCamera().setAzimuthDegrees(0);
    vi->getCamera().setElevationDegrees(90);
    vi->getCamera().setZoomDistance(5);
    vi->getCamera().setOrthogonal(true);

    vi->insert(txt1);
  }

  // Modify the scene:
  // ------------------------------------------------------
  {
    auto obj = mrpt::viz::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
    obj->setColor(0.8f, 0.8f, 0.8f);
    theScene->insert(obj);
  }

  {
    auto obj = mrpt::viz::CAxis::Create();
    obj->setFrequency(5);
    obj->enableTickMarks();
    obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
    theScene->insert(obj);
  }

  {
    auto obj = mrpt::viz::CBox::Create();
    obj->setWireframe(false);
    obj->setColor(1, 0, 0);
    obj->setLineWidth(3.0);
    obj->setPose(TPose3D(4, 2, 0, 0.2, 0.3, 0.1));
    theScene->insert(obj);
  }

  {
    auto obj = mrpt::viz::CSphere::Create();
    obj->setColor(0, 0, 1);
    obj->setRadius(0.3f);
    obj->setLocation(0, 0, 1);
    obj->setName("ball_1");
    theScene->insert(obj);
  }
  {
    auto obj = mrpt::viz::CSphere::Create();
    obj->setColor(1, 0, 0);
    obj->setRadius(0.3f);
    obj->setLocation(-1, -1, 1);
    obj->setName("ball_2");
    theScene->insert(obj);
  }

  {
    auto obj = mrpt::viz::CSphere::Create();
    obj->setColor_u8(0, 180, 0);  // slightly darker green for visible highlights
    obj->setRadius(0.5);
    obj->setLocation(0, 0, 0);
    obj->setName("USER_MOUSE_PICK");
    obj->materialShininess(0.5f);           // specular reflection intensity
    obj->materialSpecularExponent(128.0f);  // tight highlight → metallic look
    theScene->insert(obj);
  }

  // Emissive material example: a glowing yellow sphere that emits light
  // regardless of scene lighting (like an indicator light or display).
  // Associated with a point light so it actually illuminates nearby objects.
  {
    auto obj = mrpt::viz::CSphere::Create();
    obj->setColor(1, 1, 0);
    obj->setRadius(0.15f);
    obj->setLocation(3, 0, 1.5);
    obj->materialEmissive(mrpt::img::TColorf(1.0f, 0.9f, 0.0f));
    obj->castShadows(false);  // don't block directional light shadows
    obj->setName("emissive_sphere");
    theScene->insert(obj);
  }

  // Configure hemisphere ambient lighting (blueish sky, brownish ground)
  {
    auto& lp = theScene->getViewport("main")->lightParameters();
    lp.ambientSkyColor = mrpt::img::TColorf(0.7f, 0.8f, 1.0f);
    lp.ambientGroundColor = mrpt::img::TColorf(0.4f, 0.35f, 0.3f);

    const mrpt::img::TColorf fogCol(0.5f, 0.5f, 0.5f);
    if (0)
    {
      // Distance fog (linear mode)
      lp.fog_enabled = true;
      lp.fog_color = fogCol;
      lp.fog_near = 15.0f;
      lp.fog_far = 50.0f;
    }

    // Match background to fog color for seamless blending at distance
    theScene->getViewport("main")->setCustomBackgroundColor(fogCol);
  }

  // Add a point light co-located with the emissive sphere
  {
    auto& lp = theScene->getViewport("main")->lightParameters();
    lp.lights.push_back(mrpt::viz::TLight::PointLight(
        {3.0f, 0.0f, 1.5f},                    // position (same as sphere)
        mrpt::img::TColorf(1.0f, 0.9f, 0.0f),  // warm yellow color
        0.8f,                                  // diffuse
        0.5f,                                  // specular
        1.0f,                                  // attenuation constant
        0.09f,                                 // attenuation linear
        0.032f                                 // attenuation quadratic
        ));
  }

  // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
  win.unlockAccess3DScene();

  // Texts:
  mrpt::viz::TFontParams fp;
  fp.color = TColorf(1, 1, 1);
  win.addTextMessage(0.01, 0.85, "This is a 2D message", 0 /*id*/, fp);

  win.setCameraElevationDeg(25.0f);
  // win.setProjectiveModel(false);

  std::cout << "\n";
  std::cout << "Control with mouse or keyboard. Valid keys:"
            << "\n";
  std::cout << "  ESC                        -> Exit"
            << "\n";
  std::cout << "  Left/right cursor arrow    -> Camera azimuth"
            << "\n";
  std::cout << "  P                          -> Enable / disable 'place object' "
               "mode"
            << "\n";
  std::cout << "\n";

  bool end = false;
  bool placeMode = false;

  CTicTac timer;
  timer.Tic();

  while (!end && win.isOpen())
  {
    const double t = timer.Tac();

    // Move the scene:
    auto& scene = win.get3DSceneAndLock();

    const double R1 = 8;
    const double W1 = 5.0;
    const double Q1 = 3.3;
    auto obj1 = scene->getByName("ball_1");
    obj1->setLocation(
        R1 * cos(W1 * t) * sin(Q1 * t), R1 * sin(W1 * t), R1 * cos(W1 * t) * cos(Q1 * t));

    const double R2 = 6;
    const double W2 = 1.3;
    const double Q2 = 7.2;
    auto obj2 = scene->getByName("ball_2");
    obj2->setLocation(
        R2 * cos(W2 * t) * sin(Q2 * t), R2 * sin(W2 * t), R2 * cos(W2 * t) * cos(Q2 * t));

    // Move emissive sphere slowly in a circle and update its point light
    {
      const double Re = 3.0, We = 0.3;
      const float ex = static_cast<float>(Re * cos(We * t));
      const float ey = static_cast<float>(Re * sin(We * t));
      const float ez = 1.5f;
      auto emSphere = scene->getByName("emissive_sphere");
      emSphere->setLocation(ex, ey, ez);

      // Update the point light position (light index 1, after the default directional)
      auto& lp = scene->getViewport("main")->lightParameters();
      if (lp.lights.size() > 1)
      {
        lp.lights[1].position = {ex, ey, ez};
      }
    }

    mrpt::viz::TFontParams fp2;
    fp2.color = TColorf(.8f, .8f, .8f);
    fp2.vfont_name = "sans";
    fp2.vfont_scale = 14;

    win.addTextMessage(
        0.02, 0.02,  // X,Y<=1 means coordinates are factors over the entire
                     // viewport area.
        format("ball#1 pos: %s ", obj1->getPose().translation().asString().c_str()), 10 /*id*/,
        fp2);

    win.addTextMessage(
        5, -15,  // |X|,|Y|>1 means absolute coordinates, negative means
        // from the top instead of the bottom.
        format("Time: %s", mrpt::system::dateTimeLocalToString(mrpt::Clock::now()).c_str()),
        20 /* id */, fp2);

    // Show management of (x,y) mouse coordinates and 3D rays:
    // Get the ray in 3D for the latest mouse (X,Y):
    // ------------------------------------------------------------
    if (const auto ray = win.getLastMousePositionRay(); placeMode && ray)
    {
      // Create a 3D plane, e.g. Z=0
      const mrpt::math::TPlane ground_plane(
          TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));

      // Intersection of the line with the plane:
      mrpt::math::TObject3D inters;
      mrpt::math::intersect(*ray, ground_plane, inters);

      // Interpret the intersection as a point, if there is an
      // intersection:
      mrpt::math::TPoint3D inters_pt;
      if (inters.getPoint(inters_pt))
      {
        // Move an object to the position picked by the user:
        // printf("PT: %f %f %f\n",);
        scene->getByName("USER_MOUSE_PICK")->setLocation(inters_pt.x, inters_pt.y, inters_pt.z);
      }
    }

    // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
    win.unlockAccess3DScene();

    // Update window:
    win.forceRepaint();
    std::this_thread::sleep_for(20ms);

    if (mrpt::system::os::kbhit())
    {
      end = true;
    }
    if (win.keyHit())
    {
      mrptKeyModifier kmods = {};
      int key = win.getPushedKey(&kmods);
      printf("Key pushed: %c (%i) - modifiers: 0x%04X\n", char(key), key, kmods);

      if (key == MRPTK_ESCAPE)
      {
        end = true;
      }

      if (key == MRPTK_RIGHT)
      {
        win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() + 5);
      }
      if (key == MRPTK_LEFT)
      {
        win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() - 5);
      }

      if (key == 'p' || key == 'P')
      {
        placeMode = !placeMode;
        win.setCursorCross(placeMode);
      }
    };
  }
}
}  // namespace

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    TestDisplay3D();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
