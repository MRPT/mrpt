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

#include <mrpt/core/round.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/stock_objects.h>
#include <mrpt/system/CObserver.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace mrpt::system;

// This is my custom class to handle the pre/post render events:
struct TMyExtraRenderingStuff : public mrpt::system::CObserver
{
  viz::CSphere::Ptr ball_obj;  // The ball moving in the scene

  TMyExtraRenderingStuff() {}
  void OnEvent(const mrptEvent& e) override
  {
    // Pre/post render events were removed in MRPT v3.
    // Custom rendering can be done via other mechanisms.
  }
};

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
  CDisplayWindow3D win("Example of 3D Scene Visualization - MRPT", 640, 480);

  Scene::Ptr& theScene = win.get3DSceneAndLock();

  // The unique instance of the observer class:
  TMyExtraRenderingStuff my_extra_rendering;

  // And start subscribing to the viewport events:
  viz::Viewport::Ptr the_main_view = theScene->getViewport("main");
  my_extra_rendering.observeBegin(*the_main_view);

  // Modify the scene:
  // ------------------------------------------------------
  {
    viz::CGridPlaneXY::Ptr obj = viz::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
    obj->setColor(0.8f, 0.8f, 0.8f);
    theScene->insert(obj);
  }

  theScene->insert(mrpt::viz::stock_objects::CornerXYZ());

  if (true)
  {
    viz::CAxis::Ptr obj = viz::CAxis::Create();
    obj->setFrequency(5);
    obj->enableTickMarks();
    obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
    theScene->insert(obj);
  }

  {
    viz::CSphere::Ptr obj = viz::CSphere::Create();
    obj->setColor(0, 0, 1);
    obj->setRadius(0.3f);
    obj->setLocation(0, 0, 1);
    obj->setName("ball_1");
    theScene->insert(obj);

    // And also let my rendering object access this ball properties:
    my_extra_rendering.ball_obj = obj;
  }

  // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
  win.unlockAccess3DScene();

  // Texts:
  win.addTextMessage(0.01, 0.85, "This is a 2D message", 0 /*id */);

  win.setCameraElevationDeg(25.0f);
  // win.setCameraProjective(false);

  win.addTextMessage(0.7, 0.9, "Press 'h' for help", 1 /*id*/);

  cout << endl;
  cout << "Control with mouse or keyboard. Valid keys:" << endl;
  cout << "  ESC                        -> Exit" << endl;
  cout << "  Left/right cursor arrow    -> Camera azimuth" << endl;
  cout << endl;

  bool end = false;

  CTicTac timer;
  timer.Tic();

  while (!end && win.isOpen())
  {
    // Move the scene:
    Scene::Ptr& scene = win.get3DSceneAndLock();

    viz::CVisualObject::Ptr obj1 = scene->getByName("ball_1");
    const double t = timer.Tac();
    const double R = 8;
    const double W = 5.0, Q = 3.3;
    obj1->setLocation(R * cos(W * t) * sin(Q * t), R * sin(W * t), R * cos(W * t) * cos(Q * t));

    // Update the texts on the gl display:
    win.addTextMessage(5, 5, mrpt::format("FPS=%5.02f", win.getRenderingFPS()), 0);

    // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
    win.unlockAccess3DScene();

    // Update window:
    win.forceRepaint();
    std::this_thread::sleep_for(1ms);

    if (mrpt::system::os::kbhit()) end = true;
    if (win.keyHit())
    {
      mrptKeyModifier kmods;
      int key = win.getPushedKey(&kmods);
      // printf("Key pushed: %c (%i) - modifiers:
      // 0x%04X\n",char(key),key,kmods);

      if (key == MRPTK_ESCAPE) end = true;

      if (key == 'h' || key == 'H')
      {
        std::cout << "These are the supported commands:\n"
                     " - 'h': Toggle help view\n"
                     " - '<-' and '->': Rotate camera\n"
                     " - 'Alt+Enter': Toggle fullscreen\n"
                     " - 'ESC': Quit"
                     "\n";
      }

      if (key == MRPTK_RIGHT) win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() + 5);
      if (key == MRPTK_LEFT) win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() - 5);
    }
  };
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    TestDisplay3D();
    // leave time for the window to close
    std::this_thread::sleep_for(50ms);
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
}
