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

#include <mrpt/apps_gui/CameraSelectionGUI.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Scene.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace mrpt::obs;

// ------------------------------------------------------
//				TestOpenGLVideo
// ------------------------------------------------------
void TestOpenGLVideo()
{
  // Show to the user a list of possible camera drivers and creates and open
  // the selected camera.
  std::cout << "Please, select the input video file or camera...\n";

  mrpt::hwdrivers::CCameraSensor::Ptr cam = mrpt::apps::prepareVideoSourceFromUserSelection();
  if (!cam)
  {
    return;
  }
  std::cout << "Video stream open OK\n";

  // Create 3D window:
  CDisplayWindow3D win("Demo of video textures with MRPT's OpenGL objects", 640, 480);

  // XY Grid
  viz::CGridPlaneXY::Ptr gl_ground = viz::CGridPlaneXY::Create(-7, 7, -7, 7, 0, 1);
  gl_ground->setColor(0.7, 0.7, 0.7);

  // An opengl plane with the video texture
  viz::CTexturedPlane::Ptr gl_plane1 =
      viz::CTexturedPlane::Create(0, 1, 0, 0.75);  // 4/3 aspect ratio
  viz::CTexturedPlane::Ptr gl_plane2 = viz::CTexturedPlane::Create(0, 1, 0, 0.75);
  viz::CTexturedPlane::Ptr gl_plane3 = viz::CTexturedPlane::Create(0, 1, 0, 0.75);

  gl_plane1->setPose(mrpt::poses::CPose3D(0, 0, 1, 0.0_deg, 0.0_deg, -90.0_deg));
  gl_plane2->setPose(mrpt::poses::CPose3D(1, 0, 1, 120.0_deg, 0.0_deg, -90.0_deg));
  gl_plane3->setPose(mrpt::poses::CPose3D(0, 0, 1, 60.0_deg, 0.0_deg, -90.0_deg));

  win.setCameraZoom(5);

  // Insert objects in scene:
  {
    Scene::Ptr& theScene = win.get3DSceneAndLock();
    theScene->insert(gl_ground);
    theScene->insert(gl_plane1);
    theScene->insert(gl_plane2);
    theScene->insert(gl_plane3);
    // IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
    win.unlockAccess3DScene();
  }
  win.repaint();

  std::cout << "Close the window to end.\n";
  while (win.isOpen())
  {
    win.addTextMessage(5, 5, format("%.02fFPS", win.getRenderingFPS()));
    std::this_thread::sleep_for(1ms);

    // Grab new video frame:
    CObservation::Ptr obs = cam->getNextFrame();
    if (obs)
    {
      if (IS_CLASS(*obs, CObservationImage))
      {
        CObservationImage::Ptr o = std::dynamic_pointer_cast<CObservationImage>(obs);
        win.get3DSceneAndLock();
        gl_plane1->assignImage(o->image);
        gl_plane2->assignImage(o->image);
        gl_plane3->assignImage(o->image);
        win.unlockAccess3DScene();
        win.repaint();
      }
    }
  }
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    TestOpenGLVideo();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
