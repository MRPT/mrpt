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

/**
 * \example opengl_ssao_demo
 *
 * \brief Interactive demo of Screen-Space Ambient Occlusion (SSAO) and
 * dynamic point lighting.
 *
 * The scene contains a ground plane, several boxes, cylinders and spheres
 * arranged to highlight the difference SSAO makes in crevices and corners.
 * A global directional light and an optional colored point light are set up.
 *
 * Keyboard shortcuts:
 *  - S : toggle SSAO on/off
 *  - P : toggle point light on/off
 *  - H : toggle shadow casting on/off
 *  - ESC / Q : quit
 *
 * \image html opengl_ssao_demo_screenshot.png
 */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/Viewport.h>

#include <chrono>
#include <cstdio>
#include <thread>

namespace
{

// Index of the point light inside TLightParameters::lights (light 0 is the
// directional one; light 1 is our point light).
constexpr int POINT_LIGHT_INDEX = 1;

// -----------------------------------------------------------------------
// Build the demo 3D scene
// -----------------------------------------------------------------------
void buildScene(mrpt::viz::Scene& scene)
{
  // -- Ground plane --
  {
    auto obj = mrpt::viz::CGridPlaneXY::Create(-10, 10, -10, 10, 0, 0.5f);
    obj->setColor(0.55f, 0.55f, 0.55f);
    scene.insert(obj);
  }

  // -- A flat base plate so SSAO has something to occlude against the floor --
  {
    auto box = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-5.5, -5.5, -0.02), mrpt::math::TPoint3D(5.5, 5.5, 0.0));
    box->setColor(0.75f, 0.75f, 0.75f);
    box->enableLight(true);
    scene.insert(box);
  }

  // -- Row of boxes of increasing height --
  for (int i = 0; i < 5; i++)
  {
    const float h = 0.5f + 0.4f * static_cast<float>(i);
    auto box = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-0.35, -0.35, 0.0), mrpt::math::TPoint3D(0.35, 0.35, h));
    box->setColor(0.8f, 0.4f + 0.1f * static_cast<float>(i), 0.3f);
    box->enableLight(true);
    mrpt::math::TPose3D p;
    p.x = -4.0 + 2.0 * i;
    p.y = -2.5;
    box->setPose(p);
    scene.insert(box);
  }

  // -- Row of cylinders --
  for (int i = 0; i < 4; i++)
  {
    const float r = 0.25f + 0.1f * static_cast<float>(i);
    auto cyl = mrpt::viz::CCylinder::Create(r, r, 1.2f, 20);
    cyl->setColor(0.3f, 0.5f, 0.8f + 0.05f * static_cast<float>(i));
    cyl->enableLight(true);
    mrpt::math::TPose3D p;
    p.x = -3.0 + 2.0 * i;
    p.y = 1.5;
    cyl->setPose(p);
    scene.insert(cyl);
  }

  // -- A cluster of spheres of various sizes --
  const float spos[][3] = {
      { 0.0f, 4.5f, 0.45f},
      { 1.1f, 4.5f, 0.30f},
      {-1.0f, 4.5f, 0.30f},
      { 0.5f, 5.4f, 0.25f},
      {-0.5f, 3.6f, 0.25f},
  };
  const float srad[] = {0.45f, 0.30f, 0.30f, 0.25f, 0.25f};
  for (int i = 0; i < 5; i++)
  {
    auto sph = mrpt::viz::CSphere::Create(srad[i], 18);
    sph->setColor(0.9f, 0.85f, 0.3f);
    sph->enableLight(true);
    mrpt::math::TPose3D p;
    p.x = spos[i][0];
    p.y = spos[i][1];
    p.z = spos[i][2];
    sph->setPose(p);
    scene.insert(sph);
  }

  // -- A "room corner" (two tall boxes placed at right angles) --
  {
    auto wallA = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-0.15, -0.15, 0.0), mrpt::math::TPoint3D(0.15, 4.0, 2.0));
    wallA->setColor(0.85f, 0.82f, 0.78f);
    wallA->enableLight(true);
    mrpt::math::TPose3D p;
    p.x = -4.8;
    p.y = 2.0;
    wallA->setPose(p);
    scene.insert(wallA);
  }
  {
    auto wallB = mrpt::viz::CBox::Create(
        mrpt::math::TPoint3D(-0.15, -0.15, 0.0), mrpt::math::TPoint3D(4.0, 0.15, 2.0));
    wallB->setColor(0.78f, 0.82f, 0.85f);
    wallB->enableLight(true);
    mrpt::math::TPose3D p;
    p.x = -4.8;
    p.y = 6.0;
    wallB->setPose(p);
    scene.insert(wallB);
  }

  // -- Small box cluster (tight packing to show contact shadows) --
  const int NB = 3;
  for (int ix = 0; ix < NB; ix++)
  {
    for (int iy = 0; iy < NB; iy++)
    {
      auto box = mrpt::viz::CBox::Create(
          mrpt::math::TPoint3D(0.0, 0.0, 0.0), mrpt::math::TPoint3D(0.45, 0.45, 0.45));
      box->setColor(
          0.65f + 0.1f * static_cast<float>(ix), 0.65f + 0.1f * static_cast<float>(iy), 0.65f);
      box->enableLight(true);
      mrpt::math::TPose3D p;
      p.x = 2.5 + 0.48 * ix;
      p.y = 3.5 + 0.48 * iy;
      box->setPose(p);
      scene.insert(box);
    }
  }
}

// -----------------------------------------------------------------------
// Configure lighting (called once; call again to re-apply after toggle)
// -----------------------------------------------------------------------
void setupLighting(mrpt::viz::Viewport& vp, bool pointLightOn)
{
  auto& lp = vp.lightParameters();

  // Hemisphere ambient: neutral sky/ground
  lp.ambientSkyColor = mrpt::img::TColorf(0.5f, 0.55f, 0.65f);
  lp.ambientGroundColor = mrpt::img::TColorf(0.3f, 0.28f, 0.25f);
  lp.ambient = 0.25f;

  // Clear existing lights; add directional + optional point light
  lp.lights.clear();

  // Directional sun-like light
  mrpt::viz::TLight sun;
  sun.type = mrpt::viz::TLightType::Directional;
  sun.direction = {-0.6f, -0.4f, -0.7f};
  sun.color = mrpt::img::TColorf(1.0f, 0.95f, 0.85f);
  sun.diffuse = 0.85f;
  sun.specular = 0.4f;
  lp.lights.push_back(sun);

  // Colored point light above the scene (toggle-able)
  if (pointLightOn)
  {
    lp.lights.push_back(mrpt::viz::TLight::PointLight(
        {0.0f, 0.0f, 4.0f},                    // above the scene center
        mrpt::img::TColorf(0.4f, 0.6f, 1.0f),  // cool blue-white
        1.0f,                                  // diffuse
        0.7f,                                  // specular
        1.0f,                                  // att_constant
        0.05f,                                 // att_linear
        0.008f                                 // att_quadratic
        ));
  }
}

// -----------------------------------------------------------------------
// Update the on-screen status text
// -----------------------------------------------------------------------
void updateStatusText(mrpt::gui::CDisplayWindow3D& win, bool ssaoOn, bool pointOn, bool shadowsOn)
{
  mrpt::viz::TFontParams fp;
  fp.color = mrpt::img::TColorf(1.0f, 1.0f, 1.0f);
  fp.vfont_scale = 14;
  win.addTextMessage(0.02, 0.97, "SSAO Demo  |  S=SSAO  P=Point light  H=Shadows  ESC=Quit", 0, fp);

  mrpt::viz::TFontParams fp2;
  fp2.vfont_scale = 13;
  fp2.color = ssaoOn ? mrpt::img::TColorf(0.3f, 1.0f, 0.3f) : mrpt::img::TColorf(1.0f, 0.4f, 0.4f);
  win.addTextMessage(0.02, 0.04, std::string("SSAO: ") + (ssaoOn ? "ON" : "OFF"), 1, fp2);

  mrpt::viz::TFontParams fp3;
  fp3.vfont_scale = 13;
  fp3.color = pointOn ? mrpt::img::TColorf(0.3f, 1.0f, 0.3f) : mrpt::img::TColorf(1.0f, 0.4f, 0.4f);
  win.addTextMessage(0.02, 0.07, std::string("Point light: ") + (pointOn ? "ON" : "OFF"), 2, fp3);

  mrpt::viz::TFontParams fp4;
  fp4.vfont_scale = 13;
  fp4.color =
      shadowsOn ? mrpt::img::TColorf(0.3f, 1.0f, 0.3f) : mrpt::img::TColorf(1.0f, 0.4f, 0.4f);
  win.addTextMessage(0.02, 0.10, std::string("Shadows: ") + (shadowsOn ? "ON" : "OFF"), 3, fp4);
}

// -----------------------------------------------------------------------
void RunDemo()
{
  mrpt::gui::CDisplayWindow3D win("SSAO Demo - MRPT", 1280, 800);

  bool ssaoOn = false;
  bool pointLightOn = true;
  bool shadowsOn = false;

  // ----- Build scene -----
  {
    mrpt::viz::Scene::Ptr& scene = win.get3DSceneAndLock();
    buildScene(*scene);
    auto vp = scene->getViewport("main");

    setupLighting(*vp, pointLightOn);
    vp->lightParameters().ssao_enabled = ssaoOn;
    vp->enableShadowCasting(shadowsOn);

    win.unlockAccess3DScene();
  }

  // ----- Camera -----
  win.setCameraAzimuthDeg(40.0f);
  win.setCameraElevationDeg(30.0f);
  win.setCameraZoom(20.0f);
  win.setFOV(55.0f);

  updateStatusText(win, ssaoOn, pointLightOn, shadowsOn);
  win.forceRepaint();

  std::printf(
      "\nSSAO Demo\n"
      "  S - Toggle SSAO (Screen-Space Ambient Occlusion)\n"
      "  P - Toggle point light\n"
      "  H - Toggle shadow casting\n"
      "  ESC / Q - Quit\n\n");

  // ----- Event loop -----
  while (win.isOpen())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    if (win.keyHit())
    {
      mrpt::gui::mrptKeyModifier kmods{};
      const int key = win.getPushedKey(&kmods);

      bool changed = false;

      if (key == mrpt::gui::MRPTK_ESCAPE || key == 'q' || key == 'Q')
      {
        break;
      }
      else if (key == 's' || key == 'S')
      {
        ssaoOn = !ssaoOn;
        changed = true;
      }
      else if (key == 'p' || key == 'P')
      {
        pointLightOn = !pointLightOn;
        changed = true;
      }
      else if (key == 'h' || key == 'H')
      {
        shadowsOn = !shadowsOn;
        changed = true;
      }

      if (changed)
      {
        {
          mrpt::viz::Scene::Ptr& scene = win.get3DSceneAndLock();
          auto vp = scene->getViewport("main");
          setupLighting(*vp, pointLightOn);
          vp->lightParameters().ssao_enabled = ssaoOn;
          vp->enableShadowCasting(shadowsOn);
          win.unlockAccess3DScene();
        }
        updateStatusText(win, ssaoOn, pointLightOn, shadowsOn);
        win.forceRepaint();
      }
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------------
int main()
{
  try
  {
    RunDemo();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "MRPT error: %s\n", e.what());
    return -1;
  }
}
