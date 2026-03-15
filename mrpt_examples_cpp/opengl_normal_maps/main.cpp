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
 * \example opengl_normal_maps
 *
 * \brief Example showing normal map textures on 3D planes.
 *
 * This example demonstrates the use of normal maps for enhanced lighting
 * on textured surfaces. The scene shows:
 * - An asphalt ground plane (with normal map)
 * - Two brick walls: one without a normal map, one with a normal map
 * - A skybox
 * - Hemispheric ambient lighting
 *
 * Textures are downloaded on the fly using wget/curl.
 *
 * \image html opengl_normal_maps_screenshot.png
 */

#include <mrpt/core/format.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/Viewport.h>

#include <iostream>
#include <string>
#include <thread>

namespace
{

// ---------------------------------------------------------------------------
// Texture URLs
// ---------------------------------------------------------------------------
const char* URL_ASPHALT_COLOR =
    "https://mrpt.github.io/mvsim-models/textures-cgbookcase/"
    "Asphalt01_1K_BaseColor.png";
const char* URL_ASPHALT_NORMAL =
    "https://mrpt.github.io/mvsim-models/textures-cgbookcase/"
    "Asphalt01_1K_Normal.png";
const char* URL_BRICK_COLOR =
    "https://mrpt.github.io/mvsim-models/textures-cgbookcase/"
    "BrickWall01_1K_BaseColor.png";
const char* URL_BRICK_NORMAL =
    "https://mrpt.github.io/mvsim-models/textures-cgbookcase/"
    "BrickWall01_1K_Normal.png";

// Skybox
const char* URL_SKYBOX_ZIP = "https://mrpt.github.io/mvsim-models/skyboxes/SunSet.zip";
const char* SKYBOX_DIR = "./SunSet/";
const char* SKYBOX_FILE_PATTERN = "./SunSet/SunSet%s.jpg";

// ---------------------------------------------------------------------------
// Helper: download a single file if not already present
// ---------------------------------------------------------------------------
void downloadFile(const std::string& url, const std::string& destPath)
{
  if (mrpt::system::fileExists(destPath))
  {
    return;
  }

  std::cout << "Downloading: " << url << " -> " << destPath << "\n";

  // Try wget first, fall back to curl
  int ret = ::system(mrpt::format(R"(wget  "%s" -O "%s")", url.c_str(), destPath.c_str()).c_str());
  if (ret != 0)
  {
    ret = ::system(mrpt::format(R"(wget  "%s" -o "%s")", url.c_str(), destPath.c_str()).c_str());
  }
  if (ret != 0)
  {
    THROW_EXCEPTION_FMT("Could not download file: %s", url.c_str());
  }
}

// ---------------------------------------------------------------------------
// Download skybox ZIP and extract if needed
// ---------------------------------------------------------------------------
void downloadSkybox()
{
  if (mrpt::system::directoryExists(SKYBOX_DIR))
  {
    return;
  }

  std::cout << "Downloading skybox...\n";
  int ret = ::system(mrpt::format("wget -q \"%s\" -O skybox.zip", URL_SKYBOX_ZIP).c_str());
  if (ret != 0)
  {
    ret = ::system(mrpt::format("wget -q \"%s\" -o skybox.zip", URL_SKYBOX_ZIP).c_str());
  }
  if (ret != 0)
  {
    THROW_EXCEPTION("Could not download skybox zip.");
  }

  ret = ::system("unzip -q skybox.zip");
  if (ret != 0)
  {
    THROW_EXCEPTION("Could not unzip skybox.");
  }
}

// ---------------------------------------------------------------------------
// Main demo
// ---------------------------------------------------------------------------
void TestNormalMaps()
{
  // Download textures
  downloadFile(URL_ASPHALT_COLOR, "asphalt_color.png");
  downloadFile(URL_ASPHALT_NORMAL, "asphalt_normal.png");
  downloadFile(URL_BRICK_COLOR, "brick_color.png");
  downloadFile(URL_BRICK_NORMAL, "brick_normal.png");
  downloadSkybox();

  // -----------------------------------------------------------------------
  // Create the window
  // -----------------------------------------------------------------------
  mrpt::gui::CDisplayWindow3D win("Normal Maps Demo - MRPT", 1024, 768);

  mrpt::viz::Scene::Ptr& theScene = win.get3DSceneAndLock();

  // -----------------------------------------------------------------------
  // Skybox
  // -----------------------------------------------------------------------
  {
    using mrpt::viz::CUBE_TEXTURE_FACE;
    auto sb = mrpt::viz::CSkyBox::Create();

    const std::vector<std::pair<CUBE_TEXTURE_FACE, const char*>> faceImages = {
        { CUBE_TEXTURE_FACE::FRONT, "Front"},
        {  CUBE_TEXTURE_FACE::BACK,  "Back"},
        {CUBE_TEXTURE_FACE::BOTTOM,  "Down"},
        {   CUBE_TEXTURE_FACE::TOP,    "Up"},
        {  CUBE_TEXTURE_FACE::LEFT,  "Left"},
        { CUBE_TEXTURE_FACE::RIGHT, "Right"},
    };
    for (const auto& p : faceImages)
    {
      const std::string fil = mrpt::format(SKYBOX_FILE_PATTERN, p.second);
      sb->assignImage(p.first, mrpt::img::CImage::LoadFromFile(fil));
    }
    theScene->insert(sb);
  }

  // -----------------------------------------------------------------------
  // Hemispheric ambient lighting (blueish sky, brownish ground)
  // -----------------------------------------------------------------------
  {
    auto& lp = theScene->getViewport("main")->lightParameters();
    lp.ambientSkyColor = mrpt::img::TColorf(0.6f, 0.7f, 1.0f);
    lp.ambientGroundColor = mrpt::img::TColorf(0.4f, 0.35f, 0.25f);

    // One directional sun-like light
    lp.lights.clear();
    mrpt::viz::TLight sunLight;
    sunLight.type = mrpt::viz::TLightType::Directional;
    sunLight.direction = {-0.5f, -0.3f, -1.0f};
    sunLight.color = mrpt::img::TColorf(1.0f, 0.95f, 0.85f);
    sunLight.diffuse = 0.9f;
    sunLight.specular = 0.4f;
    lp.lights.push_back(sunLight);
  }

  // -----------------------------------------------------------------------
  // Ground: asphalt WITH normal map
  // -----------------------------------------------------------------------
  {
    const float halfSize = 8.0f;
    auto ground = mrpt::viz::CTexturedPlane::Create(-halfSize, halfSize, -halfSize, halfSize);
    ground->enableLighting(true);

    ground->assignImage(mrpt::img::CImage::LoadFromFile("asphalt_color.png"));
    ground->assignNormalMap(mrpt::img::CImage::LoadFromFile("asphalt_normal.png"));

    // Ground lies in XY plane at Z=0 by default — no pose change needed
    theScene->insert(ground);
  }

  // -----------------------------------------------------------------------
  // Wall 1 (left): brick WITHOUT normal map
  // -----------------------------------------------------------------------
  {
    // A vertical wall: XY plane rotated 90° around X, placed at y=-8
    const float wallW = 8.0f;  // half-width in X
    const float wallH = 3.0f;  // height
    auto wall = mrpt::viz::CTexturedPlane::Create(-wallW, wallW, 0.0f, wallH);
    wall->enableLighting(true);

    wall->assignImage(mrpt::img::CImage::LoadFromFile("brick_color.png"));
    // No normal map for this wall

    // Rotate 90° around X axis so the plane stands vertically, facing +Y
    mrpt::math::TPose3D pose;
    pose.x = 0;
    pose.y = -8.0;
    pose.z = 0;
    pose.yaw = 0;
    pose.roll = mrpt::DEG2RAD(90.0);
    pose.pitch = 0;
    wall->setPose(pose);

    theScene->insert(wall);
  }

  // -----------------------------------------------------------------------
  // Wall 2 (right): brick WITH normal map
  // -----------------------------------------------------------------------
  {
    const float wallW = 8.0f;
    const float wallH = 3.0f;
    auto wall = mrpt::viz::CTexturedPlane::Create(-wallW, wallW, 0.0f, wallH);
    wall->enableLighting(true);

    wall->assignImage(mrpt::img::CImage::LoadFromFile("brick_color.png"));
    wall->assignNormalMap(mrpt::img::CImage::LoadFromFile("brick_normal.png"));

    mrpt::math::TPose3D pose;
    pose.x = 0;
    pose.y = 8.0;
    pose.z = 0;
    pose.yaw = mrpt::DEG2RAD(180.0);
    pose.roll = mrpt::DEG2RAD(90.0);
    pose.pitch = 0;
    wall->setPose(pose);

    theScene->insert(wall);
  }

  // IMPORTANT: unlock before adding 2D text messages
  win.unlockAccess3DScene();

  // -----------------------------------------------------------------------
  // 2D overlay labels
  // -----------------------------------------------------------------------
  mrpt::viz::TFontParams fp;
  fp.color = mrpt::img::TColorf(1, 1, 1);
  fp.vfont_scale = 15;
  win.addTextMessage(0.02, 0.95, "Normal Maps Demo (press ESC to exit)", 0, fp);

  mrpt::viz::TFontParams fp2;
  fp2.color = mrpt::img::TColorf(1, 1, 0.5f);
  fp2.vfont_scale = 13;
  win.addTextMessage(0.02, 0.04, "Front wall: NO normal map", 1, fp2);
  win.addTextMessage(0.02, 0.08, "Back wall:  WITH normal map", 2, fp2);

  // -----------------------------------------------------------------------
  // Camera
  // -----------------------------------------------------------------------
  win.setCameraAzimuthDeg(30.0f);
  win.setCameraElevationDeg(20.0f);
  win.setCameraZoom(18.0f);
  win.setFOV(60.0f);

  win.forceRepaint();

  std::cout << "\n";
  std::cout << "Normal Maps Demo\n";
  std::cout << "  Front wall (y=-8): brick texture WITHOUT normal map\n";
  std::cout << "  Back wall  (y=+8): brick texture WITH normal map\n";
  std::cout << "  Ground           : asphalt WITH normal map\n";
  std::cout << "\nPress ESC or any key to exit.\n";

  // -----------------------------------------------------------------------
  // Event loop
  // -----------------------------------------------------------------------
  while (win.isOpen())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (win.keyHit())
    {
      mrpt::gui::mrptKeyModifier kmods{};
      int key = win.getPushedKey(&kmods);
      if (key == mrpt::gui::MRPTK_ESCAPE)
      {
        break;
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
    TestNormalMaps();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
