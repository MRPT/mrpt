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

/** @file demo_imgui_scene.cpp
 *  Minimal demo: MRPT 3D scene rendered inside a Dear ImGui window.
 *
 *  Build:
 *    mkdir build && cd build
 *    cmake .. -DCMAKE_BUILD_TYPE=Release
 *    cmake --build .
 *    ./demo_imgui_scene
 */

#include <mrpt/imgui/CImGuiSceneView.h>

// MRPT viz
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>

// This must come before imgui
#include <mrpt/opengl/opengl_api.h>

// Dear ImGui + GLFW + OpenGL3 backend
#include <imgui.h>
//
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
//
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

namespace {
// ------------------------------------------------------------------
// Build a sample MRPT scene
// ------------------------------------------------------------------
[[nodiscard]] mrpt::viz::Scene::Ptr createSampleScene()
{
  auto scene = mrpt::viz::Scene::Create();
  auto& mainVP = *scene->getViewport("main");

  // Ground grid
  {
    auto grid = mrpt::viz::CGridPlaneXY::Create(-20.0f, 20.0f, -20.0f, 20.0f, 0.0f, 1.0f);
    grid->setColor_u8(mrpt::img::TColor(200, 200, 200, 100));
    mainVP.insert(grid);
  }

  // Axes
  {
    auto axes = mrpt::viz::CAxis::Create(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f, 0.5f, 2.0f);
    axes->enableTickMarks();
    mainVP.insert(axes);
  }

  // A box
  {
    auto box =
        mrpt::viz::CBox::Create(mrpt::math::TPoint3D(-1, -1, 0), mrpt::math::TPoint3D(1, 1, 2));
    box->setColor(0.2f, 0.6f, 0.9f, 0.7f);
    box->setLocation(3.0, 0.0, 0.0);
    mainVP.insert(box);
  }

  // A sphere
  {
    auto sphere = mrpt::viz::CSphere::Create(1.0f);
    sphere->setColor(0.9f, 0.3f, 0.2f, 0.8f);
    sphere->setLocation(-3.0, 0.0, 1.5);
    mainVP.insert(sphere);
  }

  // Corner coordinate frames via stock_objects
  {
    auto corner = mrpt::viz::stock_objects::CornerXYZSimple(1.5f, 3.0f);
    corner->setLocation(0.0, 4.0, 0.0);
    mainVP.insert(corner);
  }

  // Random point cloud
  {
    auto pc = mrpt::viz::CPointCloud::Create();
    pc->setPointSize(3.0f);
    pc->setColor(0.1f, 0.8f, 0.1f);

    std::mt19937 rng(42);
    std::normal_distribution<float> dist(0.0f, 2.0f);
    const size_t N = 5000;
    pc->resize(N);
    for (size_t i = 0; i < N; ++i)
    {
      pc->setPoint(i, dist(rng) - 5.0f, dist(rng) + 5.0f, std::abs(dist(rng)));
    }

    mainVP.insert(pc);
  }

  return scene;
}

// ------------------------------------------------------------------
void glfwErrorCallback(int error, const char* description)
{
  std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}
}

// ------------------------------------------------------------------
int main()
{
  // --- GLFW init ---
  glfwSetErrorCallback(glfwErrorCallback);
  if (glfwInit() == GLFW_FALSE)
  {
    std::fprintf(stderr, "Failed to initialize GLFW.\n");
    return EXIT_FAILURE;
  }

  // GL 3.3 Core
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(1280, 720, "MRPT + Dear ImGui Demo", nullptr, nullptr);
  if (window == nullptr)
  {
    std::fprintf(stderr, "Failed to create GLFW window.\n");
    glfwTerminate();
    return EXIT_FAILURE;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // VSync

  // --- Dear ImGui init ---
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  // --- MRPT scene + widget ---
  auto scene = createSampleScene();

  mrpt::imgui::CImGuiSceneView sceneView;
  sceneView.setScene(scene);
  sceneView.setBackgroundColor(0.15f, 0.15f, 0.18f);

  // Tweak the default camera
  sceneView.camera().setZoomDistance(25.0f);
  sceneView.camera().setAzimuthDegrees(-140.0f);
  sceneView.camera().setElevationDegrees(30.0f);

  // Optional click callback
  sceneView.onLeftClick = [](float px, float py)
  { std::printf("Clicked at widget coords (%.1f, %.1f)\n", px, py); };

  // --- animation state ---
  float sphereAngle = 0.0f;

  // ----------------------------------------------------------
  // Main loop
  // ----------------------------------------------------------
  while (glfwWindowShouldClose(window) == GLFW_FALSE)
  {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Create a dockspace in main viewport.
    ImGui::DockSpaceOverViewport();

    // --- Animate the sphere ---
    sphereAngle += 0.01f;
    if (auto vp = scene->getViewport("main"); vp)
    {
      // Find the sphere and move it
      if (auto sp = vp->getByClass<mrpt::viz::CSphere>(); sp)
      {
        sp->setLocation(
            -3.0 + 2.0 * std::cos(static_cast<double>(sphereAngle)),
            2.0 * std::sin(static_cast<double>(sphereAngle)), 1.5);
      }
    }

    // --- 3D viewport window ---
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("3D Scene"))
    {
      sceneView.render();
    }
    ImGui::End();

    // --- Controls window ---
    if (ImGui::Begin("Controls"))
    {
      auto& cam = sceneView.camera();

      float az = cam.getAzimuthDegrees();
      float el = cam.getElevationDegrees();
      float dist = cam.getZoomDistance();

      bool changed = false;
      changed |= ImGui::SliderFloat("Azimuth", &az, -180.0f, 180.0f);
      changed |= ImGui::SliderFloat("Elevation", &el, -89.0f, 89.0f);
      changed |= ImGui::SliderFloat("Distance", &dist, 0.5f, 100.0f);

      if (changed)
      {
        cam.setAzimuthDegrees(az);
        cam.setElevationDegrees(el);
        cam.setZoomDistance(dist);
      }

      ImGui::Separator();

      float bg[3] = {0.15f, 0.15f, 0.18f};
      if (ImGui::ColorEdit3("Background", bg))
      {
        sceneView.setBackgroundColor(bg[0], bg[1], bg[2]);
      }

      ImGui::Separator();
      ImGui::Text(
          "Camera pointing at: (%.1f, %.1f, %.1f)", static_cast<double>(cam.getPointingAtX()),
          static_cast<double>(cam.getPointingAtY()), static_cast<double>(cam.getPointingAtZ()));

      ImGui::Text("FPS: %.1f", static_cast<double>(io.Framerate));
    }
    ImGui::End();

    // --- Render ImGui ---
    ImGui::Render();
    int fbW = 0;
    int fbH = 0;
    glfwGetFramebufferSize(window, &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // --- Cleanup ---
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return EXIT_SUCCESS;
}
