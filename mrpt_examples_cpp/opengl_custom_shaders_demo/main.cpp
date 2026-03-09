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
 * Demo: installing a custom GLSL shader that colours triangles by eye-space depth.
 *
 * In MRPT v3 the shader pipeline is managed by ShaderProgramManager (inside
 * CompiledScene).  Custom shaders are installed via:
 *   ShaderProgramManager::loadCustomProgram()   – add a new named program
 *   ShaderProgramManager::overrideBuiltinProgram() – replace a built-in slot
 *
 * CDisplayWindow3D::sendFunctionWithShaderManager() runs a callback on the
 * GL thread with a reference to the active ShaderProgramManager.
 */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/ShaderProgramManager.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/Scene.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Custom vertex shader: same inputs as the built-in TRIANGLES_NO_LIGHT shader
// (position, vertexColor, pmv_matrix) plus mv_matrix to compute eye-space z.
// ---------------------------------------------------------------------------
static const char* CUSTOM_VERT = R"XXX(#version 300 es

layout(location = 0) in highp vec3 position;
layout(location = 1) in lowp  vec4 vertexColor;

uniform highp mat4 pmv_matrix;
uniform highp mat4 mv_matrix;

out highp float frag_eyeZ;

void main()
{
    gl_Position = pmv_matrix * vec4(position, 1.0);
    // Eye-space Z (positive = in front of camera)
    frag_eyeZ = -(mv_matrix * vec4(position, 1.0)).z;
}
)XXX";

// ---------------------------------------------------------------------------
// Custom fragment shader: map eye-space depth to a blue→red colour ramp.
// ---------------------------------------------------------------------------
static const char* CUSTOM_FRAG = R"XXX(#version 300 es

in highp float frag_eyeZ;
out lowp vec4 color;

void main()
{
    // Remap depth [0, 30] → [0, 1] and use as a blue→red gradient
    highp float t = clamp(frag_eyeZ / 30.0, 0.0, 1.0);
    color = vec4(t, 0.2, 1.0 - t, 1.0);
}
)XXX";

// ---------------------------------------------------------------------------
static void installCustomShader(mrpt::opengl::ShaderProgramManager& mgr)
{
  std::string errMsg;
  auto prog = mgr.loadCustomProgram("depth_colour", CUSTOM_VERT, CUSTOM_FRAG, "", &errMsg);
  if (!prog)
  {
    std::cerr << "[installCustomShader] Compilation/link error:\n" << errMsg << "\n";
    return;
  }

  std::cout << "[installCustomShader] Custom depth-colour shader compiled OK.\n";

  // Override the no-light triangles slot so all boxes use the new shader:
  mgr.overrideBuiltinProgram(mrpt::opengl::DefaultShaderID::TRIANGLES_NO_LIGHT, prog);
}

// ---------------------------------------------------------------------------
void DemoCustomShaders()
{
  using namespace mrpt;
  using namespace mrpt::gui;
  using namespace mrpt::viz;
  using namespace mrpt::math;

  CDisplayWindow3D win("Demo of MRPT v3 custom shaders", 640, 480);

  Scene::Ptr& theScene = win.get3DSceneAndLock();

  // Add a few boxes that will be rendered with the custom depth-colour shader
  {
    auto obj = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
    obj->setLocation(0, 0, 0);
    theScene->insert(obj);

    auto obj2 = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
    obj2->setLocation(0, 4, 0);
    theScene->insert(obj2);

    auto obj3 = viz::CBox::Create(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), false);
    obj3->setLocation(0, 8, 0);
    theScene->insert(obj3);
  }

  win.setCameraZoom(25);
  win.unlockAccess3DScene();

  // Wait for the GL context to be ready before installing the custom shader
  win.wait_for_GL_context();

  // Install the custom shader on the GL thread via ShaderProgramManager
  win.sendFunctionWithShaderManager([](mrpt::opengl::ShaderProgramManager& mgr)
                                    { installCustomShader(mgr); });

  win.repaint();

  std::cout << "Close the window to end.\n";
  while (win.isOpen())
  {
    std::this_thread::sleep_for(50ms);
    win.repaint();
  }
}

// ---------------------------------------------------------------------------
int main()
{
  try
  {
    DemoCustomShaders();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
