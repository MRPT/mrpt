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

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>
#include <mrpt/opengl/config.h>  // MRPT_HAS_OPENGL_GLUT

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

MRPT2NanoguiGLCanvas::MRPT2NanoguiGLCanvas(nanogui::Widget* parent) : nanogui::GLCanvas(parent) {}

void MRPT2NanoguiGLCanvas::drawGL()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  std::lock_guard<std::mutex> lck(scene_mtx);

  try
  {
    // Set the background color:
    mrpt::img::TColorf m_backgroundColor = {0.7f, 0.7f, 0.7f, 1.0f};
    glClearColor(
        m_backgroundColor.R, m_backgroundColor.G, m_backgroundColor.B, m_backgroundColor.A);

    if (!scene)
    {
      return;  // No scene -> nothing to render
    }

    // We need the size of the viewport:
    GLint win_dims[4];
    glGetIntegerv(GL_VIEWPORT, win_dims);

    // Set the camera params in the scene:
    mrpt::viz::Viewport::Ptr view = scene->getViewport("main");
    if (!view)
    {
      THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");
    }
    mrpt::viz::CCamera& cam = view->getCamera();

    m_cameraCtrl.applyTo(cam);

    // Compile/update and render via CompiledScene:
    auto scenePtr = scene;
    auto lastPtr = m_lastCompiledScenePtr.lock();
    if (!m_compiledScene || lastPtr.get() != scenePtr.get())
    {
      m_compiledScene = std::make_unique<mrpt::opengl::CompiledScene>();
      m_compiledScene->compile(*scene);
      m_lastCompiledScenePtr = scenePtr;
    }
    else
    {
      m_compiledScene->updateIfNeeded();
    }
    m_compiledScene->render(win_dims[2], win_dims[3], win_dims[0], win_dims[1]);
  }
  catch (const std::exception& e)
  {
    std::cerr << "[MRPT2NanoguiGLCanvas::drawGL] Exception:\n" << mrpt::exception_to_str(e);
  }
#endif
}

namespace
{
uint8_t nanoguiButtonsToMrpt(int b)
{
  using C = mrpt::viz::COrbitCameraController;
  uint8_t out = 0;
  if ((b & (1 << GLFW_MOUSE_BUTTON_LEFT)) != 0)
  {
    out |= C::ButtonLeft;
  }
  if ((b & (1 << GLFW_MOUSE_BUTTON_MIDDLE)) != 0)
  {
    out |= C::ButtonMiddle;
  }
  if ((b & (1 << GLFW_MOUSE_BUTTON_RIGHT)) != 0)
  {
    out |= C::ButtonRight;
  }
  return out;
}

uint8_t nanoguiModsToMrpt(int m)
{
  using C = mrpt::viz::COrbitCameraController;
  uint8_t out = 0;
  if ((m & GLFW_MOD_SHIFT) != 0)
  {
    out |= C::ModShift;
  }
  if ((m & GLFW_MOD_CONTROL) != 0)
  {
    out |= C::ModControl;
  }
  if ((m & GLFW_MOD_ALT) != 0)
  {
    out |= C::ModAlt;
  }
  return out;
}
}  // namespace

bool MRPT2NanoguiGLCanvas::mouseMotionEvent(
    const nanogui::Vector2i& p, const nanogui::Vector2i& /*rel*/, int button, int modifiers)
{
  m_lastModifiers = nanoguiModsToMrpt(modifiers);
  m_cameraCtrl.onMouseMove(p.x(), p.y(), nanoguiButtonsToMrpt(button), m_lastModifiers);
  return true;
}

bool MRPT2NanoguiGLCanvas::mouseButtonEvent(
    const nanogui::Vector2i& p, int button, bool down, int modifiers)
{
  m_lastModifiers = nanoguiModsToMrpt(modifiers);
  // nanogui passes a single button id, not a bitmask, on press/release
  using C = mrpt::viz::COrbitCameraController;
  uint8_t btn = 0;
  if (button == GLFW_MOUSE_BUTTON_LEFT)
  {
    btn = C::ButtonLeft;
  }
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
  {
    btn = C::ButtonMiddle;
  }
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
  {
    btn = C::ButtonRight;
  }

  m_cameraCtrl.onMouseButton(p.x(), p.y(), btn, down);
  return true;
}

bool MRPT2NanoguiGLCanvas::scrollEvent(const nanogui::Vector2i& /*p*/, const nanogui::Vector2f& rel)
{
  m_cameraCtrl.onScroll(rel.y(), m_lastModifiers);
  return true;
}

#endif  // MRPT_HAS_NANOGUI
