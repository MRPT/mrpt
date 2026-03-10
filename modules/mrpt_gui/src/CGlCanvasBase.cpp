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
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/opengl/config.h>  // MRPT_HAS_OPENGL_GLUT
#include <mrpt/opengl/opengl_api.h>

#include <cstdlib>
#include <iostream>

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// Windows:
#include <windows.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif
#endif  // MRPT_HAS_OPENGL_GLUT

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::viz;
using namespace std;

CGlCanvasBase::~CGlCanvasBase()
{
  // Ensure all OpenGL resources are freed before the opengl context is gone:
  m_compiledScene.reset();
}

void CGlCanvasBase::resizeViewport(int w, int h)  // NOLINT
{
#if MRPT_HAS_OPENGL_GLUT
  if (w == -1 || h == -1)
  {
    return;
  }

  glViewport(0, 0, static_cast<GLint>(w), static_cast<GLint>(h));
#endif
}

void CGlCanvasBase::setUseCameraFromScene(bool is) { useCameraFromScene = is; }
bool CGlCanvasBase::getUseCameraFromScene() const { return useCameraFromScene; }

void CGlCanvasBase::setOpenGLSceneRef(Scene::Ptr scene) { m_openGLScene = std::move(scene); }

double CGlCanvasBase::renderCanvas(int width, int height)
{
#if MRPT_HAS_OPENGL_GLUT
  double At = 0.1;

#ifdef MRPT_OPENGL_PROFILER
  mrpt::system::CTimeLoggerEntry tle(opengl_profiler(), "renderCanvas");
#endif

  try
  {
    const double t0 = mrpt::Clock::nowDouble();

#ifdef MRPT_OPENGL_PROFILER
    mrpt::system::CTimeLoggerEntry tle1(opengl_profiler(), "renderCanvas.1_preRender");
#endif

    // Call PreRender user code:
    preRender();
    CHECK_OPENGL_ERROR();

#ifdef MRPT_OPENGL_PROFILER
    tle1.stop();
    mrpt::system::CTimeLoggerEntry tle2(opengl_profiler(), "renderCanvas.2_resizeViewport");
#endif

    // Set the viewport
    resizeViewport(static_cast<GLsizei>(width), static_cast<GLsizei>(height));

#ifdef MRPT_OPENGL_PROFILER
    tle2.stop();
    mrpt::system::CTimeLoggerEntry tle3(opengl_profiler(), "renderCanvas.3_renderScene");
#endif

    if (m_openGLScene)
    {
      // Set the camera params in the scene:
      if (!useCameraFromScene)
      {
        if (Viewport::Ptr view = m_openGLScene->getViewport("main"); view)
        {
          mrpt::viz::CCamera& cam = view->getCamera();
          m_cameraCtrl.applyTo(cam);
        }
        else
        {
          std::cerr << "[CGlCanvasBase::renderCanvas] Warning: there "
                       "is no 'main' viewport in the 3D scene!"
                    << "\n";
        }
      }

      // Compile/update and render via CompiledScene:
      auto scenePtr = m_openGLScene;
      auto lastPtr = m_lastCompiledScenePtr.lock();
      if (!m_compiledScene || lastPtr.get() != scenePtr.get())
      {
        m_compiledScene = std::make_unique<mrpt::opengl::CompiledScene>();
        m_compiledScene->compile(*m_openGLScene);
        m_lastCompiledScenePtr = scenePtr;
      }
      else
      {
        m_compiledScene->updateIfNeeded();
      }
      m_compiledScene->render(width, height);

    }  // end if "m_openGLScene!=nullptr"

#ifdef MRPT_OPENGL_PROFILER
    tle3.stop();
    mrpt::system::CTimeLoggerEntry tle4(opengl_profiler(), "renderCanvas.4_postRender");
#endif

    postRender();

    // Swap buffers to disply new image:
    // Was: glFinish();
    // It's not actually required and we return from this function faster.

#ifdef MRPT_OPENGL_PROFILER
    tle4.stop();
    mrpt::system::CTimeLoggerEntry tle5(opengl_profiler(), "renderCanvas.5_swapBuffers");
#endif

    swapBuffers();
    CHECK_OPENGL_ERROR();

#ifdef MRPT_OPENGL_PROFILER
    tle5.stop();
#endif

    const double t1 = mrpt::Clock::nowDouble();

    At = t1 - t0;
  }
  catch (const std::exception& e)
  {
    const std::string err_msg =
        std::string("[CGLCanvasBase::Render] Exception:\n") + mrpt::exception_to_str(e);
    std::cerr << err_msg;
    renderError(err_msg);
  }

  return At;
#else
  THROW_EXCEPTION("Cant render: MRPT was built without OpenGL");
#endif
}
