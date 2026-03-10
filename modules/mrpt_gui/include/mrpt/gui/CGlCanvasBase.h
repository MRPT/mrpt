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

#pragma once

#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/viz/COrbitCameraController.h>
#include <mrpt/viz/Scene.h>

namespace mrpt::viz
{
class CCamera;
}  // namespace mrpt::viz

namespace mrpt::gui
{
/** This base class implements a working with opengl::Camera and a OpenGL
 * canvas, and it's used in gui::CWxGLCanvasBase and gui::CQtGlCanvasBase.
 * \ingroup mrpt_gui_grp
 */
class CGlCanvasBase
{
 public:
  CGlCanvasBase() = default;
  virtual ~CGlCanvasBase();

  CGlCanvasBase(const CGlCanvasBase&) = delete;
  CGlCanvasBase& operator=(const CGlCanvasBase&) = delete;
  CGlCanvasBase(CGlCanvasBase&&) noexcept = default;
  CGlCanvasBase& operator=(CGlCanvasBase&&) noexcept = default;

  /** Sets the minimum of the zoom
   * See also setMaximumZoom(float) */
  void setMinimumZoom(float zoom);

  /** Sets the maximum of the zoom
   * See also setMinimumZoom(float) */
  void setMaximumZoom(float zoom);

  /** Saves the click position of the mouse
   * See also setMouseClicked(bool) */
  void setMousePos(int x, int y);

  /** Sets the property mouseClicked
   * By default, this property is false.
   * See also setMousePos(int, int) */
  void setMouseClicked(bool is);

  /** Sets the last mouse position */
  void updateLastPos(int x, int y);

  /** Calls the glViewport function*/
  void resizeViewport(int w, int h);

  [[nodiscard]] const mrpt::viz::COrbitCameraController& orbitCameraController() const
  {
    return m_cameraCtrl;
  }
  [[nodiscard]] mrpt::viz::COrbitCameraController& orbitCameraController() { return m_cameraCtrl; }

  /** If set to true (default=false), the cameraPointingX,... parameters are
   * ignored and the camera stored in the 3D scene is used instead.
   * See also `bool getUseCameraFromScene()`
   */
  void setUseCameraFromScene(bool is);

  /** See also void setUseCameraFromScene(bool)
   */
  [[nodiscard]] bool getUseCameraFromScene() const;

  void getLastMousePosition(int& x, int& y) const
  {
    x = m_mouseLastX;
    y = m_mouseLastY;
  }

  virtual void onCameraChanged(const mrpt::viz::COrbitCameraController&) {}

  /**  At constructor an empty scene is created. The object is freed at GL
  canvas destructor.
   * This function returns a smart pointer to the opengl scene
  getOpenGLSceneRef		  */
  mrpt::viz::Scene::Ptr& getOpenGLSceneRef() { return m_openGLScene; }

  void setOpenGLSceneRef(mrpt::viz::Scene::Ptr scene);

  /** Returns the shader manager for the current compiled scene, or nullptr
   * if the scene has not been compiled yet.
   * \note Must be called from the OpenGL context thread. */
  mrpt::opengl::ShaderProgramManager* getShaderManager()
  {
    return m_compiledScene ? &m_compiledScene->shaderManager() : nullptr;
  }

 protected:
  virtual void swapBuffers() = 0;
  virtual void preRender() = 0;
  virtual void postRender() = 0;
  virtual void renderError(const std::string& err_msg) = 0;

  virtual double renderCanvas(int width = -1, int height = -1);

 private:
  bool useCameraFromScene = false;
  mrpt::viz::Scene::Ptr m_openGLScene = mrpt::viz::Scene::Create();
  std::unique_ptr<mrpt::opengl::CompiledScene> m_compiledScene;
  std::weak_ptr<mrpt::viz::Scene> m_lastCompiledScenePtr;
  int m_mouseLastX = 0, m_mouseLastY = 0;
  int m_mouseClickX = 0, m_mouseClickY = 0;
  bool mouseClicked = false;

  mrpt::viz::COrbitCameraController m_cameraCtrl;

};  // end of class

/** A headless dummy implementation of CGlCanvasBase: can be used to keep track
 * of user UI mouse events and update the camera parameters, with actual
 * rendering being delegated to someone else. \ingroup mrpt_gui_grp
 */
class CGlCanvasBaseHeadless : public CGlCanvasBase
{
 public:
  CGlCanvasBaseHeadless() = default;

 protected:
  void swapBuffers() override {}
  void preRender() override {}
  void postRender() override {}
  void renderError(const std::string& e) override;
};

}  // namespace mrpt::gui
