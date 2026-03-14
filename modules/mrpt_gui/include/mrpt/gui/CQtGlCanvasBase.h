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
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/gui/config.h>
#include <mrpt/viz/COrbitCameraController.h>

#if MRPT_HAS_Qt5

#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 4, 0)
#include <QGLWidget>
#define QOpenGLWidget QGLWidget
#else
#include <QOpenGLWidget>
#endif

namespace mrpt::gui
{
class CQtGlCanvasBase : public QOpenGLWidget, public mrpt::gui::CGlCanvasBase
{
 public:
  CQtGlCanvasBase(QWidget* parent = nullptr);

  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;

  [[nodiscard]] mrpt::viz::Viewport::Ptr mainViewport() const;

  /** Returns the current zoom distance from the orbit camera controller. */
  [[nodiscard]] float getCameraZoomDistance() const;

  /** Direct access to the orbit camera controller.
   *  Use this to read or write azimuth, elevation, zoom, pointing-at, etc.
   */
  [[nodiscard]] mrpt::viz::COrbitCameraController& cameraController()
  {
    return orbitCameraController();
  }
  [[nodiscard]] const mrpt::viz::COrbitCameraController& cameraController() const
  {
    return orbitCameraController();
  }

 protected:
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

  void swapBuffers() override {}
  void preRender() override {}
  void postRender() override {}
  void renderError(const std::string& err_msg) override;

  virtual void insertToMap(const viz::CVisualObject::Ptr& newObject);
  virtual void removeFromMap(const viz::CVisualObject::Ptr& newObject);

 private:
  mrpt::viz::Viewport::Ptr m_mainViewport;

};  // end of class

}  // namespace mrpt::gui
#endif  // MRPT_HAS_Qt5