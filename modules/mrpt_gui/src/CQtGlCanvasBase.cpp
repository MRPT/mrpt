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

#include <mrpt/gui/CQtGlCanvasBase.h>
#include <mrpt/opengl/config.h>  // MRPT_HAS_OPENGL_GLUT

#if MRPT_HAS_Qt5
#include <QMouseEvent>

using namespace mrpt;
using namespace mrpt::gui;

// -----------------------------------------------------------------------
// Internal helpers: Qt → mrpt::viz::COrbitCameraController bitmasks
// -----------------------------------------------------------------------
namespace
{
using C = mrpt::viz::COrbitCameraController;

uint8_t qtModsToMrpt(Qt::KeyboardModifiers m)
{
  uint8_t out = 0;
  if ((m & Qt::ShiftModifier) != 0)
  {
    out |= C::ModShift;
  }
  if ((m & Qt::ControlModifier) != 0)
  {
    out |= C::ModControl;
  }
  if ((m & Qt::AltModifier) != 0)
  {
    out |= C::ModAlt;
  }
  return out;
}

// For press/release events: event->button() is a single enum value.
uint8_t qtSingleButtonToMrpt(Qt::MouseButton b)
{
  switch (b)
  {
    case Qt::LeftButton:
      return C::ButtonLeft;
    case Qt::MiddleButton:
      return C::ButtonMiddle;
    case Qt::RightButton:
      return C::ButtonRight;
    default:
      return 0;
  }
}

// For move events: event->buttons() is a bitmask of all held buttons.
uint8_t qtButtonsToMrpt(Qt::MouseButtons b)
{
  uint8_t out = 0;
  if ((b & Qt::LeftButton) != 0)
  {
    out |= C::ButtonLeft;
  }
  if ((b & Qt::MiddleButton) != 0)
  {
    out |= C::ButtonMiddle;
  }
  if ((b & Qt::RightButton) != 0)
  {
    out |= C::ButtonRight;
  }
  return out;
}

}  // namespace

// -----------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------
CQtGlCanvasBase::CQtGlCanvasBase(QWidget* parent) : QOpenGLWidget(parent)
{
#if MRPT_HAS_OPENGL_GLUT
  m_mainViewport = getOpenGLSceneRef()->getViewport("main");
  setMouseTracking(true);
#else
  std::cerr << "[mrpt::gui::CQtGlCanvasBase] *Warning*: MRPT built without "
               "OpenGL support."
            << "\n";
#endif
}

// -----------------------------------------------------------------------
// OpenGL lifecycle
// -----------------------------------------------------------------------
void CQtGlCanvasBase::initializeGL()
{
#if MRPT_HAS_OPENGL_GLUT
  QOpenGLWidget::initializeGL();
#endif
}

void CQtGlCanvasBase::paintGL()
{
#if MRPT_HAS_OPENGL_GLUT
  // Sync the orbit controller into the scene camera before every frame.
  if (m_mainViewport)
  {
    mrpt::viz::CCamera& cam = m_mainViewport->getCamera();
    orbitCameraController().applyTo(cam);
  }
#endif
  renderCanvas();
}

void CQtGlCanvasBase::resizeGL(int width, int height)
{
#if MRPT_HAS_OPENGL_GLUT
  if (height == 0)
  {
    height = 1;
  }
  glViewport(0, 0, width, height);
  QOpenGLWidget::resizeGL(width, height);
#endif
}

// -----------------------------------------------------------------------
// Scene / viewport helpers (unchanged)
// -----------------------------------------------------------------------
viz::Viewport::Ptr CQtGlCanvasBase::mainViewport() const { return m_mainViewport; }

float CQtGlCanvasBase::getCameraZoomDistance() const
{
  return orbitCameraController().getZoomDistance();
}

void CQtGlCanvasBase::insertToMap(const viz::CVisualObject::Ptr& newObject)
{
  assert(m_mainViewport);
  m_mainViewport->insert(newObject);
}

void CQtGlCanvasBase::removeFromMap(const viz::CVisualObject::Ptr& newObject)
{
  assert(m_mainViewport);
  m_mainViewport->removeObject(newObject);
}

// -----------------------------------------------------------------------
// Mouse events
// -----------------------------------------------------------------------
void CQtGlCanvasBase::mousePressEvent(QMouseEvent* event)
{
  orbitCameraController().onMouseButton(
      event->pos().x(), event->pos().y(), qtSingleButtonToMrpt(event->button()),
      /*down=*/true);

  QOpenGLWidget::mousePressEvent(event);
}

void CQtGlCanvasBase::mouseReleaseEvent(QMouseEvent* event)
{
  orbitCameraController().onMouseButton(
      event->pos().x(), event->pos().y(), qtSingleButtonToMrpt(event->button()),
      /*down=*/false);

  QOpenGLWidget::mouseReleaseEvent(event);
}

void CQtGlCanvasBase::mouseMoveEvent(QMouseEvent* event)
{
  const auto buttons = qtButtonsToMrpt(event->buttons());

  if (buttons != 0)
  {
    orbitCameraController().onMouseMove(
        event->pos().x(), event->pos().y(), buttons, qtModsToMrpt(event->modifiers()));
    update();
  }

  QOpenGLWidget::mouseMoveEvent(event);
}

void CQtGlCanvasBase::wheelEvent(QWheelEvent* event)
{
  // Qt gives angleDelta in eighths of a degree; 120 eighths == 1 standard
  // scroll notch. Normalise to ±1 so all backends use the same scale.
  const float delta = static_cast<float>(event->angleDelta().y()) / 120.0f;
  orbitCameraController().onScroll(delta, qtModsToMrpt(event->modifiers()));
  update();
  QOpenGLWidget::wheelEvent(event);
}

// -----------------------------------------------------------------------
// Misc overrides
// -----------------------------------------------------------------------
void CQtGlCanvasBase::renderError(const std::string& err_msg) { Q_UNUSED(err_msg); }

#endif  // MRPT_HAS_Qt5