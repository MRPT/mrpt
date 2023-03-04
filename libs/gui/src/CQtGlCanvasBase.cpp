/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/gui/CQtGlCanvasBase.h>

#if MRPT_HAS_Qt5
#include <QMouseEvent>

using namespace mrpt;
using namespace mrpt::gui;

CQtGlCanvasBase::CQtGlCanvasBase(QWidget* parent)
	: QOpenGLWidget(parent), mrpt::gui::CGlCanvasBase()
{
#if MRPT_HAS_OPENGL_GLUT
	m_mainViewport = getOpenGLSceneRef()->getViewport("main");
	setMouseTracking(true);
#else
	std::cerr << "[mrpt::gui::CQtGlCanvasBase] *Warning*: MRPT built without "
				 "OpenGL support."
			  << std::endl;
#endif
}

void CQtGlCanvasBase::initializeGL()
{
#if MRPT_HAS_OPENGL_GLUT

	QOpenGLWidget::initializeGL();
#endif
}

void CQtGlCanvasBase::paintGL() { renderCanvas(); }
void CQtGlCanvasBase::resizeGL(int width, int height)
{
#if MRPT_HAS_OPENGL_GLUT
	if (height == 0) height = 1;
	glViewport(0, 0, width, height);

	QOpenGLWidget::resizeGL(width, height);
#endif
}

opengl::Viewport::Ptr CQtGlCanvasBase::mainViewport() const
{
	return m_mainViewport;
}

float CQtGlCanvasBase::getCameraZoomDistance() const
{
	mrpt::opengl::CCamera& cam = m_mainViewport->getCamera();
	return cam.getZoomDistance();
}

void CQtGlCanvasBase::mousePressEvent(QMouseEvent* event)
{
	setMousePos(event->pos().x(), event->pos().y());
	setMouseClicked(true);

	m_isPressLMouseButton = (event->button() == Qt::LeftButton);
	m_isPressMMouseButton = (event->button() == Qt::MiddleButton);
	m_isPressRMouseButton = (event->button() == Qt::RightButton);

	QOpenGLWidget::mousePressEvent(event);
}

void CQtGlCanvasBase::mouseMoveEvent(QMouseEvent* event)
{
	int X = event->pos().x();
	int Y = event->pos().y();
	updateLastPos(X, Y);

	if (m_isPressLMouseButton || m_isPressRMouseButton || m_isPressMMouseButton)
	{
		// Proxy variables to cache the changes:
		CamaraParams params = cameraParams();

		if (m_isPressLMouseButton)
		{
			if (event->modifiers() == Qt::ShiftModifier)
				updateZoom(params, X, Y);

			else if (event->modifiers() == Qt::ControlModifier)
				updateRotate(params, X, Y);

			else if (event->modifiers() == Qt::NoModifier)
				updateOrbitCamera(params, X, Y);
		}
		else
			updatePan(params, X, Y);

		setMousePos(X, Y);
		setCameraParams(params);

		updateCamerasParams();
		update();
	}

	QOpenGLWidget::mouseMoveEvent(event);
}

void CQtGlCanvasBase::mouseReleaseEvent(QMouseEvent* event)
{
	setMouseClicked(false);
	m_isPressLMouseButton = false;
	m_isPressMMouseButton = false;
	m_isPressRMouseButton = false;
	QOpenGLWidget::mouseReleaseEvent(event);
}

void CQtGlCanvasBase::wheelEvent(QWheelEvent* event)
{
	CamaraParams params = cameraParams();
	if (event->modifiers() != Qt::ShiftModifier)
	{
		// regular zoom:
		updateZoom(params, event->angleDelta().y());
	}
	else
	{
		// Move vertically +-Z:
		params.cameraPointingZ +=
			event->angleDelta().y() * params.cameraZoomDistance * 1e-4;
	}

	setCameraParams(params);

	updateCamerasParams();

	update();
	QOpenGLWidget::wheelEvent(event);
}

void CQtGlCanvasBase::renderError(const std::string& err_msg)
{
	Q_UNUSED(err_msg);
}

void CQtGlCanvasBase::updateCamerasParams()
{
	mrpt::opengl::CCamera& cam = m_mainViewport->getCamera();
	updateCameraParams(cam);
}

void CQtGlCanvasBase::insertToMap(const opengl::CRenderizable::Ptr& newObject)
{
	assert(m_mainViewport);
	m_mainViewport->insert(newObject);
}

void CQtGlCanvasBase::removeFromMap(const opengl::CRenderizable::Ptr& newObject)
{
	assert(m_mainViewport);
	m_mainViewport->removeObject(newObject);
}

bool CQtGlCanvasBase::isPressLMouseButton() const
{
	return m_isPressLMouseButton;
}

bool CQtGlCanvasBase::isPressMMouseButton() const
{
	return m_isPressMMouseButton;
}

bool CQtGlCanvasBase::isPressRMouseButton() const
{
	return m_isPressRMouseButton;
}

void CQtGlCanvasBase::unpressMouseButtons()
{
	m_isPressLMouseButton = false;
	m_isPressRMouseButton = false;
	m_isPressMMouseButton = false;
}

#endif	// MRPT_HAS_Qt5
