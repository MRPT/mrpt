/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/CQtGlCanvasBase.h>

#if MRPT_HAS_Qt5
#include <QMouseEvent>

using namespace mrpt;
using namespace mrpt::gui;

CQtGlCanvasBase::CQtGlCanvasBase(QWidget* parent)
	: QOpenGLWidget(parent), mrpt::gui::CGlCanvasBase()

{
	m_mainViewport = getOpenGLSceneRef()->getViewport("main");
	setMouseTracking(true);
}

void CQtGlCanvasBase::initializeGL()
{
	clearColors();

	QOpenGLWidget::initializeGL();
}

void CQtGlCanvasBase::paintGL() { renderCanvas(); }
void CQtGlCanvasBase::resizeGL(int width, int height)
{
	if (height == 0) height = 1;
	glViewport(0, 0, width, height);

	QOpenGLWidget::resizeGL(width, height);
}

opengl::COpenGLViewport::Ptr CQtGlCanvasBase::mainViewport() const
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
	m_isPressRMouseButton = (event->button() == Qt::RightButton);

	QOpenGLWidget::mousePressEvent(event);
}

void CQtGlCanvasBase::mouseMoveEvent(QMouseEvent* event)
{
	if (m_isPressLMouseButton ||
		m_isPressRMouseButton)  // QApplication::keyboardModifiers()//
	// event->modifiers() == Qt::ControlModifier
	// Qt::ShiftModifier
	{
		int X = event->pos().x();
		int Y = event->pos().y();
		updateLastPos(X, Y);

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
	m_isPressRMouseButton = false;
	QOpenGLWidget::mouseReleaseEvent(event);
}

void CQtGlCanvasBase::wheelEvent(QWheelEvent* event)
{
	CamaraParams params = cameraParams();
	updateZoom(params, event->delta());
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

bool CQtGlCanvasBase::isPressRMouseButton() const
{
	return m_isPressRMouseButton;
}

void CQtGlCanvasBase::unpressMouseButtons()
{
	m_isPressLMouseButton = false;
	m_isPressRMouseButton = false;
}

#endif  // MRPT_HAS_Qt5
