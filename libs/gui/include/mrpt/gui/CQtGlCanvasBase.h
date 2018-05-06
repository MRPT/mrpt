/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/gui/CGlCanvasBase.h>

#include <mrpt/config.h>
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
	virtual ~CQtGlCanvasBase() = default;

	virtual void initializeGL() override;
	virtual void paintGL() override;
	virtual void resizeGL(int width, int height) override;

	mrpt::opengl::COpenGLViewport::Ptr mainViewport() const;

	/** Returns the zoom distance of the camera
	 * See also setZoomDistance(float), getZoomDistance()*/
	float getCameraZoomDistance() const;

   protected:
	virtual void mousePressEvent(QMouseEvent* event) override;
	virtual void mouseMoveEvent(QMouseEvent* event) override;
	virtual void mouseReleaseEvent(QMouseEvent* event) override;
	virtual void wheelEvent(QWheelEvent* event) override;

	virtual void swapBuffers() override {}
	virtual void preRender() override {}
	virtual void postRender() override {}
	virtual void renderError(const std::string& err_msg) override;

	virtual void updateCamerasParams();
	virtual void insertToMap(const opengl::CRenderizable::Ptr& newObject);
	virtual void removeFromMap(const opengl::CRenderizable::Ptr& newObject);

	bool isPressLMouseButton() const;
	bool isPressRMouseButton() const;
	/** m_isPressLMouseButton and m_isPressRMouseButton are saved in
	 * mousePressEvent for mouseMoveEvent as true
	 * This function sets it as false */
	void unpressMouseButtons();

   private:
	bool m_isPressLMouseButton;
	bool m_isPressRMouseButton;

	mrpt::opengl::COpenGLViewport::Ptr m_mainViewport;

};  // end of class

}
#endif  // MRPT_HAS_Qt5


