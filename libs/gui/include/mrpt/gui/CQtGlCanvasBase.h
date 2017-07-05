/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QGLWidget>
#include <mrpt/gui/CGlCanvasBase.h>

namespace mrpt
{
namespace gui
{
class CQtGlCanvasBase: public QGLWidget, public mrpt::gui::CGlCanvasBase
{

public:
	CQtGlCanvasBase(QWidget* parent = nullptr);
	virtual ~CQtGlCanvasBase() = default;

	virtual void initializeGL() override;
	virtual void paintGL() override;
	virtual void resizeGL(int width, int height) override;

protected:
	virtual void mousePressEvent(QMouseEvent *event) override;
	virtual void mouseMoveEvent(QMouseEvent *event) override;
	virtual void mouseReleaseEvent(QMouseEvent *event) override;
	virtual void wheelEvent(QWheelEvent *event) override;

	virtual void swapBuffers() override {}
	virtual void preRender() override {}
	virtual void postRender() override {}
	virtual void renderError(const std::string &err_msg) override;

private:
	bool m_isPressLMouseButton;
	bool m_isPressRMouseButton;

};// end of class

}	// end namespace
}	// end namespace

