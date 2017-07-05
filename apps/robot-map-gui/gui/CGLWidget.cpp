/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CGLWidget.h"

#include "mrpt/maps/TMetricMapInitializer.h"
#include "mrpt/utils/CConfigFile.h"
#include "mrpt/utils/CFileGZOutputStream.h"
#include "mrpt/utils/CFileGZInputStream.h"
#include "mrpt/opengl/CGridPlaneXY.h"
#include "mrpt/opengl/CPointCloud.h"

#include "mrpt/gui/CGlCanvasBase.h"

#include "cmath"

#include <QMouseEvent>
#include <QApplication>
#include <QDebug>


using namespace mrpt;
using namespace mrpt::opengl;

CGlWidget::CGlWidget(QWidget *parent)
	: CQtGlCanvasBase(parent)
{
	COpenGLViewport::Ptr view = m_openGLScene->getViewport("main");
	ASSERT_(view);

	setMouseTracking(true);
	// The ground:
	mrpt::opengl::CGridPlaneXY::Ptr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
	groundPlane->setColor(0.4,0.4,0.4);
	view->insert( groundPlane );

	// The camera pointing to the current robot pose:
	CCamera &cam = view->getCamera();
	updateCameraParams(cam);

}

void CGlWidget::fillMap(const CSetOfObjects::Ptr &renderizableMap)
{
	COpenGLViewport::Ptr view = m_openGLScene->getViewport("main");
	ASSERT_(view);
	view->insert(renderizableMap);

	m_map = renderizableMap;

	update();
}

void CGlWidget::setSelected(const math::TPose3D &pose)
{
	CPointCloud::Ptr points = CPointCloud::Create();
	points->insertPoint(pose.x, pose.y, pose.z);
	points->setColor(mrpt::utils::TColorf(mrpt::utils::TColor::red));
	points->setPointSize(10.);
	m_map->insert(points);
}
