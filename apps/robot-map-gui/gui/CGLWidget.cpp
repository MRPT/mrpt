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

#include "CDocument.h"

#include "cmath"

#include <QMouseEvent>
#include <QApplication>
#include <QDebug>


using namespace mrpt;
using namespace mrpt::opengl;

CGlWidget::CGlWidget(QWidget *parent)
	: CQtGlCanvasBase(parent)
	, m_observationSize(10.)
	, m_doc(nullptr)
	, m_isShowObs(false)
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

CGlWidget::~CGlWidget()
{
	m_visiblePoints.clear();
}

void CGlWidget::fillMap(const CSetOfObjects::Ptr &renderizableMap)
{
	COpenGLViewport::Ptr view = m_openGLScene->getViewport("main");
	ASSERT_(view);
	view->insert(renderizableMap);

	m_map = renderizableMap;

	if (m_isShowObs)
		setSelectedObservation(m_isShowObs);
	else
		update();
}

void CGlWidget::setSelected(const math::TPose3D &pose)
{
	if (!m_map)
		return;

	CPointCloud::Ptr points = CPointCloud::Create();
	points->insertPoint(pose.x, pose.y, pose.z);
	points->setColor(mrpt::utils::TColorf(mrpt::utils::TColor::red));
	points->setPointSize(10.);
	m_map->insert(points);

	update();
}

void CGlWidget::setSelectedObservation(bool is)
{
	m_isShowObs = is;
	if (!m_doc ||!m_map)
		return;

	if (is)
		for (auto iter = m_doc->simplemap().begin(); iter != m_doc->simplemap().end(); ++iter)
		{
			CPointCloud::Ptr points = CPointCloud::Create();
			math::TPose3D pose = iter->first->getMeanVal();
			points->insertPoint(pose.x, pose.y, pose.z);
			points->setColor(mrpt::utils::TColorf(mrpt::utils::TColor::red));
			points->setPointSize(m_observationSize);

			m_map->insert(points);
			m_visiblePoints.push_back(points);
		}
	else
	{
		for (auto &iter: m_visiblePoints)
			m_map->removeObject(iter);
		m_visiblePoints.clear();
	}

	update();
}

void CGlWidget::setDocument(CDocument *doc)
{
	m_doc = doc;
	if (m_isShowObs)
		setSelectedObservation(m_isShowObs);
}
