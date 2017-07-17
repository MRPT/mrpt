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
#include "mrpt/opengl/CTexturedPlane.h"
#include "mrpt/gui/CGlCanvasBase.h"
#include "mrpt/opengl/stock_objects.h"
#include "mrpt/math/geometry.h"

#include "CDocument.h"

#include "cmath"

#include <QMouseEvent>
#include <QApplication>
#include <QDebug>


using namespace mrpt;
using namespace mrpt::opengl;

CGlWidget::CGlWidget(bool is2D, QWidget *parent)
	: CQtGlCanvasBase(parent)
	, m_doc(nullptr)
	, m_miniMapSize(-1.0)
	, m_minimapPercentSize(0.25)
	, m_observationSize(10.)
	, m_isShowObs(false)
	, m_is2D(is2D)
{
	clearColorR = 1.0;
	clearColorG = 1.0;
	clearColorB = 1.0;


	if (m_is2D)
	{
		m_miniMapViewport = m_openGLScene->createViewport("miniMap");
		m_miniMapViewport->setBorderSize(2);
		m_miniMapViewport->setViewportPosition(0.01,0.01, m_minimapPercentSize, m_minimapPercentSize);
		m_miniMapViewport->setTransparent(false);

		setAzimuthDegrees(-90);
		setElevationDegrees(90);

		CCamera &camMiniMap = m_miniMapViewport->getCamera();
		updateCameraParams(camMiniMap);
		camMiniMap.setOrthogonal();
	}

	updateCamerasParams();
	// The ground:
	mrpt::opengl::CGridPlaneXY::Ptr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
	groundPlane->setColor(0.4,0.4,0.4);
	insertToMap(groundPlane);
}

CGlWidget::~CGlWidget()
{
	m_visiblePoints.clear();
}

void CGlWidget::fillMap(const CSetOfObjects::Ptr &renderizableMap)
{
	insertToMap(renderizableMap);
	m_map = renderizableMap;
	float xMin = 0;
	float xMax = 0;
	float yMin = 0;
	float yMax = 0;

	if (m_is2D)
	{
		if (m_map->size() == 1)
		{
			CRenderizable *ren = m_map->begin()->get();
			CTexturedPlane *textured = dynamic_cast<CTexturedPlane *>(ren);
			if (textured)
			{
				textured->getPlaneCorners(xMin, xMax, yMin, yMax);
			}
			else
			{
				CPointCloud *points = dynamic_cast<CPointCloud *>(ren);
				if (points)
				{
					const std::vector<float> &arrayX = points->getArrayX();
					const std::vector<float> &arrayY = points->getArrayY();

					for (auto &itX: arrayX)
					{
						xMin = std::min(xMin, itX);
						xMax = std::max(xMax, itX);
					}
					for (auto &itY: arrayY)
					{
						yMin = std::min(yMin, itY);
						yMax = std::max(yMax, itY);
					}
				}

			}
		}

		float xDist = xMax - xMin;
		float yDist = yMax - yMin;

		CCamera &camMiniMap = m_miniMapViewport->getCamera();
		updateCameraParams(camMiniMap);
		camMiniMap.setZoomDistance(std::max(xDist, yDist));
		updateCamerasParams();
	}

	if (m_isShowObs)
		setSelectedObservation(m_isShowObs);
	else
		update();
}

void CGlWidget::setSelected(const math::TPose3D &pose)
{
	if (!m_map)
		return;

	if (m_currentObs)
		m_map->removeObject(m_currentObs);

	if (m_currentLaserScan)
		m_map->removeObject(m_currentLaserScan);

	m_currentObs = opengl::stock_objects::CornerXYZSimple();
	m_currentObs->setPose( pose );
	m_map->insert(m_currentObs);

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

void CGlWidget::setLaserScan(CPlanarLaserScan::Ptr laserScan)
{
	if (m_currentLaserScan)
		m_map->removeObject(m_currentLaserScan);

	m_currentLaserScan = laserScan;
	m_map->insert(m_currentLaserScan);

	update();
}

void CGlWidget::setDocument(CDocument *doc)
{
	m_doc = doc;
	if (m_isShowObs)
		setSelectedObservation(m_isShowObs);
}

void CGlWidget::setZoom(float zoom)
{
	CamaraParams params = cameraParams();
	params.cameraZoomDistance = zoom;
	setCameraParams(params);
	updateCamerasParams();

	update();

}

float CGlWidget::getZoom() const
{
	return getCameraZoomDistance();
}

void CGlWidget::resizeGL(int width, int height)
{
	CQtGlCanvasBase::resizeGL(width, height);

	if (m_is2D)
	{
		GLint	win_dims[4];
		glGetIntegerv( GL_VIEWPORT, win_dims );
		m_miniMapSize = std::min(win_dims[2], win_dims[3]) * m_minimapPercentSize;
	}
	updateMinimapPos();
}

void CGlWidget::updateCamerasParams()
{
	float zoom = getCameraZoomDistance();
	CQtGlCanvasBase::updateCamerasParams();

	if (m_is2D)
	{
		CCamera &camMiniMap = m_miniMapViewport->getCamera();
		float zoomMiniMap = camMiniMap.getZoomDistance();
		updateCameraParams(camMiniMap);
		camMiniMap.setProjectiveModel(false);
		camMiniMap.setZoomDistance(zoomMiniMap);
	}

	if (zoom != getCameraZoomDistance())
		emit zoomChanged(getCameraZoomDistance());

}

void CGlWidget::insertToMap(const CRenderizable::Ptr &newObject)
{
	CQtGlCanvasBase::insertToMap(newObject);
	if (m_is2D)
	{
		assert(m_miniMapViewport);
		m_miniMapViewport->insert(newObject);
	}
}

void CGlWidget::mouseMoveEvent(QMouseEvent *event)
{
	CQtGlCanvasBase::mouseMoveEvent(event);
	QPoint pos = event->pos();
	mrpt::math::TLine3D outRay;
	mainViewport()->get3DRayForPixelCoord(pos.x(), pos.y(), outRay);

	const mrpt::math::TPlane ground_plane(mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(1, 0, 0), mrpt::math::TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(outRay, ground_plane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	mrpt::math::TPoint3D inters_pt;
	if (inters.getPoint(inters_pt))
	{
		emit mousePosChanged(inters_pt.x, inters_pt.y);
	}
}

void CGlWidget::updateMinimapPos()
{
	if (!m_is2D)
		return;

	GLint	win_dims[4];
	glGetIntegerv( GL_VIEWPORT, win_dims );

	COpenGLViewport::Ptr miniMap = m_openGLScene->getViewport("miniMap");
	float w = m_miniMapSize/win_dims[2];
	float h = m_miniMapSize/win_dims[3];
	miniMap->setViewportPosition(0.01,0.01, w, h);

}
