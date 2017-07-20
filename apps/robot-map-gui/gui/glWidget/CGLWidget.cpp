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
	, m_groundPlane(mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5))
	, m_doc(nullptr)
	, m_miniMapSize(-1.0)
	, m_minimapPercentSize(0.25)
	, m_observationSize(10.)
	, m_isShowObs(false)
	, m_visiblePoints(CPointCloud::Create())
	, m_currentObs(opengl::stock_objects::CornerXYZSimple())
	, m_line(CSetOfLines::Create())
	, m_is2D(is2D)
	, m_showRobot(false)
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
	m_groundPlane->setColor(0.4,0.4,0.4);
	setVisibleGrid(true);
}

CGlWidget::~CGlWidget()
{
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

	m_map->removeObject(m_currentObs);

	if (m_currentLaserScan)
		m_map->removeObject(m_currentLaserScan);

	m_showRobot = true;

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
		m_map->insert(m_visiblePoints);

	else
		m_map->removeObject(m_visiblePoints);


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
		m_map->removeObject(m_visiblePoints);

	m_visiblePoints = CPointCloud::Create();
	m_visiblePoints->setColor(mrpt::utils::TColorf(mrpt::utils::TColor::red));
	m_visiblePoints->setPointSize(m_observationSize);
	for (auto iter = m_doc->simplemap().begin(); iter != m_doc->simplemap().end(); ++iter)
	{
		math::TPose3D pose = iter->first->getMeanVal();
		m_visiblePoints->insertPoint(pose.x, pose.y, pose.z);
	}

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
	return getZoomDistance();
}

void CGlWidget::setAzimuthDegrees(float ang)
{
	CQtGlCanvasBase::setAzimuthDegrees(ang);
	emit azimuthChanged(ang);
	updateCamerasParams();
}

void CGlWidget::setElevationDegrees(float ang)
{
	CQtGlCanvasBase::setElevationDegrees(ang);
	emit elevationChanged(ang);
	updateCamerasParams();
}

void CGlWidget::setBackgroundColor(float r, float g, float b)
{
	clearColorR = r;
	clearColorG = g;
	clearColorB = b;

	update();
}

void CGlWidget::setGridColor(double r, double g, double b, double a)
{
	m_groundPlane->setColor(r, g, b, a);
	update();
}

void CGlWidget::setVisibleGrid(bool is)
{
	if (is)
		insertToMap(m_groundPlane);
	else
		removeFromMap(m_groundPlane);

	update();
}

void CGlWidget::setBot(int value)
{
	if (!m_map)
		return;

	math::TPose3D pose;

	if (m_showRobot)
	{
		m_map->removeObject(m_currentObs);
		pose = m_currentObs->getPose();
	}

	switch (value) {
	case 0:
		m_currentObs = opengl::stock_objects::CornerXYZSimple();
		break;
	case 1:
		m_currentObs = opengl::stock_objects::CornerXYZ();
		break;
	case 2:
		m_currentObs = opengl::stock_objects::RobotGiraff();
		break;
	case 3:
		m_currentObs = opengl::stock_objects::RobotRhodon();
		break;
	case 4:
		m_currentObs = opengl::stock_objects::RobotPioneer();
		break;
	case 5:
		m_currentObs = opengl::stock_objects::BumblebeeCamera();
		break;
	default:
		m_currentObs = opengl::stock_objects::CornerXYZSimple();
		break;
	}
	if (m_showRobot)
	{
		m_currentObs->setPose( pose );
		m_map->insert(m_currentObs);
	}

	update();
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

	if (zoom != getZoomDistance())
		emit zoomChanged(getZoomDistance());

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

void CGlWidget::removeFromMap(const CRenderizable::Ptr &newObject)
{
	CQtGlCanvasBase::removeFromMap(newObject);
	if (m_is2D)
	{
		assert(m_miniMapViewport);
		m_miniMapViewport->removeObject(newObject);
	}
}

void CGlWidget::mouseMoveEvent(QMouseEvent *event)
{
	CQtGlCanvasBase::mouseMoveEvent(event);

	std::pair<bool, math::TPoint3D> scenePos = sceneToWorld(event->pos());
	if (scenePos.first)
		emit mousePosChanged(scenePos.second.x, scenePos.second.y);
}

void CGlWidget::mousePressEvent(QMouseEvent *event)
{
	CQtGlCanvasBase::mousePressEvent(event);
	if (!m_isShowObs) return;

	QPoint pos = event->pos();
	QPoint otherPos(pos.x() + m_observationSize, pos.y() + m_observationSize);

	auto scenePos = sceneToWorld(pos);
	auto sceneOtherPos = sceneToWorld(otherPos);

	if (scenePos.first && sceneOtherPos.first)
	{
		auto xs = m_visiblePoints->getArrayX();
		if (xs.empty()) return;
		auto ys = m_visiblePoints->getArrayY();
		assert(xs.size() == ys.size());

		bool foundPose = false;
		math::TPose3D clickedPose(0.0, 0.0, m_visiblePoints->getArrayZ()[0], 0.0, 0.0, 0.0);

		double xDistPow = std::pow(sceneOtherPos.second.x - scenePos.second.x, 2);
		double yDistPow = std::pow(sceneOtherPos.second.y - scenePos.second.y, 2);
		double maxRadius = xDistPow + yDistPow;

		for (size_t i = 0; i < xs.size(); ++i)
		{
			double dist = std::pow(scenePos.second.x - xs[i], 2) + std::pow(scenePos.second.y - ys[i], 2);

			if (dist < maxRadius)
			{
				clickedPose.x = xs[i];
				clickedPose.y = ys[i];
				foundPose = true;
			}
		}

		if (foundPose)
			setSelected(clickedPose);
	}
}

std::pair<bool, math::TPoint3D> CGlWidget::sceneToWorld(const QPoint &pos) const
{
	mrpt::math::TLine3D outRay;
	mainViewport()->get3DRayForPixelCoord(pos.x(), pos.y(), outRay);

	const mrpt::math::TPlane groundPlane(mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(1, 0, 0), mrpt::math::TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(outRay, groundPlane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	mrpt::math::TPoint3D intersPt;
	bool converted = inters.getPoint(intersPt);
	return std::make_pair(converted, intersPt);
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
