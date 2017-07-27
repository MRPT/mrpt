/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
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

CGlWidget::CGlWidget(bool is2D, QWidget* parent)
	: CQtGlCanvasBase(parent),
	  m_groundPlane(
		  mrpt::make_aligned_shared<CGridPlaneXY>(-200, 200, -200, 200, 0, 5)),
	  m_doc(nullptr),
	  m_miniMapSize(-1.0),
	  m_minimapPercentSize(0.25),
	  m_observationSize(10.),
	  m_observationColor(mrpt::utils::TColor::red),
	  m_selectedObsSize(15.0),
	  m_selectedColor(mrpt::utils::TColor::green),
	  m_isShowObs(false),
	  m_visiblePoints(mrpt::make_aligned_shared<CPointCloud>()),
	  m_selectedPointsCloud(mrpt::make_aligned_shared<CPointCloud>()),
	  m_currentObs(opengl::stock_objects::CornerXYZSimple()),
	  m_line(mrpt::make_aligned_shared<CSetOfLines>()),
	  m_is2D(is2D),
	  m_showRobot(false)
{
	setClearColors(1.0, 1.0, 1.0);

	m_visiblePoints->setColor(m_observationColor);
	m_visiblePoints->setPointSize(m_observationSize);

	m_selectedPointsCloud->setColor(m_selectedColor);
	m_selectedPointsCloud->setPointSize(m_selectedObsSize);

	if (m_is2D)
	{
		m_miniMapViewport = getOpenGLSceneRef()->createViewport("miniMap");
		m_miniMapViewport->setBorderSize(2);
		m_miniMapViewport->setViewportPosition(
			0.01, 0.01, m_minimapPercentSize, m_minimapPercentSize);
		m_miniMapViewport->setTransparent(false);

		setAzimuthDegrees(-90.0f);
		setElevationDegrees(90.0f);

		CCamera& camMiniMap = m_miniMapViewport->getCamera();
		updateCameraParams(camMiniMap);
		camMiniMap.setOrthogonal();
	}

	updateCamerasParams();
	m_groundPlane->setColor(0.4, 0.4, 0.4);
	setVisibleGrid(true);
}

CGlWidget::~CGlWidget() {}
void CGlWidget::fillMap(const CSetOfObjects::Ptr& renderizableMap)
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
			CRenderizable* ren = m_map->begin()->get();
			CTexturedPlane* textured = dynamic_cast<CTexturedPlane*>(ren);
			if (textured)
			{
				textured->getPlaneCorners(xMin, xMax, yMin, yMax);
			}
			else
			{
				CPointCloud* points = dynamic_cast<CPointCloud*>(ren);
				if (points)
				{
					const std::vector<float>& arrayX = points->getArrayX();
					const std::vector<float>& arrayY = points->getArrayY();

					for (auto& itX : arrayX)
					{
						xMin = std::min(xMin, itX);
						xMax = std::max(xMax, itX);
					}
					for (auto& itY : arrayY)
					{
						yMin = std::min(yMin, itY);
						yMax = std::max(yMax, itY);
					}
				}
			}
		}

		float xDist = xMax - xMin;
		float yDist = yMax - yMin;

		CCamera& camMiniMap = m_miniMapViewport->getCamera();
		updateCameraParams(camMiniMap);
		camMiniMap.setZoomDistance(std::max(xDist, yDist));
		updateCamerasParams();
	}

	if (m_isShowObs)
		setSelectedObservation(m_isShowObs);
	else
		update();
}

void CGlWidget::setSelected(const math::TPose3D& pose)
{
	if (!m_map) return;

	removeRobotDirection();

	m_showRobot = true;

	m_currentObs->setPose(pose);
	m_map->insert(m_currentObs);

	update();
}

void CGlWidget::setSelectedObservation(bool is)
{
	m_isShowObs = is;
	if (!m_doc || !m_map) return;

	if (is)
	{
		m_map->insert(m_visiblePoints);
		m_map->insert(m_selectedPointsCloud);
	}

	else
	{
		m_map->removeObject(m_visiblePoints);
		m_map->removeObject(m_selectedPointsCloud);
		m_selectedPointsCloud->clear();
		removeRobotDirection();
	}

	update();
}

void CGlWidget::setLaserScan(CPlanarLaserScan::Ptr laserScan)
{
	if (m_currentLaserScan) m_map->removeObject(m_currentLaserScan);

	m_currentLaserScan = laserScan;
	m_map->insert(m_currentLaserScan);

	update();
}

void CGlWidget::setDocument(CDocument* doc)
{
	m_doc = doc;

	if (m_isShowObs)
	{
		m_map->removeObject(m_visiblePoints);
		m_map->removeObject(m_selectedPointsCloud);
	}
	m_selectedPointsCloud->clear();
	m_visiblePoints->clear();
	for (auto iter = m_doc->simplemap().begin();
		 iter != m_doc->simplemap().end(); ++iter)
	{
		math::TPose3D pose = iter->first->getMeanVal();
		m_visiblePoints->insertPoint(pose.x, pose.y, pose.z);
	}

	if (m_isShowObs) setSelectedObservation(m_isShowObs);
}

void CGlWidget::setZoom(float zoom)
{
	CamaraParams params = cameraParams();
	params.cameraZoomDistance = zoom;
	setCameraParams(params);
	updateCamerasParams();

	update();
}

float CGlWidget::getZoom() const { return getZoomDistance(); }
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

void CGlWidget::setBackgroundColor(float r, float g, float b, float a)
{
	setClearColors(r, g, b, a);

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
}

bool CGlWidget::setBot(int value)
{
	if (!m_map) return false;

	math::TPose3D pose;

	if (m_showRobot)
	{
		m_map->removeObject(m_currentObs);
		pose = m_currentObs->getPose();
	}
	mrpt::opengl::CSetOfObjects::Ptr currentObs = m_currentObs;

	switch (value)
	{
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
		m_currentObs->setPose(pose);
		m_map->insert(m_currentObs);
		return currentObs != m_currentObs;
	}

	return false;
}

bool CGlWidget::setObservationSize(double s)
{
	if (m_observationSize != s)
	{
		m_observationSize = s;
		m_visiblePoints->setPointSize(m_observationSize);
		return true;
	}
	return false;
}

bool CGlWidget::setObservationColor(int type)
{
	utils::TColorf color = typeToColor(type);
	if (color.R != m_observationColor.R || color.B != m_observationColor.B ||
		color.G != m_observationColor.G || color.A != m_observationColor.A)
	{
		m_observationColor = color;
		m_visiblePoints->setColor(m_observationColor);
		return true;
	}
	return false;
}

bool CGlWidget::setSelectedObservationSize(double s)
{
	if (m_selectedObsSize != s)
	{
		m_selectedObsSize = s;
		m_selectedPointsCloud->setPointSize(m_selectedObsSize);
		return true;
	}
	return false;
}

bool CGlWidget::setSelectedObservationColor(int type)
{
	utils::TColorf color = typeToColor(type);
	if (color.R != m_selectedColor.R || color.B != m_selectedColor.B ||
		color.G != m_selectedColor.G || color.A != m_selectedColor.A)
	{
		m_selectedColor = color;
		m_selectedPointsCloud->setColor(m_selectedColor);
		return true;
	}
	return false;
}

void CGlWidget::resizeGL(int width, int height)
{
	CQtGlCanvasBase::resizeGL(width, height);

	if (m_is2D)
	{
		GLint win_dims[4];
		glGetIntegerv(GL_VIEWPORT, win_dims);
		m_miniMapSize =
			std::min(win_dims[2], win_dims[3]) * m_minimapPercentSize;
	}
	updateMinimapPos();
}

void CGlWidget::updateCamerasParams()
{
	float zoom = getCameraZoomDistance();
	CQtGlCanvasBase::updateCamerasParams();

	if (m_is2D)
	{
		CCamera& camMiniMap = m_miniMapViewport->getCamera();
		float zoomMiniMap = camMiniMap.getZoomDistance();
		updateCameraParams(camMiniMap);
		camMiniMap.setProjectiveModel(false);
		camMiniMap.setZoomDistance(zoomMiniMap);
	}

	if (zoom != getZoomDistance()) emit zoomChanged(getZoomDistance());
}

void CGlWidget::insertToMap(const CRenderizable::Ptr& newObject)
{
	CQtGlCanvasBase::insertToMap(newObject);
	if (m_is2D)
	{
		assert(m_miniMapViewport);
		m_miniMapViewport->insert(newObject);
	}
}

void CGlWidget::removeFromMap(const CRenderizable::Ptr& newObject)
{
	CQtGlCanvasBase::removeFromMap(newObject);
	if (m_is2D)
	{
		assert(m_miniMapViewport);
		m_miniMapViewport->removeObject(newObject);
	}
}

void CGlWidget::mouseMoveEvent(QMouseEvent* event)
{
	CQtGlCanvasBase::mouseMoveEvent(event);

	std::pair<bool, math::TPoint3D> scenePos = sceneToWorld(event->pos());
	if (scenePos.first)
		emit mousePosChanged(scenePos.second.x, scenePos.second.y);
}

void CGlWidget::mousePressEvent(QMouseEvent* event)
{
	CQtGlCanvasBase::mousePressEvent(event);
	if (!m_isShowObs) return;

	QPoint pos = event->pos();
	bool isPressCtrl = event->modifiers() == Qt::ControlModifier;
	QPoint otherPos(pos.x() + m_observationSize, pos.y() + m_observationSize);

	auto scenePos = sceneToWorld(pos);
	auto sceneOtherPos = sceneToWorld(otherPos);

	if (scenePos.first && sceneOtherPos.first)
	{
		double xDistPow =
			std::pow(sceneOtherPos.second.x - scenePos.second.x, 2);
		double yDistPow =
			std::pow(sceneOtherPos.second.y - scenePos.second.y, 2);
		double maxDist = xDistPow + yDistPow;

		bool updateScene = false;
		if (isPressCtrl)
		{
			int i = searchSelectedPose(
				scenePos.second.x, scenePos.second.y, maxDist);
			if (i != -1)
			{
				auto point = removeFromSelected(i);
				setVisiblePose(point.x, point.y, point.z);
				updateScene = true;
			}
		}
		else if (deselectAll())
			update();

		if (!updateScene)
		{
			int i = searchPose(scenePos.second.x, scenePos.second.y, maxDist);
			if (i != -1)
			{
				removeFromVisible(i);
				selectPose(
					scenePos.second.x, scenePos.second.y, getSafeZPose());
				updateScene = true;
			}
		}
		if (updateScene)
		{
			update();
			unpressMouseButtons();
		}
	}
}

utils::TColorf CGlWidget::typeToColor(int type) const
{
	mrpt::utils::TColor color = mrpt::utils::TColor::red;
	;
	switch (type)
	{
		case 0:
			color = mrpt::utils::TColor::red;
			break;
		case 1:
			color = mrpt::utils::TColor::green;
			break;
		case 2:
			color = mrpt::utils::TColor::blue;
			break;
		case 3:
			color = mrpt::utils::TColor::white;
			break;
		case 4:
			color = mrpt::utils::TColor::black;
			break;
		case 5:
			color = mrpt::utils::TColor::gray;
			break;
		default:
			break;
	}
	return mrpt::utils::TColorf(color);
}

std::pair<bool, math::TPoint3D> CGlWidget::sceneToWorld(const QPoint& pos) const
{
	mrpt::math::TLine3D outRay;
	mainViewport()->get3DRayForPixelCoord(pos.x(), pos.y(), outRay);

	const mrpt::math::TPlane groundPlane(
		mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(1, 0, 0),
		mrpt::math::TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(outRay, groundPlane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	mrpt::math::TPoint3D intersPt;
	bool converted = inters.getPoint(intersPt);
	return std::make_pair(converted, intersPt);
}

int CGlWidget::searchPoseFromList(
	float x, float y, double maxDist, const std::vector<float>& xs,
	const std::vector<float>& ys) const
{
	assert(xs.size() == ys.size());

	std::map<double, int> minDistPos;
	for (size_t i = 0; i < xs.size(); ++i)
	{
		double dist = std::pow(x - xs[i], 2) + std::pow(y - ys[i], 2);
		if (dist < maxDist) minDistPos.emplace(dist, i);
	}

	if (minDistPos.empty()) return -1;

	return minDistPos.begin()->second;
}

math::TPoint3D CGlWidget::removePoseFromPointsCloud(
	CPointCloud::Ptr pointsCloud, int index) const
{
	auto xs = pointsCloud->getArrayX();
	auto ys = pointsCloud->getArrayY();
	auto zs = pointsCloud->getArrayZ();
	assert(xs.size() == ys.size() && xs.size() == zs.size());

	assert(index < xs.size());
	math::TPoint3D point;

	auto itX = xs.begin() + index;
	point.x = *itX;
	xs.erase(itX);

	auto itY = ys.begin() + index;
	point.y = *itY;
	ys.erase(itY);

	auto itZ = zs.begin() + index;
	point.z = *itZ;
	zs.erase(itZ);

	pointsCloud->setAllPoints(xs, ys, zs);
	return point;
}

void CGlWidget::removeRobotDirection()
{
	m_map->removeObject(m_currentObs);

	if (m_currentLaserScan) m_map->removeObject(m_currentLaserScan);

	m_showRobot = false;
}

void CGlWidget::updateMinimapPos()
{
	if (!m_is2D) return;

	GLint win_dims[4];
	glGetIntegerv(GL_VIEWPORT, win_dims);

	COpenGLViewport::Ptr miniMap = getOpenGLSceneRef()->getViewport("miniMap");
	float w = m_miniMapSize / win_dims[2];
	float h = m_miniMapSize / win_dims[3];
	miniMap->setViewportPosition(0.01, 0.01, w, h);
}

int CGlWidget::searchSelectedPose(float x, float y, double maxDist)
{
	auto xs = m_selectedPointsCloud->getArrayX();
	auto ys = m_selectedPointsCloud->getArrayY();

	return searchPoseFromList(x, y, maxDist, xs, ys);
}

int CGlWidget::searchPose(float x, float y, double maxDist)
{
	auto xs = m_visiblePoints->getArrayX();
	auto ys = m_visiblePoints->getArrayY();

	return searchPoseFromList(x, y, maxDist, xs, ys);
}

math::TPoint3D CGlWidget::removeFromSelected(int index)
{
	return removePoseFromPointsCloud(m_selectedPointsCloud, index);
}

void CGlWidget::selectPose(float x, float y, float z)
{
	m_selectedPointsCloud->insertPoint(x, y, z);
	math::TPose3D clickedPose(x, y, z, 0.0, 0.0, 0.0);
	setSelected(clickedPose);
}

void CGlWidget::setVisiblePose(float x, float y, float z)
{
	m_visiblePoints->insertPoint(x, y, z);
}

bool CGlWidget::deselectAll()
{
	auto xs = m_selectedPointsCloud->getArrayX();
	auto ys = m_selectedPointsCloud->getArrayY();
	auto zs = m_selectedPointsCloud->getArrayZ();
	assert(xs.size() == ys.size() && xs.size() == zs.size());

	auto xv = m_visiblePoints->getArrayX();
	auto yv = m_visiblePoints->getArrayY();
	auto zv = m_visiblePoints->getArrayZ();

	assert(xv.size() == yv.size() && xv.size() == zv.size());

	for (size_t i = 0; i < xs.size(); ++i)
	{
		xv.push_back(xs[i]);
		yv.push_back(ys[i]);
		zv.push_back(zs[i]);
	}
	m_visiblePoints->setAllPoints(xv, yv, zv);

	bool changedVectors = xs.size();
	xs.clear();
	ys.clear();
	zs.clear();
	m_selectedPointsCloud->setAllPoints(xs, ys, zs);

	removeRobotDirection();

	return changedVectors;
}

float CGlWidget::getSafeZPose() const
{
	auto zs = m_visiblePoints->getArrayZ();
	float z = 0.0;
	if (!zs.empty())
		z = zs[0];
	else
	{
		zs = m_selectedPointsCloud->getArrayZ();
		if (!zs.empty()) z = zs[0];
	}
	return z;
}

math::TPoint3D CGlWidget::removeFromVisible(int index)
{
	return removePoseFromPointsCloud(m_visiblePoints, index);
}
