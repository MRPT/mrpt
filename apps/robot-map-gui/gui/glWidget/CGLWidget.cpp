/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CGLWidget.h"

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CGlCanvasBase.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/stock_objects.h>
#include <QApplication>
#include <QMouseEvent>
#include <cmath>

#include "CDocument.h"

// Include libraries in linking (needed for Windows)
#include <mrpt/config.h>
#if defined(_MSC_VER)
#pragma comment(lib, "opengl32.lib")
#endif

using namespace mrpt;
using namespace mrpt::opengl;

CGlWidget::CGlWidget(bool is2D, QWidget* parent)
	: CQtGlCanvasBase(parent),
	  m_groundPlane(std::make_shared<CGridPlaneXY>(-200, 200, -200, 200, 0, 5)),
	  m_doc(nullptr),
	  m_miniMapSize(-1.0),
	  m_minimapPercentSize(0.25),
	  m_observationSize(10.),
	  m_observationColor(mrpt::img::TColor::red()),
	  m_selectedObsSize(15.0),
	  m_selectedColor(mrpt::img::TColor::green()),
	  m_isShowObs(false),
	  m_visiblePoints(std::make_shared<CSetOfObjects>()),
	  m_currentObs(opengl::stock_objects::CornerXYZSimple()),
	  m_line(std::make_shared<CSetOfLines>()),
	  m_is2D(is2D),
	  m_showRobot(false),
	  m_moveSelected(false),
	  m_pressedPos(0, 0),
	  m_lastPos(0, 0)
{
	setClearColors(1.0, 1.0, 1.0);
	setMinimumZoom(1.0f);
	setMaximumZoom(3200.0f);

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
	m_groundPlane->setColor(0.4f, 0.4f, 0.4f);
	setVisibleGrid(true);
	setFocusPolicy(Qt::StrongFocus);
}

CGlWidget::~CGlWidget() = default;
void CGlWidget::fillMap(const CSetOfObjects::Ptr& renderizableMap)
{
	removeFromMap(m_map);
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
			auto* textured = dynamic_cast<CTexturedPlane*>(ren);
			if (textured)
			{
				textured->getPlaneCorners(xMin, xMax, yMin, yMax);
			}
			else
			{
				auto* points = dynamic_cast<CPointCloud*>(ren);
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
		m_map->insert(m_visiblePoints);

	else
	{
		m_map->removeObject(m_visiblePoints);
		deselectAll();
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

	if (m_isShowObs) m_map->removeObject(m_visiblePoints);

	deselectAll();
	m_visiblePoints->clear();

	size_t id = 0;
	for (auto iter = m_doc->simplemap().begin();
		 iter != m_doc->simplemap().end(); ++iter)
	{
		math::TPose3D pose = iter->first->getMeanVal().asTPose();
		CRobotPose::Ptr robotPose = std::make_shared<CRobotPose>(id);
		robotPose->setPose(pose);
		m_visiblePoints->insert(robotPose);
		++id;
	}

	if (m_isShowObs) setSelectedObservation(m_isShowObs);
}

void CGlWidget::updateObservations()
{
	bool isShowObs = m_isShowObs;

	std::vector<CRobotPose::Ptr> selectedPoints = m_selectedPoints;

	if (isShowObs) setSelectedObservation(false);
	m_visiblePoints->clear();

	int id = 0;
	for (auto iter = m_doc->simplemap().begin();
		 iter != m_doc->simplemap().end(); ++iter)
	{
		math::TPose3D pose = iter->first->getMeanVal().asTPose();
		CRobotPose::Ptr robotPose = std::make_shared<CRobotPose>(id);
		robotPose->setPose(pose);
		m_visiblePoints->insert(robotPose);
		++id;
	}

	double maxDist = maximumSizeObservation(QPoint(0, 0));
	ASSERT_(maxDist != -1.0);

	for (auto& it : selectedPoints)
	{
		size_t id = it->getId();
		selectPoint(id);
	}

	emit selectedChanged(m_selectedPoints);

	if (isShowObs) setSelectedObservation(isShowObs);
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
void CGlWidget::setCameraParams(const gui::CGlCanvasBase::CamaraParams& params)
{
	CGlCanvasBase::CamaraParams m_cameraParams = cameraParams();
	if (m_cameraParams.cameraAzimuthDeg != params.cameraAzimuthDeg)
		emit azimuthChanged(params.cameraAzimuthDeg);

	if (m_cameraParams.cameraElevationDeg != params.cameraElevationDeg)
		emit elevationChanged(params.cameraElevationDeg);

	CQtGlCanvasBase::setCameraParams(params);
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

	m_map->removeObject(m_currentObs);
	if (m_showRobot) pose = m_currentObs->getPose();

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
	}

	return true;
}

bool CGlWidget::setObservationSize(double s)
{
	if (m_observationSize != s)
	{
		m_observationSize = s;
		return true;
	}
	return false;
}

bool CGlWidget::setObservationColor(int type)
{
	img::TColorf color = typeToColor(type);
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
		return true;
	}
	return false;
}

bool CGlWidget::setSelectedObservationColor(int type)
{
	img::TColorf color = typeToColor(type);
	if (color.R != m_selectedColor.R || color.B != m_selectedColor.B ||
		color.G != m_selectedColor.G || color.A != m_selectedColor.A)
	{
		m_selectedColor = color;
		return true;
	}
	return false;
}

void CGlWidget::updateSelectionWithoutSignals(const std::vector<size_t>& idx)
{
	blockSignals(true);
	deselectAll();

	for (auto& id : idx) selectPoint(id);

	blockSignals(false);
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
		ASSERT_(m_miniMapViewport);
		m_miniMapViewport->insert(newObject);
	}
}

void CGlWidget::removeFromMap(const CRenderizable::Ptr& newObject)
{
	CQtGlCanvasBase::removeFromMap(newObject);
	if (m_is2D)
	{
		ASSERT_(m_miniMapViewport);
		m_miniMapViewport->removeObject(newObject);
	}
}

void CGlWidget::mouseMoveEvent(QMouseEvent* event)
{
	CQtGlCanvasBase::mouseMoveEvent(event);

	std::pair<bool, math::TPoint3D> scenePos = sceneToWorld(event->pos());
	if (scenePos.first)
		emit mousePosChanged(scenePos.second.x, scenePos.second.y);

	if (m_moveSelected)
	{
		auto scenePos = sceneToWorld(m_lastPos);
		auto sceneOtherPos = sceneToWorld(event->pos());
		if (scenePos.first && sceneOtherPos.first)
		{
			double distX = scenePos.second.x - sceneOtherPos.second.x;
			double distY = scenePos.second.y - sceneOtherPos.second.y;
			if (distX != 0.0 || distY != 0.0)
			{
				for (auto& it : m_selectedPoints)
				{
					CRobotPose::Ptr robotPose =
						std::dynamic_pointer_cast<CRobotPose>(it);
					mrpt::math::TPose3D pose = robotPose->getPose();
					pose.x -= distX;
					pose.y -= distY;
					robotPose->setPose(pose);
				}
				update();
			}
		}
	}
	m_lastPos = event->pos();
}

void CGlWidget::mousePressEvent(QMouseEvent* event)
{
	CQtGlCanvasBase::mousePressEvent(event);

	if (!m_isShowObs) return;

	m_moveSelected = false;

	QPoint pos = event->pos();
	m_pressedPos = pos;
	m_lastPos = pos;
	bool isPressCtrl = event->modifiers() == Qt::ControlModifier;
	bool noModifier = event->modifiers() == Qt::NoModifier;
	QPoint otherPos(pos.x() + m_observationSize, pos.y() + m_observationSize);

	auto scenePos = sceneToWorld(pos);
	auto sceneOtherPos = sceneToWorld(otherPos);
	if (scenePos.first && sceneOtherPos.first)
	{
		bool needUnpressMouse = false;
		bool needUpdateScene = false;

		double maxDist = maximumSizeObservation(pos);
		int selectedIndex =
			searchSelectedPose(scenePos.second.x, scenePos.second.y, maxDist);

		if (selectedIndex != -1)
		{
			m_moveSelected = noModifier;

			if (isPressCtrl)
			{
				removePoseFromSelected(selectedIndex);
				needUpdateScene = true;
			}
			needUnpressMouse = noModifier || isPressCtrl;
		}
		else if (!isPressCtrl)
			needUpdateScene = deselectAll();

		if (!needUnpressMouse)
		{
			needUnpressMouse =
				selectPoint(scenePos.second.x, scenePos.second.y, maxDist);
			if (needUnpressMouse)
			{
				needUpdateScene = needUnpressMouse;
				m_moveSelected = needUnpressMouse;
			}
		}

		if (needUnpressMouse) unpressMouseButtons();

		if (needUpdateScene) update();
	}
}

void CGlWidget::mouseReleaseEvent(QMouseEvent* event)
{
	CQtGlCanvasBase::mouseReleaseEvent(event);

	if (m_moveSelected)
	{
		std::vector<size_t> idx;
		for (auto& it : m_selectedPoints) idx.push_back(it->getId());

		auto scenePos = sceneToWorld(m_pressedPos);
		auto sceneOtherPos = sceneToWorld(event->pos());
		if (scenePos.first && sceneOtherPos.first)
		{
			QPointF dist;
			dist.setX(sceneOtherPos.second.x - scenePos.second.x);
			dist.setY(sceneOtherPos.second.y - scenePos.second.y);
			if (dist.x() != 0.0 || dist.y() != 0.0)
				emit moveRobotPoses(idx, dist);
		}
		m_moveSelected = false;
	}
	m_pressedPos = event->pos();
}

void CGlWidget::keyPressEvent(QKeyEvent* event)
{
	CQtGlCanvasBase::keyPressEvent(event);
	if (event->key() == Qt::Key_Delete)
	{
		std::vector<size_t> idx;
		for (auto& it : m_selectedPoints) idx.push_back(it->getId());
		deselectAll();
		emit deleteRobotPoses(idx);
	}
}

img::TColorf CGlWidget::typeToColor(int type) const
{
	mrpt::img::TColor color = mrpt::img::TColor::red();
	switch (type)
	{
		case 0:
			color = mrpt::img::TColor::red();
			break;
		case 1:
			color = mrpt::img::TColor::green();
			break;
		case 2:
			color = mrpt::img::TColor::blue();
			break;
		case 3:
			color = mrpt::img::TColor::white();
			break;
		case 4:
			color = mrpt::img::TColor::black();
			break;
		case 5:
			color = mrpt::img::TColor::gray();
			break;
		default:
			break;
	}
	return mrpt::img::TColorf(color);
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

double CGlWidget::maximumSizeObservation(const QPoint& pos) const
{
	QPoint otherPos(pos.x() + m_observationSize, pos.y() + m_observationSize);

	auto scenePos = sceneToWorld(pos);
	auto sceneOtherPos = sceneToWorld(otherPos);
	if (!scenePos.first || !sceneOtherPos.first) return -1.0;

	double xDistPow = std::pow(sceneOtherPos.second.x - scenePos.second.x, 2);
	double yDistPow = std::pow(sceneOtherPos.second.y - scenePos.second.y, 2);
	double maxDist = xDistPow + yDistPow;

	return maxDist;
}

bool CGlWidget::selectPoint(float x, float y, double maxDist)
{
	int poseIndex = searchPose(x, y, maxDist);
	if (poseIndex != -1)
	{
		selectPoint((size_t)poseIndex);
		return true;
	}
	return false;
}

void CGlWidget::selectPoint(size_t poseIndex)
{
	auto robotPose = getRobotPose(poseIndex);
	robotPose->setSelected(true);
	m_selectedPoints.push_back(robotPose);
	removeRobotDirection();
	emit selectedChanged(m_selectedPoints);

	if (m_selectedPoints.size() == 1)
	{
		mrpt::math::TPose3D pose = robotPose->getPose();
		emit showPoseDirection(
			robotPose->getId(), pose.yaw, pose.pitch, pose.roll);
	}
}

template <class Container>
int CGlWidget::searchPoseFromList(
	float x, float y, double maxDist, Container list) const
{
	std::map<double, int> minDistPos;
	int i = 0;

	for (auto& it : list)
	{
		mrpt::math::TPose3D pos = it->getPose();
		double dist = std::pow(x - pos.x, 2) + std::pow(y - pos.y, 2);
		if (dist < maxDist) minDistPos.emplace(dist, i);
		++i;
	}

	if (minDistPos.empty()) return -1;

	return minDistPos.begin()->second;
}

CRobotPose::Ptr CGlWidget::removePoseFromPointsCloud(
	CSetOfObjects::Ptr points, int index) const
{
	auto it = points->begin() + index;
	ASSERT_(it != points->end());
	CRobotPose::Ptr robotPose = std::dynamic_pointer_cast<CRobotPose>(*it);
	ASSERT_(robotPose);
	points->removeObject(*it);
	return robotPose;
}

void CGlWidget::removeRobotDirection()
{
	if (!m_map.get()) return;
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
	return searchPoseFromList(x, y, maxDist, m_selectedPoints);
}

int CGlWidget::searchPose(float x, float y, double maxDist)
{
	return searchPoseFromList(x, y, maxDist, *m_visiblePoints);
}

void CGlWidget::removePoseFromSelected(int index)
{
	auto it = m_selectedPoints.begin() + index;
	ASSERT_(it != m_selectedPoints.end());
	CRobotPose::Ptr robotPose = std::dynamic_pointer_cast<CRobotPose>(*it);
	ASSERT_(robotPose);
	robotPose->setSelected(false);
	m_selectedPoints.erase(it);
	emit selectedChanged(m_selectedPoints);
}

void CGlWidget::selectPose(CRobotPose::Ptr robotPose)
{
	robotPose->setSelected(true);
	m_selectedPoints.push_back(robotPose);
	emit selectedChanged(m_selectedPoints);
	removeRobotDirection();
}

bool CGlWidget::deselectAll()
{
	bool changedVectors = false;

	for (auto& it : m_selectedPoints)
	{
		CRobotPose::Ptr robotPose = std::dynamic_pointer_cast<CRobotPose>(it);
		ASSERT_(robotPose);
		robotPose->setSelected(false);
		changedVectors = true;
	}

	m_selectedPoints.clear();
	emit selectedChanged(m_selectedPoints);
	removeRobotDirection();
	return changedVectors;
}

CRobotPose::Ptr CGlWidget::getRobotPose(size_t index)
{
	auto it = m_visiblePoints->begin() + index;
	ASSERT_(it != m_visiblePoints->end());
	CRobotPose::Ptr robotPose = std::dynamic_pointer_cast<CRobotPose>(*it);
	ASSERT_(robotPose);
	return robotPose;
}
