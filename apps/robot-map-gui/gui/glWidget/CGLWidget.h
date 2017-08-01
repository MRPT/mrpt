/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <QGLWidget>

#include "mrpt/gui/CQtGlCanvasBase.h"
#include "mrpt/opengl/CSetOfObjects.h"
#include "mrpt/opengl/CSetOfLines.h"
#include "mrpt/opengl/CPointCloud.h"
#include "mrpt/maps/CSimpleMap.h"
#include "mrpt/maps/CMultiMetricMap.h"
#include "mrpt/opengl/CPlanarLaserScan.h"
#include "mrpt/opengl/CGridPlaneXY.h"

class CDocument;

class CGlWidget : public mrpt::gui::CQtGlCanvasBase
{
	Q_OBJECT
   public:
	CGlWidget(bool is2D, QWidget* parent = nullptr);
	virtual ~CGlWidget();

	void fillMap(const mrpt::opengl::CSetOfObjects::Ptr& renderizableMap);
	void setDocument(CDocument* doc);

	void setSelected(const mrpt::math::TPose3D& pose);
	void setSelectedObservation(bool is);
	void setLaserScan(mrpt::opengl::CPlanarLaserScan::Ptr laserScan);

	void setZoom(float zoom);
	float getZoom() const;

	virtual void setCameraParams(const CamaraParams& params) override;

	virtual void setAzimuthDegrees(float ang) override;
	virtual void setElevationDegrees(float ang) override;

	void setBackgroundColor(float r, float g, float b, float a);
	void setGridColor(double r, double g, double b, double a);

	void setVisibleGrid(bool is);

	bool setBot(int value);

	bool setObservationSize(double s);
	bool setObservationColor(int type);

	bool setSelectedObservationSize(double s);
	bool setSelectedObservationColor(int type);

   signals:
	void zoomChanged(float zoom);
	void mousePosChanged(double x, double y);
	void azimuthChanged(float ang);
	void elevationChanged(float ang);

   protected:
	virtual void resizeGL(int width, int height) override;
	virtual void updateCamerasParams() override;
	virtual void insertToMap(
		const mrpt::opengl::CRenderizable::Ptr& newObject) override;
	virtual void removeFromMap(
		const mrpt::opengl::CRenderizable::Ptr& newObject) override;
	virtual void mouseMoveEvent(QMouseEvent* event) override;
	virtual void mousePressEvent(QMouseEvent* event) override;

   private:
	mrpt::utils::TColorf typeToColor(int type) const;
	std::pair<bool, mrpt::math::TPoint3D> sceneToWorld(const QPoint& pos) const;
	int searchPoseFromList(
		float x, float y, double maxDist, const std::vector<float>& xs,
		const std::vector<float>& ys) const;
	mrpt::math::TPoint3D removePoseFromPointsCloud(
		mrpt::opengl::CPointCloud::Ptr pointsCloud, int index) const;

	void removeRobotDirection();
	void updateMinimapPos();
	int searchSelectedPose(float x, float y, double maxDist);
	int searchPose(float x, float y, double maxDist);
	mrpt::math::TPoint3D removeFromSelected(int index);
	void selectPose(float x, float y, float z);
	void setVisiblePose(float x, float y, float z);
	bool deselectAll();
	mrpt::math::TPoint3D removeFromVisible(int index);

	mrpt::opengl::COpenGLViewport::Ptr m_miniMapViewport;
	mrpt::opengl::CSetOfObjects::Ptr m_map;
	mrpt::opengl::CGridPlaneXY::Ptr m_groundPlane;

	CDocument* m_doc;
	float m_miniMapSize;
	const float m_minimapPercentSize;
	double m_observationSize;
	mrpt::utils::TColorf m_observationColor;
	double m_selectedObsSize;
	mrpt::utils::TColorf m_selectedColor;

	bool m_isShowObs;
	mrpt::opengl::CPointCloud::Ptr m_visiblePoints;
	mrpt::opengl::CSetOfObjects::Ptr m_selectedPointsCloud;
	mrpt::opengl::CPlanarLaserScan::Ptr m_currentLaserScan;
	mrpt::opengl::CSetOfObjects::Ptr m_currentObs;
	mrpt::opengl::CSetOfLines::Ptr m_line;

	bool m_is2D;
	bool m_showRobot;
};
