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
#include <QWidget>

#include <memory>

#include "CDocument.h"
#include "gui/configWidget/CGeneralConfig.h"
#include "ui_CViewerContainer.h"

class CNode;
class CGlWidget;

class CViewerContainer : public QWidget
{
	Q_OBJECT
   public:
	CViewerContainer(QWidget* parent = nullptr);
	virtual ~CViewerContainer();
	void showRangeScan(CNode* node);
	void showRobotDirection(const mrpt::poses::CPose3D& pose);
	void applyConfigChanges(RenderizableMaps renderizableMaps);
	void updateConfigChanges(
		RenderizableMaps renderizableMaps, CDocument* doc, bool isShowAllObs);
	void setDocument(CDocument* doc);

	void setGeneralSetting(const SGeneralSetting& setting);
	void updateRobotPosesSize(double size);
	void updateSelectedRobotPosesSize(double size);

	void updateRobotPosesColor(int type);
	void updateSelectedRobotPosesColor(int type);

   signals:
	void deleteRobotPoses(const std::vector<int>& idx);
	void moveRobotPoses(const std::vector<int>& idx, const QPointF& dist);

   public slots:
	void showAllObservation(bool is);
	void changeCurrentBot(int value);
	void setVisibleGrid(bool is);
	void changeBackgroundColor(const QColor& color);
	void changeGridColor(const QColor& color);

   private slots:
	void updatePanelInfo(int index);
	void changeZoomInfo(float zoom);
	void changeAzimuthDeg(float deg);
	void changeElevationDeg(float deg);
	void zoomChanged(double d);
	void zoomChanged(int d);
	void updateMouseInfo(double x, double y);
	void updateAzimuthDegrees(double deg);
	void updateElevationDegrees(double deg);

   private:
	CGlWidget* getCurrentTabWidget() const;

	std::unique_ptr<Ui::CViewerContainer> m_ui;
	std::map<int, SType> m_tabsInfo;
};
