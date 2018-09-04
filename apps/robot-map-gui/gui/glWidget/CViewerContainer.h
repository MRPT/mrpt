/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <QWidget>

#include <memory>

#include "CDocument.h"
#include "CRobotPose.h"
#include "gui/configWidget/CGeneralConfig.h"
#include "ui_CViewerContainer.h"

class CNode;
class CGlWidget;
class QTextEdit;

/** This class implements work with all maps, which open*/
class CViewerContainer : public QWidget
{
	Q_OBJECT
   public:
	CViewerContainer(QWidget* parent = nullptr);
	~CViewerContainer() override;

	void changeHelpTextToAboutConfig();

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
	void selectedChanged(const std::vector<size_t>& idx);
	void deleteRobotPoses(const std::vector<size_t>& idx);
	void moveRobotPoses(const std::vector<size_t>& idx, const QPointF& dist);
	void showPoseDirection(size_t idx, double yaw, double pitch, double roll);

   public slots:
	void showAllObservation(bool is);

   private slots:
	void changedSelected(const std::vector<CRobotPose::Ptr>& robotPoses);
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
	bool containsHelpInfo() const;
	void forEachGl(const std::function<void(CGlWidget*)>& func);

	std::unique_ptr<Ui::CViewerContainer> m_ui;
	std::map<int, SType> m_tabsInfo;
	QTextEdit* m_information;
	const QString m_helpLoadMap;
	const QString m_helpLoadConfig;
};
