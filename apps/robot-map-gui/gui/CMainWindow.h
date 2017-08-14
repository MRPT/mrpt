/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <QMainWindow>

#include <memory>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/CSimpleMap.h>

namespace Ui
{
class CMainWindow;
}
class CDocument;
class CObservationTreeModel;
class QTreeWidgetItem;

class CMainWindow : public QMainWindow
{
	Q_OBJECT

   public:
	CMainWindow(QWidget* parent = 0);
	virtual ~CMainWindow();

	void addRobotPosesFromMap(
		std::vector<int> idx,
		mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs);
	void deleteRobotPosesFromMap(const std::vector<int>& idx);
	void moveRobotPosesOnMap(const std::vector<int>& idx, const QPointF& dist);

   private slots:
	void undo();
	void redo();
	void openMap();
	void saveMap();
	void saveAsText();
	void itemClicked(const QModelIndex& index);
	void updateConfig();
	void openConfig(const std::string& str);

	void applyConfigurationForCurrentMaps();
	void showMapConfiguration();

	void deleteRobotPoses(const std::vector<int>& idx);
	void moveRobotPoses(const std::vector<int>& idx, const QPointF& dist);

   private:
	void updateRenderMapFromConfig();
	void applyMapsChanges();
	void createNewDocument();
	void clearObservationsViewer();

	CDocument* m_document;
	CObservationTreeModel* m_model;

	std::unique_ptr<Ui::CMainWindow> m_ui;
};
