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
#include <QSettings>

#include <memory>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/CSimpleMap.h>

/** This class implements GUI of main window and connection with other classes
*/

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
		std::vector<size_t> idx,
		mrpt::maps::CSimpleMap::TPosePDFSensFramePairList posesObsPairs);
	void deleteRobotPosesFromMap(const std::vector<size_t>& idx);
	void moveRobotPosesOnMap(
		const std::vector<size_t>& idx, const QPointF& dist);
	void loadMap(const QString& fileName);

   private slots:
	void about();
	void undo();
	void redo();
	void openMap();
	void saveMap();
	void saveAsText();
	void saveAsPNG();
	void itemClicked(const QModelIndex& index);
	void updateConfig();
	void openConfig(const std::string& str);

	void applyConfigurationForCurrentMaps();
	void showMapConfiguration();

	void deleteRobotPoses(const std::vector<size_t>& idx);
	void moveRobotPoses(const std::vector<size_t>& idx, const QPointF& dist);

	void openRecent();
	void saveMetricMapRepresentation();
	void saveMetricmapInBinaryFormat();

   private:
	void updateRenderMapFromConfig();
	void applyMapsChanges();
	void createNewDocument();
	void clearObservationsViewer();

	void addToRecent(const std::string& fileName);
	void addToRecent(const QString& fileName);
	void addRecentFilesToMenu();

	void showErrorMessage(const QString& str) const;

	CDocument* m_document;
	CObservationTreeModel* m_model;

	std::unique_ptr<Ui::CMainWindow> m_ui;

	QSettings m_settings;
	QStringList m_recentFiles;
};
