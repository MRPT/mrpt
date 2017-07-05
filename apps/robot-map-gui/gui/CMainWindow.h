/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QMainWindow>

#include <memory>

#include <mrpt/opengl/CSetOfObjects.h>


namespace Ui
{
class CMainWindow;
}
class CGlWidget;
class CDocument;
class CObservationTreeModel;
class QTreeWidgetItem;

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	CMainWindow(QWidget *parent = 0);
	virtual ~CMainWindow();

private slots:
	void openMap();
	void itemClicked(const QModelIndex &index);
	void updateConfig();
	void updateConfig(const std::string str);

	void applyConfigurationForCurrentMaps();
private:
	void updateRenderMapFromConfig();
	void createNewDocument();

	CDocument *m_document;
	CObservationTreeModel *m_model;

	std::unique_ptr<Ui::CMainWindow> m_ui;
};
