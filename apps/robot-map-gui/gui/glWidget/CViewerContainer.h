/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QWidget>

#include <memory>

#include "CDocument.h"
#include "ui_CViewerContainer.h"


class CNode;

class CViewerContainer: public QWidget
{
	Q_OBJECT
public:
	CViewerContainer(QWidget *parent = nullptr);
	virtual ~CViewerContainer();
	void showRangeScan(CNode *node);
	void showRobotDirection(CNode *node);
	void applyConfigChanges(RenderizableMaps renderizableMaps);
	void updateConfigChanges(RenderizableMaps renderizableMaps, CDocument *doc, bool isShowAllObs);
	void setDocument(CDocument *doc);

public slots:
	void showAllObservation(bool is);

private slots:
	void updateZoomInfo(int index);
	void changeZoomInfo(float zoom);
	void zoomChanged(double d);
	void zoomChanged(int d);
	void updateMouseInfo(double x, double y);

private:
	std::unique_ptr<Ui::CViewerContainer> m_ui;
	std::map<int, SType> m_tabsInfo;

};
