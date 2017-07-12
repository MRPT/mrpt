/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <QTreeView>


class CObservationTreeModel;

class CObservationTree : public QTreeView
{
public:
	CObservationTree(QWidget *parent = nullptr);
	virtual ~CObservationTree() = default;
	virtual void setModel(QAbstractItemModel *model);

protected:
	virtual void contextMenuEvent(QContextMenuEvent *event);

private:
	CObservationTreeModel *m_model;
};
