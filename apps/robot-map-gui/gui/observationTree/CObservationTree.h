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
#include <QTreeView>

/** This class implements tree view for storage observations matching robot
 * poses
 */

class CObservationTreeModel;

class CObservationTree : public QTreeView
{
	Q_OBJECT
   public:
	CObservationTree(QWidget* parent = nullptr);
	~CObservationTree() override = default;
	void setModel(QAbstractItemModel* model) override;
	void changeSelected(const std::vector<size_t>& idx);

   public slots:
	void expandAll();
	void collapseAll();

   protected:
	void contextMenuEvent(QContextMenuEvent* event) override;

   private:
	CObservationTreeModel* m_model{nullptr};
};
