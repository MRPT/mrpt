/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CObservationTree.h"
#include "CObservationTreeModel.h"
#include "CNode.h"

#include <QContextMenuEvent>
#include <QMenu>
#include <QDebug>

CObservationTree::CObservationTree(QWidget* parent) : QTreeView(parent) {}
void CObservationTree::setModel(QAbstractItemModel* model)
{
	m_model = dynamic_cast<CObservationTreeModel*>(model);
	QTreeView::setModel(model);
}

void CObservationTree::changeSelected(const std::vector<size_t>& idx)
{
	blockSignals(true);
	clearSelection();
	QItemSelection selection = m_model->changeSelected(idx);
	selectionModel()->select(selection, QItemSelectionModel::Select);
	blockSignals(false);
}

void CObservationTree::expandAll()
{
	if (!m_model) return;

	QModelIndexList indexes = m_model->match(
		m_model->index(0, 0, QModelIndex()), Qt::DisplayRole, "*", -1,
		Qt::MatchWildcard | Qt::MatchRecursive);
	foreach (QModelIndex index, indexes)
		expand(index);
}

void CObservationTree::collapseAll()
{
	if (!m_model) return;

	QModelIndexList indexes = m_model->match(
		m_model->index(0, 0, QModelIndex()), Qt::DisplayRole, "*", -1,
		Qt::MatchWildcard | Qt::MatchRecursive);
	foreach (QModelIndex index, indexes)
		collapse(index);
}

void CObservationTree::contextMenuEvent(QContextMenuEvent* event)
{
	if (m_model)
	{
		QModelIndex index = indexAt(event->pos());
		CNode* node = m_model->getNode(index);
		if (node)
		{
			QMenu menu(this);
			menu.exec(event->globalPos());
		}
	}
	QWidget::contextMenuEvent(event);
}
