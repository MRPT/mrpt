/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CObservationTreeModel.h"

CObservationTreeModel::CObservationTreeModel(
	const mrpt::maps::CSimpleMap& simplemap, QObject* parent)
	: QAbstractTableModel(parent),
	  m_simplemap(simplemap),
	  m_rootNode(std::make_unique<CRootNode>(simplemap))
{
	if (!simplemap.empty())
		for (auto iter = simplemap.begin(); iter != simplemap.end(); ++iter)
		{
			m_poses.push_back(iter->first->getMeanVal());
		}
}

CObservationTreeModel::~CObservationTreeModel() = default;
QItemSelection CObservationTreeModel::changeSelected(
	const std::vector<size_t>& indexes)
{
	QItemSelection selection;
	for (auto& idx : indexes)
	{
		QModelIndex i = index(idx, 0, QModelIndex());
		if (i != QModelIndex()) selection.select(i, i);
	}
	return selection;
}

int CObservationTreeModel::rowCount(const QModelIndex& parent) const
{
	CNode* parentNode;
	if (parent.column() > 0) return 0;

	parentNode = getNodeFromIndexSafe(parent);

	return parentNode->childCount();
}

int CObservationTreeModel::columnCount(const QModelIndex& index) const
{
	Q_UNUSED(index);
	return 1;
}

QVariant CObservationTreeModel::data(const QModelIndex& index, int role) const
{
	if (!index.isValid() || role != Qt::DisplayRole) return QVariant();

	CNode* node = getNodeFromIndexSafe(index);
	return node->displayName().c_str();
}

Qt::ItemFlags CObservationTreeModel::flags(const QModelIndex& index) const
{
	return QAbstractItemModel::flags(index);
}

QModelIndex CObservationTreeModel::index(
	int row, int column, const QModelIndex& parent) const
{
	if (!hasIndex(row, column, parent)) return QModelIndex();

	CNode* parentNode = getNodeFromIndexSafe(parent);

	CNode* childItem = (parentNode->child(row));
	if (childItem)
		return createIndex(row, column, childItem);
	else
		return QModelIndex();
}

QModelIndex CObservationTreeModel::parent(const QModelIndex& index) const
{
	CNode* node = getNodeFromIndexSafe(index);
	if (node == m_rootNode.get()) return QModelIndex();

	const CNode* parent = node->parentItem();

	return createIndex(findMyRowId(parent), 0, const_cast<CNode*>(parent));
}

CNode* CObservationTreeModel::getNode(const QModelIndex& index) const
{
	CNode* node = nullptr;
	if (index.isValid() && index.internalPointer())
		node = static_cast<CNode*>(index.internalPointer());

	return node;
}

CNode* CObservationTreeModel::getNodeFromIndexSafe(
	const QModelIndex& index) const
{
	CNode* node;
	if (!index.isValid() || !index.internalPointer())
		node = m_rootNode.get();
	else
		node = static_cast<CNode*>(index.internalPointer());

	return node;
}

const std::vector<mrpt::poses::CPose3D>& CObservationTreeModel::poses() const
{
	return m_poses;
}

int CObservationTreeModel::findMyRowId(const CNode* node) const
{
	int rowInParentScope = 0;
	if (node)
	{
		const auto* parent = node->parentItem();
		if (parent)
		{
			for (int i = 0; i < parent->childCount(); ++i)
			{
				if (parent->child(i) == node) rowInParentScope = i;
			}
		}
	}

	return rowInParentScope;
}
