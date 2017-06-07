#include "CObservationTreeModel.h"


CObservationTreeModel::CObservationTreeModel(const mrpt::maps::CSimpleMap &simplemap, QObject *parent)
	: QAbstractTableModel(parent)
	, m_simplemap(simplemap)
	, rootNode_(new CRootNode(simplemap))
{
}

CObservationTreeModel::~CObservationTreeModel()
{
}

int CObservationTreeModel::rowCount(const QModelIndex &parent) const
{
	CNode* parentNode;
	if (parent.column() > 0)
		return 0;

	parentNode = getNodeFromIndexSafe(parent);

	return parentNode->childCount();
}

int CObservationTreeModel::columnCount(const QModelIndex &index) const
{
	Q_UNUSED(index);
	return 1;
}

QVariant CObservationTreeModel::data(const QModelIndex &index, int role) const
{
	if (!index.isValid() || role != Qt::DisplayRole)
		return QVariant();

	CNode *node = getNodeFromIndexSafe(index);
	return node->displayName().c_str();
}

Qt::ItemFlags CObservationTreeModel::flags(const QModelIndex &index) const
{
	return QAbstractItemModel::flags(index);
}



QModelIndex CObservationTreeModel::index(int row, int column, const QModelIndex &parent) const
{
	if (!hasIndex(row, column, parent))
		return QModelIndex();

	CNode *parentNode = getNodeFromIndexSafe(parent);

	CNode *childItem = parentNode->child(row);
	if (childItem)
		return createIndex(row, column, childItem);
	else
		return QModelIndex();
}

QModelIndex CObservationTreeModel::parent(const QModelIndex &index) const
{
	CNode* node = getNodeFromIndexSafe(index);
	if (node == rootNode_)
		return QModelIndex();

	const CNode* parent = node->parentItem();

	return createIndex(findMyRowId(parent), 0, const_cast<CNode *>(parent));

}

CNode *CObservationTreeModel::getNodeFromIndexSafe(const QModelIndex &index) const
{
	CNode* node;
	if (!index.isValid() || !index.internalPointer())
		node = rootNode_;
	else
		node = static_cast<CNode*>(index.internalPointer());


	return node;
}

int CObservationTreeModel::findMyRowId(const CNode *node) const
{
	int rowInParentScope = 0;
	if (node)
	{
		const auto* parent = node->parentItem();
		if (parent)
		{
			for (int i = 0; i < parent->childCount(); ++i)
			{
				if (parent->child(i) == node)
					rowInParentScope = i;
			}
		}
	}

	return rowInParentScope;
}
