#pragma once
#include <QAbstractTableModel>

#include "CRootNode.h"

#include <memory>


namespace mrpt{namespace maps{class CSimpleMap;}}

class CObservationTreeModel : public QAbstractTableModel
{
public:
	CObservationTreeModel(const mrpt::maps::CSimpleMap &simplemap, QObject *parent = nullptr);
	virtual ~CObservationTreeModel();

	// QAbstractItemModel interface
	virtual int rowCount(const QModelIndex &parent) const override;
	virtual int columnCount(const QModelIndex &index) const override;

	virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
	virtual Qt::ItemFlags flags (const QModelIndex &index) const override;

	virtual QModelIndex index(int row, int column, const QModelIndex &parent) const override;
	virtual QModelIndex parent(const QModelIndex &index) const override;

	CNode* getNodeFromIndexSafe(const QModelIndex& index) const;
private:
	int findMyRowId(const CNode* node) const;

	const mrpt::maps::CSimpleMap& m_simplemap;
	std::unique_ptr<CRootNode> rootNode_;

};
