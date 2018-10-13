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
#include <QAbstractTableModel>
#include <QItemSelection>
#include <QDebug>

#include "CRootNode.h"

#include <memory>

namespace mrpt
{
namespace maps
{
class CSimpleMap;
}
}  // namespace mrpt

/** This class is a model for a tree view*/

class CObservationTreeModel : public QAbstractTableModel
{
   public:
	CObservationTreeModel(
		const mrpt::maps::CSimpleMap& simplemap, QObject* parent = nullptr);
	~CObservationTreeModel() override;

	QItemSelection changeSelected(const std::vector<size_t>& indexes);

	// QAbstractItemModel interface
	int rowCount(const QModelIndex& parent) const override;
	int columnCount(const QModelIndex& index) const override;

	QVariant data(
		const QModelIndex& index, int role = Qt::DisplayRole) const override;
	Qt::ItemFlags flags(const QModelIndex& index) const override;

	QModelIndex index(
		int row, int column, const QModelIndex& parent) const override;
	QModelIndex parent(const QModelIndex& index) const override;

	CNode* getNode(const QModelIndex& index) const;
	CNode* getNodeFromIndexSafe(const QModelIndex& index) const;
	const std::vector<mrpt::poses::CPose3D>& poses() const;

   private:
	int findMyRowId(const CNode* node) const;

	const mrpt::maps::CSimpleMap& m_simplemap;
	std::unique_ptr<CRootNode> m_rootNode;
	std::vector<mrpt::poses::CPose3D> m_poses;
};
