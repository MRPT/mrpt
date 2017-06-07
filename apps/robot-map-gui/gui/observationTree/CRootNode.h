#pragma once
#include <string>
#include <vector>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


class CRootNode : public CNode
{
public:
	CRootNode(const mrpt::maps::CSimpleMap &simplemap);
	virtual ~CRootNode() = default;

	// INode interface
public:
	virtual int childCount() const override;
	virtual CNode* child(int id) override;
	virtual const CNode* parentItem() const override;
	virtual void addNewChild() override;

private:
	std::vector<CNode *> m_posesNode;
};
