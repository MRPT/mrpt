#pragma once
#include <string>
#include <vector>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


class CRootNode : public CNode
{
public:
	CRootNode(const mrpt::maps::CSimpleMap &simplemap);
	~CRootNode() = default;

	// INode interface
	int childCount() const override;
	CNode* child(int id) override;
	ObjectType type() const override;
	std::string displayName() const override;

private:
	std::vector<CNode *> m_posesNode;

};
