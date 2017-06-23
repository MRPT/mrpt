#pragma once

#include "CNode.h"


class CBaseObservationNode : public CNode
{
public:

	CBaseObservationNode(CNode* parent);
	virtual ~CBaseObservationNode() = default;
	// CNode interface
	int childCount() const override;
	CNode *child(int id) override;
};
