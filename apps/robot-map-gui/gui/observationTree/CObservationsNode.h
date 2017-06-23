#pragma once

#include "CNode.h"

#include "mrpt/obs/CSensoryFrame.h"

#include <vector>


class CBaseObservationNode;

class CObservationsNode : public CNode
{
public:
	CObservationsNode(CNode* parent, const mrpt::obs::CSensoryFrame::Ptr &sensoryFrame);
	~CObservationsNode();
	// CNode interface
	int childCount() const override;
	CNode *child(int id) override;
	ObjectType type() const override;
	std::string displayName() const override;

private:
	std::vector<std::unique_ptr<CBaseObservationNode>> m_observations;
};
