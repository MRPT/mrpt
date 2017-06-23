#pragma once

#include "CBaseObservationNode.h"

#include "mrpt/obs/CObservation2DRangeScan.h"


class CRangeScanNode : public CBaseObservationNode
{
public:
	CRangeScanNode(CNode* parent, mrpt::obs::CObservation2DRangeScan::Ptr);

	// CNode interface
	ObjectType type() const override;
	std::string displayName() const override;
};

