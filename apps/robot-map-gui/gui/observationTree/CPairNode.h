#pragma once
#include "CNode.h"

#include <string>

#include "mrpt/maps/CSimpleMap.h"


class CObservationsNode;
class CPoseNode;

class CPairNode :public CNode
{
public:
	CPairNode(CNode *parent, const mrpt::maps::CSimpleMap::TPosePDFSensFramePair &poseSensFramePair);
	virtual ~CPairNode();

	int childCount() const;
	CNode* child(int id);
	ObjectType type() const override;
	std::string displayName() const override;

private:
	std::unique_ptr<CPoseNode>  m_pose;
	std::unique_ptr<CObservationsNode> m_observations;

};


