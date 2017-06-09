#pragma once
#include <string>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


class CPosesNode;

class CPairNode :public CNode
{
public:
	CPairNode(CNode *parent, const mrpt::maps::CSimpleMap::TPosePDFSensFramePair &poseSensFramePair);
	virtual ~CPairNode() = default;

	virtual int childCount() const;
	virtual CNode* child(int id);
	virtual void addNewChild();


private:
	std::vector<std::unique_ptr<CPosesNode> > m_observation;

};


