#pragma once
#include <string>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


class CPosesNode :public CNode
{
public:
	CPosesNode(CNode *parent, const mrpt::maps::CSimpleMap::TPosePDFSensFramePair &poseSensFramePair);
	virtual ~CPosesNode() = default;

	virtual int childCount() const;
	virtual const CNode* parentItem() const;
	virtual CNode* child(int id);
	virtual void addNewChild();


private:
	CNode *parent_;

};


