#pragma once
#include <string>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


namespace mrpt{namespace poses{class CPose3D;}}

class CPosesNode :public CNode
{
public:
	CPosesNode(CNode *parent, mrpt::poses::CPose3D::Ptr pose);
	virtual ~CPosesNode() = default;

	virtual int childCount() const;
	virtual CNode* child(int id);
	virtual void addNewChild();


private:
	mrpt::poses::CPose3D::Ptr m_pose;
};


