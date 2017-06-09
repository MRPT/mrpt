#pragma once
#include <string>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"


namespace mrpt{namespace poses{class CPose3D;}}

class CPosesNode :public CNode
{
public:
	CPosesNode(CNode *parent, const mrpt::poses::CPose3D &pose, const std::string &name);
	virtual ~CPosesNode() = default;

	virtual int childCount() const;
	virtual CNode* child(int id);
	virtual void addNewChild();

	mrpt::poses::CPose3D getPose() const;

private:
	mrpt::poses::CPose3D m_pose;
};


