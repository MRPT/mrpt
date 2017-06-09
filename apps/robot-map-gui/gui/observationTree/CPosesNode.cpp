#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"


using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::poses;


std::string getNameFromPose(const CPose3D::Ptr pose)
{
	std::string str;
	pose->asString(str);
	return str;
}


CPosesNode::CPosesNode(CNode *parent, CPose3D::Ptr pose)
	: CNode(parent, "Frame" + getNameFromPose(pose))
	, m_pose(pose)
{

}

int CPosesNode::childCount() const
{
	return 0;
}


CNode *CPosesNode::child(int id)
{
	return nullptr;
}

void CPosesNode::addNewChild()
{
	return;
}


