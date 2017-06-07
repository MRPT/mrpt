#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"


using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::poses;


std::string getNameFromPosePair(const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
{
	CPose3D pose = poseSensFramePair.first->getMeanVal();
	std::string str;
	pose.asString(str);
	return str;
}

CPosesNode::CPosesNode(CNode *parent, const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
	: CNode(getNameFromPosePair(poseSensFramePair))
	, parent_(parent)
{

}

int CPosesNode::childCount() const
{
	return 0;
}

const CNode *CPosesNode::parentItem() const
{
	return nullptr;
}

CNode *CPosesNode::child(int id)
{
	return nullptr;
}

void CPosesNode::addNewChild()
{
	return;
}


