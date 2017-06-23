#include "CPairNode.h"

#include "CPosesNode.h"
#include "CObservationsNode.h"


using namespace mrpt;
using namespace mrpt::maps;


CPairNode::CPairNode(CNode *parent, const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
	: CNode(parent)
	, m_pose(std::make_unique<CPoseNode>(this, poseSensFramePair.first->getMeanVal()))
	, m_observations(std::make_unique<CObservationsNode>(this, poseSensFramePair.second))
{

}

CPairNode::~CPairNode()
{

}

int CPairNode::childCount() const
{
	return 2;
}

CNode *CPairNode::child(int id)
{
	switch (id) {
	case 0:
		return m_pose.get();
	case 1:
		return m_observations.get();
	default:
		return nullptr;
	};
}


CNode::ObjectType CPairNode::type() const
{
	return ObjectType::PosWithObservationPair;
}

std::string CPairNode::displayName() const
{
	return "Position";
}
