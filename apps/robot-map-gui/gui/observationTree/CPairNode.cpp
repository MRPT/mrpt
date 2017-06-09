#include "CPairNode.h"
#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"


using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::obs;


std::string getNameFromPosePair(const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
{
	CPose3D pose = poseSensFramePair.first->getMeanVal();
	std::string str;
	pose.asString(str);
	return str;
}

CPose3D::Ptr getPose(CObservation::Ptr obs)
{
	CPose3D::Ptr pose = CPose3D::Create();
	obs->getSensorPose(*pose);
	return pose;
}

CPairNode::CPairNode(CNode *parent, const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
	: CNode(parent, "Pose " + getNameFromPosePair(poseSensFramePair))
{
	CSensoryFrame::Ptr frame = poseSensFramePair.second;
	for (auto iter = frame->begin(); iter != frame->end(); ++iter)
	{
		m_observation.push_back(std::make_unique<CPosesNode>(this, getPose(*iter)));
	}
}

int CPairNode::childCount() const
{
	return m_observation.size();
}

CNode *CPairNode::child(int id)
{
	return m_observation.at(id).get();
}

void CPairNode::addNewChild()
{
	return;
}


