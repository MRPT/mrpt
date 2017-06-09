#include "CPairNode.h"
#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"
#include "mrpt/obs/CObservation2DRangeScan.h"


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

CPose3D getPoseFromObservation(CObservation::Ptr obs)
{
	CPose3D::Ptr pose = CPose3D::Create();
	obs->getSensorPose(*pose);
	return *(pose.get());
}

CPairNode::CPairNode(CNode *parent, const CSimpleMap::TPosePDFSensFramePair &poseSensFramePair)
	: CPosesNode(parent, poseSensFramePair.first->getMeanVal(), "Pose ")
{
	CSensoryFrame::Ptr frame = poseSensFramePair.second;
	for (auto iter = frame->begin(); iter != frame->end(); ++iter)
	{
		m_observation.push_back(std::make_unique<CPosesNode>(this, getPoseFromObservation(*iter), "Frame "));
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

