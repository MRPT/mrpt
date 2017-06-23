#include "CObservationsNode.h"

#include "CRangeScanNode.h"
#include "CBaseObservationNode.h"

#include "mrpt/obs/CObservation2DRangeScan.h"


using namespace mrpt;
using namespace mrpt::obs;

CObservationsNode::CObservationsNode(CNode *parent, const CSensoryFrame::Ptr &sensoryFrame)
	: CNode(parent)
{
	auto scanPtr = sensoryFrame->getObservationByClass<CObservation2DRangeScan>();
	if (scanPtr)
	{
		m_observations.emplace_back(std::make_unique<CRangeScanNode>(this, scanPtr));
	}
}

CObservationsNode::~CObservationsNode()
{
}

int CObservationsNode::childCount() const
{
	return m_observations.size();
}

CNode *CObservationsNode::child(int id)
{
	return static_cast<CNode *>(m_observations[id].get());
}

CNode::ObjectType CObservationsNode::type() const
{
	return ObjectType::Observations;
}

std::string CObservationsNode::displayName() const
{
	return "Observations";
}
