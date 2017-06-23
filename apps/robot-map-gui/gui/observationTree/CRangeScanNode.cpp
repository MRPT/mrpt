#include "CRangeScanNode.h"


using namespace mrpt;
using namespace mrpt::obs;

CRangeScanNode::CRangeScanNode(CNode *parent, CObservation2DRangeScan::Ptr)
	: CBaseObservationNode(parent)
{

}

CNode::ObjectType CRangeScanNode::type() const
{
	ObjectType::RangeScan;
}

std::string CRangeScanNode::displayName() const
{
	return "Range scan";
}
