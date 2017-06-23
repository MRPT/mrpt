#include "CBaseObservationNode.h"


CBaseObservationNode::CBaseObservationNode(CNode *parent)
	: CNode(parent)
{
}

int CBaseObservationNode::childCount() const
{
	return 0;
}

CNode *CBaseObservationNode::child(int id)
{
	return nullptr;
}
