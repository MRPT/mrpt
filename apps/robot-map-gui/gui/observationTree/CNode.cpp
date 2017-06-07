#include "CNode.h"


CNode::CNode(const std::string& name)
	: name_(name)
{

}

const CNode*CNode::child(int id) const
{
	return const_cast<CNode *>(this)->child(id);
}

CNode::TypeObject CNode::type() const
{
	return TypeObject::Node;
}

const std::string& CNode::displayName() const
{
	return name_;
}


