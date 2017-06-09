#include "CNode.h"


CNode::CNode(CNode *parent, const std::string& name)
	: m_parent(parent)
	, name_(name)
{

}

const CNode *CNode::parentItem() const
{
	return m_parent;
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


