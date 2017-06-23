#include "CNode.h"


CNode::CNode(CNode *parent)
	: m_parent(parent)
{

}

const CNode *CNode::parentItem() const
{
	return m_parent;
}

void CNode::addNewChild()
{
	throw std::runtime_error("addNewChild not implemented!");
}

const CNode*CNode::child(int id) const
{
	return const_cast<CNode *>(this)->child(id);
}
