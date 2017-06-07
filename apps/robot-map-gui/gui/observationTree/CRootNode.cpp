#include "CRootNode.h"
#include "CPosesNode.h"

#include <cassert>


using namespace mrpt;
using namespace mrpt::maps;

CRootNode::CRootNode(const CSimpleMap &simplemap)
	: CNode ("")
{
	for (auto iter = simplemap.begin(); iter != simplemap.end(); ++iter)
	{
		CPosesNode *node = new CPosesNode(this, *iter);
		m_posesNode.push_back(node);

	}
}

int CRootNode::childCount() const
{
	return 1;
}

CNode* CRootNode::child(int id)
{
	assert(id <= m_posesNode.size());
	return m_posesNode[id];
}

const CNode* CRootNode::parentItem() const
{
	return nullptr;
}

void CRootNode::addNewChild()
{
	return;
}


