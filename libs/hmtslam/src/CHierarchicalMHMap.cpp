/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/serialization/CArchive.h>
#include <cstring>

using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::hmtslam;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CHierarchicalMHMap, CSerializable, mrpt::hmtslam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHierarchicalMHMap::CHierarchicalMHMap() = default;
/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CHierarchicalMHMap::~CHierarchicalMHMap() { clear(); }
/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CHierarchicalMHMap::clear()
{
	// Remaining arcs and nodes will be deleted.
	// Using smart ptr makes this simple:
	m_nodes.clear();
	m_arcs.clear();
}

uint8_t CHierarchicalMHMap::serializeGetVersion() const { return 0; }
void CHierarchicalMHMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	// Nodes:
	out.WriteAs<uint32_t>(nodeCount());
	for (const auto& n : m_nodes) out << *n.second;

	// Arcs:
	out.WriteAs<uint32_t>(arcCount());
	for (const auto& a : m_arcs) out << *a;
}

void CHierarchicalMHMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t i, n;

			// Clear previous contents:
			clear();

			// Nodes:
			in >> n;
			for (i = 0; i < n; i++)
			{
				CHMHMapNode::Ptr node = std::make_shared<CHMHMapNode>(
					this);  // This insert the node in my internal list via the
				// callback method
				in >> *node;
			}

			// Arcs:
			in >> n;
			for (i = 0; i < n; i++)
			{
				// This insert the node in my internal list via the callback
				// method
				CHMHMapNode::Ptr p1, p2;
				CHMHMapArc::Ptr arc = std::make_shared<CHMHMapArc>(
					p1, p2, THypothesisIDSet(), this);
				in >> *arc;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
						onNodeDestruction
  ---------------------------------------------------------------*/
void CHierarchicalMHMap::onNodeDestruction(CHMHMapNode* node)
{
	TNodeList::iterator it;

	it = m_nodes.find(node->getID());

	if (it != m_nodes.end())
		if (node == it->second.get()) m_nodes.erase(it);
}

/*---------------------------------------------------------------
						onArcDestruction
  ---------------------------------------------------------------*/
void CHierarchicalMHMap::onArcDestruction(CHMHMapArc* arc)
{
	// Important note: We cannot create a temporary smart pointer here, since
	//  it will lead to an infinity recursion!  (BUGFIX, JLBC SEP-2009)
	auto it = m_arcs.find_ptr_to(arc);
	if (it != m_arcs.end()) m_arcs.erase(it);
}

/*---------------------------------------------------------------
						onNodeAddition
  ---------------------------------------------------------------*/
void CHierarchicalMHMap::onNodeAddition(CHMHMapNode::Ptr& node)
{
	// Check if it is not already in the list:
	auto it = m_nodes.find(node->m_ID);

	if (it != m_nodes.end())
	{
		// Already in the list:
		ASSERT_(node == it->second);
		return;
	}
	else
	{
		// It is a new node: add to the list:
		m_nodes[node->m_ID] = node;
	}
}

/*---------------------------------------------------------------
						onArcAddition
  ---------------------------------------------------------------*/
void CHierarchicalMHMap::onArcAddition(CHMHMapArc::Ptr& arc)
{
	// Check if it is not already in the list:
	auto it = m_arcs.find(arc);

	if (it == m_arcs.end())  // Is it new?
		m_arcs.push_back(arc);
}

void CHierarchicalMHMap::loadFromXMLfile(std::string fileName)
{
	THROW_EXCEPTION(
		"Needs to be ported to tinyxml2! Please, give it a hand if you need "
		"this method.");
}

void CHierarchicalMHMap::dumpAsXMLfile(std::string fileName) const
{
	THROW_EXCEPTION(
		"Needs to be ported to tinyxml2! Please, give it a hand if you need "
		"this method.");
}
