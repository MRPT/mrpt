/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/random.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CHMHMapNode, CSerializable, mrpt::hmtslam)

/*---------------------------------------------------------------
						Default constructor
  ---------------------------------------------------------------*/
CHMHMapNode::CHMHMapNode(
	CHierarchicalMHMap* parent, const THypothesisIDSet& hyps)
	: m_hypotheses(hyps),
	  m_arcs(),
	  m_parent(parent),
	  m_nodeType(NODE_TYPES, DEFAULT_NODE_TYPE),
	  m_label("none")
{
	// Assure that ID is unique in the graph:
	// -----------------------------------------
	if (m_parent.get())
	{
		// Parent will be nullptr only in the default constructor for a
		// temporary
		//  initialization before loading the object from "readFromStream"
		m_ID = 0;
		do
		{
			m_ID++; /* =
				(((uint64_t)getRandomGenerator().drawUniform(0.0f,0xFFFF))
				<< 32) |
				(((uint64_t)getRandomGenerator().drawUniform(0.0f,0xFFFF)) <<
				16) |
				(((uint64_t)getRandomGenerator().drawUniform(0.0f,0xFFFF)));*/
		} while (m_parent->getNodeByID(m_ID));
	}
}

/*---------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
CHMHMapNode::~CHMHMapNode()
{
	// To the graph:
	if (m_parent.get()) m_parent->onNodeDestruction(this);

	// To the arcs:
	for (auto& m_arc : m_arcs) m_arc->onNodeDestruction(this);
}

uint8_t CHMHMapNode::serializeGetVersion() const { return 0; }
void CHMHMapNode::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << m_ID << m_label;
	out << m_nodeType;
	out << m_annotations;
	out << m_hypotheses;
}

void CHMHMapNode::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> m_ID >> m_label >> m_nodeType >> m_annotations >>
				m_hypotheses;

			// It's not necessary since at ::Create this is already done
			// (but...check!)
			// if (m_parent.get())
			//	m_parent->onNodeAddition(this);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CHMHMapNode::onArcDestruction(CHMHMapArc* arc)
{
	MRPT_START

	// Important note: We cannot create a temporary smart pointer here, since
	//  it will lead to an infinity recursion!  (BUGFIX, JLBC SEP-2009)

	// Check if arc is from/to this node:
	if (arc->m_nodeFrom == m_ID || arc->m_nodeTo == m_ID)
	{
		// Remove from the list:
		auto it = m_arcs.find_ptr_to(arc);
		if (it != m_arcs.end()) m_arcs.erase(it);
	}

	MRPT_END
}

/*---------------------------------------------------------------
					onArcAddition
  ---------------------------------------------------------------*/
void CHMHMapNode::onArcAddition(const CHMHMapArc::Ptr& arc)
{
	MRPT_START

	// Check if arc is from/to this node:
	if (arc->m_nodeFrom == m_ID || arc->m_nodeTo == m_ID)
	{
		// Already in the list?
		auto it = m_arcs.find(arc);
		if (it == m_arcs.end()) m_arcs.push_back(arc);  // Add to the list:
	}

	MRPT_END
}
/*---------------------------------------------------------------
					onArcAddition
  ---------------------------------------------------------------*/
CHMHMapNode::TNodeID CHMHMapNode::getID() const { return m_ID; }
/*---------------------------------------------------------------
					getLevelInTheHierarchy
  ---------------------------------------------------------------*/
unsigned int CHMHMapNode::getLevelInTheHierarchy()
{
	TArcList::iterator itArc;
	unsigned int level = 0;

	for (itArc = m_arcs.begin(); itArc != m_arcs.end(); itArc++)
	{
		// I am a "level+1" from the level below if a "belongs" arc points to
		// me:
		if ((*itArc)->m_arcType == "Membership" &&
			(*itArc)->m_nodeTo == this->m_ID)
		{
			unsigned int L = m_parent->getNodeByID((*itArc)->m_nodeFrom)
								 ->getLevelInTheHierarchy();
			level = max(L + 1, level);
		}
	}

	return level;
}

/*---------------------------------------------------------------
					getRelatedArcsCount
  ---------------------------------------------------------------*/
unsigned int CHMHMapNode::getRelatedArcsCount()
{
	return (unsigned int)m_arcs.size();
}

/*---------------------------------------------------------------
					getArcs
  ---------------------------------------------------------------*/
void CHMHMapNode::getArcs(TArcList& out, const THypothesisID& hyp_id) const
{
	out.clear();
	for (const auto& m_arc : m_arcs)
		if (m_arc->m_hypotheses.has(hyp_id)) out.push_back(m_arc);
}

/*---------------------------------------------------------------
					getArcs
  ---------------------------------------------------------------*/
void CHMHMapNode::getArcs(
	TArcList& out, const char* arcType, const THypothesisID& hyp_id) const
{
	out.clear();
	for (const auto& a : m_arcs)
		if (a->m_hypotheses.has(hyp_id) && a->m_arcType == arcType)
			out.push_back(a);
}

/*---------------------------------------------------------------
					isNeighbor
  ---------------------------------------------------------------*/
bool CHMHMapNode::isNeighbor(
	const TNodeID& otherArea, const THypothesisID& hyp_id) const
{
	for (const auto& m_arc : m_arcs)
		if (m_arc->m_hypotheses.has(hyp_id) &&
			(m_arc->m_nodeFrom == otherArea || m_arc->m_nodeTo == otherArea))
			return true;
	return false;  // Nope
}
