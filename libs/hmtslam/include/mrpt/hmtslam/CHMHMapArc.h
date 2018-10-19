/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/hmtslam/CHMHMapNode.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::hmtslam
{
class CHierarchicalMHMap;

/** A class for representing an arc between two nodes in a hierarchical,
 * multi-hypothesis map.
 *   The arc itself will be considered only if some given hypothesisID matchs
 * its own ID.
 * \note Create objects by invoking the class factory "::Create"
 *
 * \sa CHierarchicalMHMap,CHMHMapNode
 * \ingroup mrpt_hmtslam_grp
 */
class CHMHMapArc : public mrpt::serialization::CSerializable
{
	friend class CHierarchicalMHMap;
	friend class CHMHMapNode;
	friend class CHierarchicalMapMHPartition;
	friend class TArcList;

	DEFINE_SERIALIZABLE(CHMHMapArc)

   public:
	/** The hypothesis IDs under which this arc exists.
	 */
	THypothesisIDSet m_hypotheses;

   protected:
	/** The origin/target nodes for this arc.
	 */
	CHMHMapNode::TNodeID m_nodeFrom, m_nodeTo;

	/** The hierarchical graph in which this object is into. */
	mrpt::safe_ptr<CHierarchicalMHMap> m_parent;

	/** Event handler to be called just before a node is being destroyed: it
	 * should be called only for nodes involved in the arc, altought other cases
	 * must be handled without effects
	 *   When a node involved in the arc is delected, the corresponding pointer
	 * in the arc will be set to nullptr and the arc is no longer a valid one.
	 */
	void onNodeDestruction(CHMHMapNode* node);

   public:
	/** Private constructor (see ::Create class factory)
	 */
	CHMHMapArc(
		const CHMHMapNode::TNodeID& from = 0,
		const CHMHMapNode::TNodeID& to = 0,
		const THypothesisIDSet& hyps = THypothesisIDSet(),
		CHierarchicalMHMap* parent = nullptr);

	/** Alternative constructor, using pointers for convenience.
	 */
	CHMHMapArc(
		CHMHMapNode::Ptr& from, CHMHMapNode::Ptr& to,
		const THypothesisIDSet& hyps, CHierarchicalMHMap* parent);

	/** Destructor
	 */
	~CHMHMapArc() override;

	/** Return the starting node of the arc:
	 */
	CHMHMapNode::TNodeID getNodeFrom() const
	{
		ASSERT_(m_nodeFrom != AREAID_INVALID);
		return m_nodeFrom;
	}

	/** Return the ending node of the arc:
	 */
	CHMHMapNode::TNodeID getNodeTo() const
	{
		ASSERT_(m_nodeTo != AREAID_INVALID);
		return m_nodeTo;
	}

	/** The type of the arc, the possibilities are:
			- "Membership": for abstractions
			- "Navegability"
			- "RelativePose"
	  */
	std::string m_arcType;

	/** The annotations of the arc, see the general description of the class for
	 * possible properties and values.
	 */
	CMHPropertiesValuesList m_annotations;

};  // End of class def.

}  // namespace mrpt::hmtslam
