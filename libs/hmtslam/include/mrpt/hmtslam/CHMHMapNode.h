/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/safe_pointers.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/hmtslam/HMT_SLAM_common.h>

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/hmtslam/CMHPropertiesValuesList.h>
#include <mrpt/graphs/TNodeID.h>
#include <map>

namespace mrpt::hmtslam
{
class CHierarchicalMHMap;
class CHMHMapArc;

/** A class for representing a node in a hierarchical, multi-hypothesis map.
 *   The node itself will be considered only if some given hypothesisID matchs
 * its own ID.
 * \note Create objects by invoking the class factory "::Create"
 *
 * \sa CHierarchicalMHMap,CHMHMapArc
 * \ingroup mrpt_hmtslam_grp
 */
class CHMHMapNode : public mrpt::serialization::CSerializable
{
	friend class CHierarchicalMHMap;
	friend class CHierarchicalMHMapPartition;
	friend class CHMHMapArc;

	DEFINE_SERIALIZABLE(CHMHMapNode)

   public:
	/** The type of the IDs of nodes.
	 */
	using TNodeID = mrpt::graphs::TNodeID;

	/** The hypothesis IDs under which this node exists.
	 */
	THypothesisIDSet m_hypotheses;

   protected:
	/** An unique identifier for the node: it is randomly generated at
	 * construction or read from stream when loaded.
	 */
	TNodeID m_ID{};

	/** The list of all arcs from/to this node:
	 */
	TArcList m_arcs;

	/** Event handler for arc destruction: It should be only called for arcs
	 * from/to this node, altought other case must be handled without effects.
	 * \note At *addition we use a smart pointer to assure all the implied guys
	 * use the same smrt. pnt., but at destructors the objects don't know
	 * anything but "this", thus the usage of plain pointers.
	 */
	void onArcDestruction(CHMHMapArc* arc);

	/** Event handler for arc addition: It should be only called for arcs
	 * from/to this node, although other cases have no effects.
	 */
	void onArcAddition(const std::shared_ptr<CHMHMapArc>& arc);

	/** The hierarchical graph in which this object is into.
	 */
	mrpt::safe_ptr<CHierarchicalMHMap> m_parent;

   public:
	/** Constructor
	 */
	CHMHMapNode(
		CHierarchicalMHMap* parent = nullptr,
		const THypothesisIDSet& hyps = THypothesisIDSet());

	/** Destructor
	 */
	~CHMHMapNode() override;

	/** The annotations of the node, see the general description of the class
	 * for possible properties and values.
	 */
	CMHPropertiesValuesList m_annotations;

	/** The type of the node, the possibilities are:
	 *		- Place
	 *		- Area
	 *		- TopologicalMap
	 *		- Object
	 */
	std::string m_nodeType;

	/** Reads the ID of the node (read-only property)
	 */
	TNodeID getID() const;

	/** The label of the node, only for display it to the user.
	 */
	std::string m_label;

	/** Returns the level of this node in the hierarchy of arcs
	 * "arcType_Belongs", where level=0 is the ground level, 1=its parents, etc.
	 */
	unsigned int getLevelInTheHierarchy();

	/** Returns the number of arcs starting from/ending into this node.
	 */
	unsigned int getRelatedArcsCount();

	/** Returns a list with the arcs from/to this node.
	 */
	void getArcs(TArcList& out) const { out = m_arcs; }
	/** Returns a list with the arcs from/to this node existing in a given
	 * hypothesis ID.
	 */
	void getArcs(TArcList& out, const THypothesisID& hyp_id) const;

	/** Returns a list with the arcs from/to this node existing in a given
	 * hypothesis ID and of a given type.
	 */
	void getArcs(
		TArcList& out, const char* arcType, const THypothesisID& hyp_id) const;

	/** Check whether an arc exists towards the given area */
	bool isNeighbor(
		const TNodeID& otherArea, const THypothesisID& hyp_id) const;

};  // End of class def.

/** A map between node IDs and nodes (used in HMT-SLAM).
 * \sa CHMTSLAM
 */
using TNodeList = std::map<CHMHMapNode::TNodeID, std::shared_ptr<CHMHMapNode>>;
using TNodeIDList = mrpt::containers::list_searchable<CHMHMapNode::TNodeID>;
using TNodeIDSet = std::set<CHMHMapNode::TNodeID>;
using TPairNodeIDs = std::pair<CHMHMapNode::TNodeID, CHMHMapNode::TNodeID>;

}  // namespace mrpt::hmtslam
