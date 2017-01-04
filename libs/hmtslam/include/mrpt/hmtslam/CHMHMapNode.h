/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHMHMapNode_H
#define CHMHMapNode_H

#include <mrpt/utils/safe_pointers.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/hmtslam/HMT_SLAM_common.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CMHPropertiesValuesList.h>
#include <mrpt/utils/CTypeSelector.h>
#include <map>

namespace mrpt
{
	namespace hmtslam
	{
		class HMTSLAM_IMPEXP CHierarchicalMHMap;
		class HMTSLAM_IMPEXP CHMHMapArc;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHMHMapNode,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** A class for representing a node in a hierarchical, multi-hypothesis map.
		 *   The node itself will be considered only if some given hypothesisID matchs its own ID.
		 * \note Create objects by invoking the class factory "::Create"
		 *
		 * \sa CHierarchicalMHMap,CHMHMapArc
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHMHMapNode : public mrpt::utils::CSerializable
		{
			friend class HMTSLAM_IMPEXP CHierarchicalMHMap;
			friend class HMTSLAM_IMPEXP CHierarchicalMHMapPartition;
			friend class HMTSLAM_IMPEXP CHMHMapArc;

			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CHMHMapNode )

		public:
			/** The type of the IDs of nodes.
			  */
			typedef	mrpt::utils::TNodeID  TNodeID;

			/** The hypothesis IDs under which this node exists.
			  */
			THypothesisIDSet		m_hypotheses;

		protected:
			/** An unique identifier for the node: it is randomly generated at construction or read from stream when loaded.
			  */
			TNodeID					m_ID;

			/** The list of all arcs from/to this node:
			  */
			TArcList				m_arcs;

			/** Event handler for arc destruction: It should be only called for arcs from/to this node, altought other case must be handled without effects.
			  * \note At *addition we use a smart pointer to assure all the implied guys use the same smrt. pnt., but at destructors the objects don't know anything but "this", thus the usage of plain pointers.
			  */
			void  onArcDestruction(CHMHMapArc *arc);

			/** Event handler for arc addition: It should be only called for arcs from/to this node, altought other cases have no effects.
			  */
			void  onArcAddition(CHMHMapArcPtr &arc);

			/** The hierarchical graph in which this object is into.
			  */
			mrpt::utils::safe_ptr<CHierarchicalMHMap>	m_parent;

		private:
			/** Private constructor (see ::Create class factory)
			  */
			CHMHMapNode(
				CHierarchicalMHMap		*parent = NULL,
				const THypothesisIDSet	&hyps = THypothesisIDSet() );

		public:
			/** Class factory
			  */
			static CHMHMapNodePtr Create(
				CHierarchicalMHMap		*parent = NULL,
				const THypothesisIDSet	&hyps = THypothesisIDSet() );

			/** Destructor
			 */
			virtual ~CHMHMapNode();

			/** The annotations of the node, see the general description of the class for possible properties and values.
			  */
			utils::CMHPropertiesValuesList	m_annotations;

			/** The type of the node, the possibilities are:
			  *		- Place
			  *		- Area
			  *		- TopologicalMap
			  *		- Object
			  */
			utils::CTypeSelector			m_nodeType;

			/** Reads the ID of the node (read-only property)
			  */
			TNodeID getID() const;

			/** The label of the node, only for display it to the user.
			  */
			std::string		m_label;

			/** Returns the level of this node in the hierarchy of arcs "arcType_Belongs", where level=0 is the ground level, 1=its parents, etc.
			  */
			unsigned int getLevelInTheHierarchy();

			/** Returns the number of arcs starting from/ending into this node.
			  */
			unsigned int getRelatedArcsCount();

			/** Returns a list with the arcs from/to this node.
			  */
			void getArcs( TArcList &out ) const
			{
				out = m_arcs;
			}

			/** Returns a list with the arcs from/to this node existing in a given hypothesis ID.
			  */
			void getArcs( TArcList &out, const THypothesisID &hyp_id ) const;

			/** Returns a list with the arcs from/to this node existing in a given hypothesis ID and of a given type.
			  */
			void getArcs( TArcList &out, const char *arcType, const THypothesisID &hyp_id ) const;

			/** Check whether an arc exists towards the given area */
			bool isNeighbor(const TNodeID &otherArea, const THypothesisID &hyp_id ) const;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHMHMapNode,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


		/** A map between node IDs and nodes (used in HMT-SLAM).
		  * \sa CHMTSLAM
		  */
		typedef std::map<CHMHMapNode::TNodeID,CHMHMapNodePtr>  TNodeList;
		typedef mrpt::utils::list_searchable<CHMHMapNode::TNodeID> TNodeIDList;
		typedef std::set<CHMHMapNode::TNodeID> TNodeIDSet;
		typedef std::pair<CHMHMapNode::TNodeID,CHMHMapNode::TNodeID>  TPairNodeIDs;

	} // End of namespace
} // End of namespace

#endif
