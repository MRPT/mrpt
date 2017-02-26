/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHMHMapArc_H
#define CHMHMapArc_H

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/hmtslam/CHMHMapNode.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace hmtslam
	{
		class HMTSLAM_IMPEXP CHierarchicalMHMap;

		/** A class for representing an arc between two nodes in a hierarchical, multi-hypothesis map.
		 *   The arc itself will be considered only if some given hypothesisID matchs its own ID.
		 * \note Create objects by invoking the class factory "::Create"
		 *
		 * \sa CHierarchicalMHMap,CHMHMapNode
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHMHMapArc : public mrpt::utils::CSerializable
		{
			friend class CHierarchicalMHMap;
			friend class CHMHMapNode;
			friend class CHierarchicalMapMHPartition;
			friend class TArcList;

			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CHMHMapArc )

		public:
			/** The hypothesis IDs under which this arc exists.
			  */
			THypothesisIDSet		m_hypotheses;

		protected:
			/** The origin/target nodes for this arc.
			  */
			CHMHMapNode::TNodeID	m_nodeFrom, m_nodeTo;

			/** The hierarchical graph in which this object is into. */
			mrpt::utils::safe_ptr<CHierarchicalMHMap>	m_parent;


			 /** Event handler to be called just before a node is being destroyed: it should be called only for nodes involved in the arc, altought other cases must be handled without effects
			  *   When a node involved in the arc is delected, the corresponding pointer in the arc will be set to NULL and the arc is no longer a valid one.
			  */
			 void  onNodeDestruction(CHMHMapNode *node);

		private:
			/** Private constructor (see ::Create class factory)
			  */
			CHMHMapArc(
				const CHMHMapNode::TNodeID		&from =0,
				const CHMHMapNode::TNodeID		&to   =0,
				const THypothesisIDSet			&hyps = THypothesisIDSet(),
				CHierarchicalMHMap		*parent=NULL);

			/** Alternative constructor, using pointers for convenience.
			  */
			CHMHMapArc(
				CHMHMapNodePtr		&from,
				CHMHMapNodePtr		&to,
				const THypothesisIDSet		&hyps,
				CHierarchicalMHMap			*parent);

		public:
			/** Constructor from node IDs
			  */
			static CHMHMapArcPtr Create(
				const CHMHMapNode::TNodeID		&from,
				const CHMHMapNode::TNodeID		&to,
				const THypothesisIDSet			&hyps = THypothesisIDSet(),
				CHierarchicalMHMap		*parent=NULL);

			/** Alternative constructor, using pointers for convenience.
			  */
			static CHMHMapArcPtr Create(
				CHMHMapNodePtr		&from,
				CHMHMapNodePtr		&to,
				const THypothesisIDSet		&hyps,
				CHierarchicalMHMap			*parent);

			/** Destructor
			  */
			virtual ~CHMHMapArc();

			/** Return the starting node of the arc:
			  */
			CHMHMapNode::TNodeID	 getNodeFrom() const { ASSERT_(m_nodeFrom!=AREAID_INVALID); return m_nodeFrom; }

			/** Return the ending node of the arc:
			  */
			CHMHMapNode::TNodeID	 getNodeTo() const { ASSERT_(m_nodeTo!=AREAID_INVALID); return m_nodeTo; }

			/** The type of the arc, the possibilities are:
					- "Membership": for abstractions
					- "Navegability"
					- "RelativePose"
			  */
			utils::CTypeSelector			m_arcType;

			/** The annotations of the arc, see the general description of the class for possible properties and values.
			  */
			utils::CMHPropertiesValuesList	m_annotations;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHMHMapArc,mrpt::utils::CSerializable, HMTSLAM_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
