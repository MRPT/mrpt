/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CHMHMapArc_H
#define CHMHMapArc_H

#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/hmtslam/CHMHMapNode.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace hmtslam
	{
		using namespace mrpt::slam;
		class HMTSLAM_IMPEXP CHierarchicalMHMap;

		//DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CHMHMapArc, mrpt::utils::CSerializable, HMTSLAM_IMPEXP )

		/** A class for representing an arc between two nodes in a hierarchical, multi-hypothesis map.
		 *   The arc itself will be considered only if some given hypothesisID matchs its own ID.
		 * \note Create objects by invoking the class factory "::Create"
		 *
		 * \sa CHierarchicalMHMap,CHMHMapNode
		  * \ingroup mrpt_hmtslam_grp
		 */
		class HMTSLAM_IMPEXP CHMHMapArc : public mrpt::utils::CSerializable
		{
			friend class HMTSLAM_IMPEXP CHierarchicalMHMap;
			friend class HMTSLAM_IMPEXP CHMHMapNode;
			friend class HMTSLAM_IMPEXP CHierarchicalMapMHPartition;
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

			/** The hierarchical graph in which this object is into.
			  */
			safe_ptr<CHierarchicalMHMap>	m_parent;


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


	} // End of namespace
} // End of namespace

#endif
