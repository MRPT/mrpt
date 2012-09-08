/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
