/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/graphs/CDirectedTree.h>

namespace mrpt
{
	namespace nav
	{
		/** This class contains motions and motions tree structures for the hybrid navigation algorithm
		*
		*  <b>Usage:</b><br>
		*      \note: this class inheredit mrpt::graphs::CDirectedTree, please refer to inheritance for detail about generic tree methods
		*
		*      - initialize a motions tree using .initializeMoveTree()
		*      - addEdge (from, to)
		*      - add here more instructions
		*
		* <b>Changes history</b>
		*      - 06/MAR/2014: Creation (MB)
		*  \ingroup mrpt_nav_grp
		*/

		template<class MOVE_TYPE>
		class TMoveTree : public mrpt::graphs::CDirectedTree<MOVE_TYPE>
		{
		public:
			// JL: Removed previous contents, since they are already in the base CDirectedTree<>

			/** return the TEdgeInfo of the nearest neighbor to a specific motion.
			* \note: is this better to go into CDirectedTree.h ?
			*/
			mrpt::utils::TNodeID getNearestNode(const MOVE_TYPE &TMove_)   //think about what to return here
			{
				//write me !!!
				MRPT_TODO ("WRITE getNearestMove function, here of in CDirectedTree.h?")
				return 0;
			}

		}; // end TMoveTree

		/** An edge for the move tree used for planning in SE2 and TP-space */
		class TMoveEdgeSE2_TP
		{
		public:
			TMoveEdgeSE2_TP ( const mrpt::utils::TNodeID parent_id_, const mrpt::math::TPose2D end_pose_ ) :
				parent_id (parent_id_),
				end_state( end_pose_ ),
				cost( 0.0 ),
				ptg_index ( 0 ), ptg_K ( 0 ), ptg_dist ( 0.0 )   //these are all PTGs parameters
			{}
			const mrpt::utils::TNodeID parent_id;  //!< The ID of the parent node in the tree
			const mrpt::math::TPose2D  end_state;  //!< state in SE2 as 2D pose (x, y, phi)  - \note: it is not possible to initialize a motion without a state
			double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
			int ptg_index;          //!< indicate the type of trajectory used for this motion
			int ptg_K;              //!< identify the trajectory number K of the type ptg_index
			double ptg_dist;        //!< identify the lenght of the trajectory for this motion
		};


		typedef TMoveTree<TMoveEdgeSE2_TP> TMoveTreeSE2_TP;  //!< tree data structure for planning in SE2 with TP-space

	}
}
