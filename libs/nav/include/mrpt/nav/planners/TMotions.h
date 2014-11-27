/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
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
		*  <b>About the algorithm:</b><br>
		*
		*
		* <b>Changes history</b>
		*      - 06/MAR/2014: Creation (MB)
		*  \ingroup mrpt_nav_grp
		*/

		template<class MOVE_TYPE>
		class TMoveTree : public mrpt::graphs::CDirectedTree<MOVE_TYPE>
		{
		public:

			// This is how to insert an edge from ROOT to CHILDREN:
			const static mrpt::utils::TNodeID id_root = 0;    //!< set ID of the root node always 0
			mrpt::utils::TNodeID id_child;          //!< child id will change according to the parent level
			bool reverse_;              //!< flag for forward of backward tree exploration - default = false i.e. forward
			bool treeInitialized;               //!< flag for checking if the tree is correctly initialized
			typename TMoveTree::TListEdges my_list_of_edges;   //!< structure for the list of edges

			//methods we need here are:

			/**  Initialize variables and the list of edges for the tree*/
			bool initializeMoveTree ()
			{
				my_list_of_edges = this->edges_to_children[0]; //0 is id_root but it
				/**DOESN'T COMPILE with [id_root], why? */
				// the error during test.cpp linking after compilation is:
				// undefined reference to `mrpt::nav::TMoveTree<mrpt::nav::TMoveSE2>::id_root'

				id_child = 1;               //!< just initialize the id_child as the first after the root
				reverse_ = false;

				treeInitialized = true;
				//if something wrong treeInitialized = false;  //TODO
				return treeInitialized;
			}

			/**  this allow the user to set the reverse mode in the tree - false as default*/
			inline void setReverse (bool _reverse) { reverse_=_reverse; }

			//----> VERIFY THIS !!!!!!!!
			//I want to hide the tree structure to the user that only should add the node by a addNode function, so i defined:
			/** addEdge (from, to) have to be implemented in the following way:
			*  IN-> from MOVE_TYPE parent_ to MOVE_TYPE new_motion
			*
			*  inside this method a search method have to be called to find the TNodeID of parent_ (tree_depth_level)
			*  then new_motion will be added at the next level of parent_
			*
			*  \note please call initializeMoveTree first
			*/
			void addEdge ( const MOVE_TYPE &parent_, const MOVE_TYPE &new_motion )
			{
				ASSERTMSG_(treeInitialized == true, "The tree is not initialized!")

					id_child = searchIDinTree(parent_);
				typename TMoveTree::TEdgeInfo my_edge (id_child, reverse_, new_motion );
				my_list_of_edges.push_back(my_edge);
				// something is still missing with id_child I can know what level in the tree
				// but maybe more edges at the same level exist and they may came from different directions
				// I still have doubts about how adjiacencies are implemented
			}

			/** return the Node_ID of a specific motion.
			* \note: Node_ID corresponds to the depth of the tree for a specific edge
			*/
			TNodeID searchIDinTree (const MOVE_TYPE & mov)  //TMove_ is parent
			{
				id_child = 1;   // This have to be always >=1
				// it will calculated by a search function
				//write me!!
				MRPT_TODO ("WRITE searchIDinTree function, here of in CDirectedTree.h?")
					return id_child;
			}

			/** return the TEdgeInfo of the nearest neighbor to a specific motion.
			* \note: is this better to go into CDirectedTree.h ?
			*/
			TNodeID getNearestMove(const MOVE_TYPE &TMove_)   //think about what to return here
			{
				//write me !!!
				MRPT_TODO ("WRITE getNearestMove function, here of in CDirectedTree.h?")
					return 0;
			}

		};

		/** @name TMoveSE2 class for planning in SE2 */
		class TMoveSE2
		{
		public:
			TMoveSE2 ( mrpt::poses::TPose2D POSE_) :
				state( POSE_ ),    //!< should the state be initialized as NULL or something else?
				cost( 0.0 )
			{}
			mrpt::poses::TPose2D state;  //!< state in SE2 as 2D pose (x, y, phi) - \note: it is not possible to initialize a motion without a state
			double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

		/** @name TMoveSE3 class for planning in SE3 */
		class TMoveSE3
		{
		public:
			TMoveSE3( mrpt::poses::TPose3D POSE_ ) :
				state( POSE_ ),
				cost( 0.0 )
			{}
			mrpt::poses::TPose3D state;  //!< state in SE3 as 3D pose (x, y, z, yaw, pitch, roll) - \note: it is not possible to initialize a motion without a state
			double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

		/** @name TMoveSE2_TP class for planning in SE2 and TP-space */
		class TMoveSE2_TP
		{
		public:
			TMoveSE2_TP ( mrpt::poses::TPose2D POSE_ ) :
				state( POSE_ ),
				cost( 0.0 ),
				ptg_index ( 0 ), ptg_K ( 0 ), ptg_dist ( 0.0 )   //these are all PTGs parameters, do we need more?
			{}
			mrpt::poses::TPose2D state;  //!< state in SE2 as 2D pose (x, y, phi)  - \note: it is not possible to initialize a motion without a state
			double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
			int ptg_index;          //!< indicate the type of trajectory used for this motion
			int ptg_K;              //!< identify the trajectory number K of the type ptg_index
			double ptg_dist;        //!< identify the lenght of the trajectory for this motion
		};


		typedef TMoveTree<TMoveSE2> TMoveTreeSE2;        //!< tree datastructure for planning RRT in SE2
		typedef TMoveTree<TMoveSE3> TMoveTreeSE3;        //!< tree datastructure for planning RRT in SE3
		typedef TMoveTree<TMoveSE2_TP> TMoveTreeSE2_TP;  //!< tree datastructure for planning RRT in SE2 with TP-space

	}
}
