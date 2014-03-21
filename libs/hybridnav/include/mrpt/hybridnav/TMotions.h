/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef TMotions_H
#define TMotions_H

#include <mrpt/utils.h>
#include <mrpt/graphs.h>
#include <mrpt/hybridnav.h>

namespace mrpt
{
  namespace hybridnav
  {

        /** This class contains motions and motions tree structures for the hybrid navigation algorithm
         *
         *  <b>Usage:</b><br>
         *		- write me
         *
         *
         *  <b>About the algorithm:</b><br>
         *
         *
         * <b>Changes history</b>
         *      - 06/MAR/2014: Creation (MB)
         *  \ingroup mrpt_hybridnav_grp
         */




		template<class TMotions>
        class TMotionsTree : public mrpt::graphs::CDirectedTree< TMotions >
            {
            public:

                //methods we need here are:

                //---->verify this !!!!!!!!
                //I want to hide the tree structure to the user that only should add the node by a addNode function, so i defined:
                void addNode( TMotions TMotions_) {  this->TListEdges.push( TMotions_ );   }
                // in the CDirectedTree the edges are std::list < > this use is correct when we add a new node
                // but maybe would be not efficient when we need to get a new element for the tree since the list
                // are defined as LIFO/FIFO structures, how you plan to address the nearest neighbor search?
                void getNode( int node_index) { this->TListEdges.push_back( node_index ); }
                //I imagine that we will need it for the nn_search where the node_index will came from a search function

                //!<  I saw a virtual class Visitors that should be redefined
                //!<  but from the comments I do not understant how it works
                //!<  of course it's my bad and my bad c++ code understanding level .... :-(


                //mrpt::hybridnav::TPath::TPlannedPath get_sh(); // &path
                //for our case the right methohortest_patd should be “visitBreadthFirst”, how this works in the
                //mrpt::graphs::CDirectedTree class, should we redefine it?

//!<            void erase();  //this is not necessary because it should be hereditated from CDirectedTree in clear();
//!<            why it not work when I define the TMotionTree im the test code???
//!<            should I redefine it here?
            };

		/** @name TMotionSE2 class for planning in SE2 */
		class TMotionsSE2
		{
		   public:
			   TMotionsSE2 () :
							state( ),    //!< should the state be initialized as NULL or something else?
							// like add in the namespace a #define INVALID_STATE  mrpt::poses::TPose2D( )
							// Is this already defined somewhere in MRPT (example in mrpt::utils:: blablabla
                            // check this please!!!
							cost( 0.0 )
							{}
				mrpt::poses::TPose2D state;  //!< state in SE2 as 2D pose (x, y, phi)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

		/** @name TMotionSE3 class for planning in SE3 */
		class TMotionsSE3
		{
		   public:
			   TMotionsSE3() :
							state( ),   //As before
							cost( 0.0 )
							{}
				mrpt::poses::TPose3D state;  //!< state in SE2 as 3D pose (x, y, z, yaw, pitch, roll)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

        /** @name TMotionSE2_TP class for planning in SE2 and TP-space */
		class TMotionsSE2_TP
		{
		   public:
			   TMotionsSE2_TP () :
							state( ),   //As before
							cost( 0.0 ),
							ptg_index ( 0 ), ptg_K ( 0 ), ptg_dist ( 0.0 )   //these are all PTGs parameters, do we need more?
							{}
				mrpt::poses::TPose2D state;  //!< state in SE2 as 2D pose (x, y, phi)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
				int ptg_index;          //!< indicate the type of trajectory used for this motion
				int ptg_K;              //!< identify the trajectory number K of the type ptg_index
				double ptg_dist;        //!< identify the lenght of the trajectory for this motion
		};

        /** @name TMotionSE3_TP class for planning in SE3 and TP-space */
		class TMotionsSE3_TP
		{
		   public:
			   TMotionsSE3_TP () :
							state( ), //As before
							cost( 0.0 ),
							ptg_index ( 0 ), ptg_K ( 0 ), ptg_dist ( 0.0 )   //these are all PTGs parameters, do we need more?
							{}
				mrpt::poses::TPose3D state;  //!< state in SE2 as 2D pose (x, y, phi)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
                int ptg_index;          //!< indicate the type of trajectory used for this motion
				int ptg_K;              //!< identify the trajectory number K of the type ptg_index
				double ptg_dist;        //!< identify the lenght of the trajectory for this motion
		};

        typedef TMotionsTree<TMotionsSE2> TMotionsTreeSE2;        //!< tree datastructure for planning RRT in SE2
        typedef TMotionsTree<TMotionsSE3> TMotionsTreeSE3;        //!< tree datastructure for planning RRT in SE3
        typedef TMotionsTree<TMotionsSE2_TP> TMotionsTreeSE2_TP;  //!< tree datastructure for planning RRT in SE2 amd TP-space
        typedef TMotionsTree<TMotionsSE3_TP> TMotionsTreeSE3_TP;  //!< tree datastructure for planning RRT in SE3 amd TP-space

  }
}
#endif // TMotions_H
