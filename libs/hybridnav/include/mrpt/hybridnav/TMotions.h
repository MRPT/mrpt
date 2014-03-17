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
        class TMotionTree : public mrpt::graphs::CDirectedTree< TMotions >
            {
            public:

                //methods we need here are:
                void add_element();  //and just refer to list<> or vector<> class method in CDirectedTree
                void get_element();  //I imagine that we will need it for the nn_search
                mrpt::hybridnav::TPath::TPlannedPath get_sh(); // &path
                //for our case the right methohortest_patd should be 				     	“visitBreadthFirst”, how this works in the
                //mrpt::graphs::CDirectedTree class, should we redefine it?
                void erase();
            };/**/

		/** @name TMotionSE2 class for planning in SE2 */
		class TMotionsSE2
		{
		   public:
			   TMotionsSE2 () :
							state( ),    //should the state be initialized as NULL?
							cost( 0.0 )  //should the cost be initialized as 0.0 to avoid possible memory errors?
							{}
				mrpt::poses::TPose2D state;  //!< state in SE2 as 2D pose (x, y, phi)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

		/** @name TMotionSE3 class for planning in SE3 */
		class TMotionsSE3
		{
		   public:
			   TMotionsSE3() :
							state( ),
							cost( )
							{}
				mrpt::poses::TPose3D state;  //!< state in SE2 as 3D pose (x, y, z, yaw, pitch, roll)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
		};

        /** @name TMotionSE2_TP class for planning in SE2 and TP-space */
		class TMotionsSE2_TP
		{
		   public:
			   TMotionsSE2_TP () :
							state( ),
							cost( ),
							ptg_index ( ), ptg_K ( ), ptg_dist ( )
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
							state( ),
							cost( ),
							ptg_index ( ), ptg_K ( ), ptg_dist ( )
							{}
				mrpt::poses::TPose3D state;  //!< state in SE2 as 2D pose (x, y, phi)
				double cost;                //!< cost associated to each motion, this should be defined by the user according to a spefic cost function
                int ptg_index;          //!< indicate the type of trajectory used for this motion
				int ptg_K;              //!< identify the trajectory number K of the type ptg_index
				double ptg_dist;        //!< identify the lenght of the trajectory for this motion
		};

        typedef TMotionTree<TMotionsSE2> TMotionTreeSE2;        //!< tree datastructure for planning RRT in SE2
        typedef TMotionTree<TMotionsSE3> TMotionTreeSE3;        //!< tree datastructure for planning RRT in SE3
        typedef TMotionTree<TMotionsSE2_TP> TMotionTreeSE2_TP;  //!< tree datastructure for planning RRT in SE2 amd TP-space
        typedef TMotionTree<TMotionsSE3_TP> TMotionTreeSE3_TP;  //!< tree datastructure for planning RRT in SE3 amd TP-space


  }
}
#endif // TMotions_H
