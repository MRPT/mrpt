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
							state( ), 
							parent(),  //this is still a missing part and we should implement with the tree structure
							cost( )
							{}

			   ~TMotionsSE2();

				mrpt::poses::TPose2D state;   //maybe this should be a pointer too
				TMotionsSE2 *parent;   // this will be a pointer to the parent motion in the exploration tree
				double cost;
			
				typedef TMotionTree<TMotionsSE2> TMotionTree;
		};

		/** @name TMotionSE2 class for planning in SE3 */
		class TMotionsSE3 
		{
		   public:
			   TMotionsSE3() : 
							state( ), 
							parent(),  //this is still a missing part and we should implement with the tree structure
							cost( )
							{}

			   ~TMotionsSE3();

				mrpt::poses::TPose3D state;   //maybe this should be a pointer too
				TMotionsSE2 *parent;   // this will be a pointer to the parent motion in the exploration tree
				double cost;
		
				typedef TMotionTree<TMotionsSE3> TMotionTree;
		};



  }
}
#endif // TMotions_H
