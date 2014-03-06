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
		class TMotions // no inheritances for now 
        {
            public:
                TMotions();
                ~TMotions();

			 /** @name TMotionSE2 data structure */
				struct  TMotionSE2 {
						TMotionSE2 () :
							state( ), //parent(),  //this is still a missing part and we should implement with the tree structure
							cost( ) 
						{}
				mrpt::poses::TPose2D state; 
				double cost;
				};
  
				/** @name TMotionSE2_TPS data structure for planning in SE2 and TP-Space */
				struct  TMotionSE2_TPS {				
						TMotionSE2_TPS () :
								state( ), //parent(),  
								cost( ) //, PTG_infos()
						{}
						mrpt::poses::TPose2D state; 
						// PTG_infos()   //define what we need here !!
						double cost;
				};

				/** @name TMotionSE3 data structure */
				struct  TMotionSE3 {
						TMotionSE3 () :
							state( ), //parent(),  
							cost( ) 
						{}
				mrpt::poses::TPose3D state; 
				double cost;
				};

				/** @name TMotionSE3_TPS data structure for planning in SE3 and TP-Space */
				struct  TMotionSE3_TPS {				
						TMotionSE3_TPS () :
								state( ), //parent(),  
								cost( ) //, PTG_infos()
						{}
						mrpt::poses::TPose3D state; 
						// PTG_infos()   //define what we need here !!
						double cost;
				};

			//is it possible to create a motion using something like ?
			//mrpt::hybridnav::TMotions::TMotionSE2 my_se2_motion;


            protected:

            private:
        };


		template<typename T> class TMotionTree : public mrpt::graphs::CDirectedTree< TMotions >
          {
			//methods we need here are:
			void add_element();  //and just refer to list<> or vector<> class method in 				 CDirectedTree
			void get_element();  //I imagine that we will need it for the nn_search 
			mrpt::hybridnav::TPath::TPlannedPath get_sh(); // &path  
			//for our case the right methohortest_patd should be 				     	“visitBreadthFirst”, how this works in the 
			 //mrpt::graphs::CDirectedTree class, should we redefine it? 
			void erase();
			
			typedef TMotionTree<TMotions> TMotionTree;
		};

  }
}
#endif // TMotions_H
