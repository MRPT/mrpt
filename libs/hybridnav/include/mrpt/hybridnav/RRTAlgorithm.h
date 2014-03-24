/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef RRTAlgorithm_H
#define RRTAlgorithm_H

#include <mrpt/utils.h>

namespace mrpt
{
  namespace hybridnav
  {

        /** This class implement an header-only RRT algorithm for hybrid navigation
         *
         *  <b>Usage:</b><br>
         *		- write me
         *
         *
         *  <b>About the algorithm:</b><br>
         *    - write me
         *
         * <b>Changes history</b>
         *      - 06/MAR/2014: Creation (MB)
         *  \ingroup mrpt_hybridnav_grp
         */

		template <class  POSE, class MOTIONS>   //this is to be used for any pose and any motion, how to define POSE and MOTIONS??
		class RRT
		{
          public:

            RRT ()
                {
                //define default values for the goalBias and maxLength in RRT
                goalBias_ = 0.05;
                maxLength_ = 0.0;   //the setting of maxLength should be mandatory for the user
                } /** Constr   */

            // do we need the destrctor to free memory alfter the solver? ~RRT()


            /** \brief Set the goal bias */
            inline void setGoalBias (double goalBias) {  goalBias_ = goalBias; }

            /** \brief Get the goal bias */
            inline double getGoalBias () {  return goalBias_; }

            /** \brief Set the max length of edges for RRT algorithm */
            inline void setLength (double maxLength) {  maxLength_ = maxLength; }

            /** \brief Get the goal bias */
            inline double getLength () {  return maxLength_; }

            /** \brief Setup operation befor the planner start */
            void setup ();


            /** \brief Planning using RRT
            *  Algorithm as expressed in karaman-frazzoli 2011
            *      1.  V <-- {x_init}; E<-0
            *      2.  for i=1, ... n do
            *      3.      x_ran <-- SampleFree_i
            *      4.      x_nearest <-- Nearest (G=(V,E), x_rand)
            *      5.      x_new <-- Steer (x_nearest, x_rand)
            *      6.      if ObstacleFree (x_nearest, x_new)  then
            *      7.          V <-- V U {x_new};   E <--E U {(x_nearest, x_new)}
            *      8.  return (G= (V,E)
            **/  //TODO implement this into the cpp file
			void plan(const POSE &start_pose, const POSE &goal_pose);
//			{
			// Like in the RRT pseudocode,
			// try  to be as generic as possible,
			// assuming the existence of methods to be implemented by the “user” in derived classes
			// e.g.:
//			this->addEdge(start_pose, start_pose);
//			};


            /** \brief The goalBias_ is the probability to pick the target as the next random state */
            double goalBias_;

            /** \brief The max leght of edges  */
            double maxLength_;

			mrpt::poses::CPose2D start_pose;
			mrpt::poses::CPose2D goal_pose;
		};
  }
}
#endif // RRTAlgorithm_H
