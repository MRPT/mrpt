/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef TPATH_H
#define TPATH_H

#include <mrpt/poses.h>
#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::poses;

namespace mrpt
{
  namespace hybridnav
  {

        /** This class contains methods for path data structures and handling
         *
         *  <b>Usage:</b><br>
         *		- write me
         *
         *
         *  <b>About the algorithm:</b><br>
         *
         *
         * <b>Changes history</b>
         *		- 12/DEC/2013: Creation (MB).
         *      - 21/FEB/2014: Refactoring (MB)
         *  \ingroup mrpt_hybrid_grp
         */
        class TPath //: public PTRRT_Navigator
        {
            public:
                TPath();
                virtual ~TPath();


            /** @name TPath data structures */
                struct TPathData
                {
                    TPathData() :
                        p(0,0,0),
                        max_v(0.1),max_w(0.2),
                        trg_v(0.1),ind_ptg(0),
                        K(0), ptg_dist(0.0)
                    {}

                    TPose2D p;			    //!< Coordinates are "global"
                    double max_v, max_w;	//!< Maximum velocities along this path segment.
                    double trg_v;			//!< Desired linear velocity at the target point, ie: the robot should program its velocities such as after this arc the speeds are the given ones.
                    int ind_ptg;            //!< The PTG index is needed in order to get the correct "trajectory" path
                    int K;                  //!< K from PTGs
                    float ptg_dist;         //!< PTGs distance

                };

                struct TPlannedPath : public std::vector<TPathData>   //!< An ordered vector of path poses.
                {

                    bool isApproximate;  //!< the path is approximate if the target is not reached

                    /** Return the path length
                      * \note in Euclidean metric
                      * \note the planned path is suppose to be calculated
                      */
                    double lengthEuclidean() const;

                    /** Return the path length
                      * \note in psm pseudo-meter (PTGs metric)
                      * \note the planned path is suppose to be calculated
                      */
                    double lengthPsm() const;

                    /** Save the path in a file
                      * \note the firsts line include a header containing the max velocity linear/angular
                      * \note the format is: x y phi ptg_index K ptg_dist
                      * \note if the folder doesn't exist it will not be created
                      */
                    bool  save_to_text_file(const std::string &file) const;

                    /** Given a robot pose, get the closest path index
                      * \note this function consider only the euclidean distance
                      */
                    int getClosestPathIndex (mrpt::math::TPose2D &robotPose) const;

                    /** Get the next point to follow for the reactive method, the idea is reactively navigate
                      * to the target through a set of points as a "Hop-o'-My-Thumb" path (or breadcrumbs path)
                      * \note the path have to be not-empty.
                      * \return false if can't determine the next point.
                      */
                    bool getBreadcrumbPoint(mrpt::hybridnav::TPath::TPathData &out_next_point,
                                            mrpt::math::TPose2D &robotPose,
                                            mrpt::math::TPose2D &m_target_pose,
                                            double breadcrumb_dist) ;

                };


            protected:

            private:
        };

  }
}
#endif // TPATH_H
