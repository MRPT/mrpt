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
#ifndef  ransac_optimizers_H
#define  ransac_optimizers_H

#include <mrpt/math/ransac.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace math
	{
		using std::vector;

		/** @addtogroup ransac_grp
		  * @{ */

		/** @name RANSAC detectors
			@{
		  */

		/** Fit a number of 3-D planes to a given point cloud, automatically determining the number of existing planes by means of the provided threshold and minimum number of supporting inliers.
		  * \param out_detected_planes The output list of pairs: number of supporting inliers, detected plane.
		  * \param threshold The maximum distance between a point and a temptative plane such as the point is considered an inlier.
		  * \param min_inliers_for_valid_plane  The minimum number of supporting inliers to consider a plane as valid.
		  */
		template <typename NUMTYPE>
		void BASE_IMPEXP ransac_detect_3D_planes(
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &z,
			std::vector<std::pair<size_t,TPlane> >   &out_detected_planes,
			const double           threshold,
			const size_t           min_inliers_for_valid_plane = 10
			);

		/** Fit a number of 2-D lines to a given point cloud, automatically determining the number of existing lines by means of the provided threshold and minimum number of supporting inliers.
		  * \param out_detected_lines The output list of pairs: number of supporting inliers, detected line.
		  * \param threshold The maximum distance between a point and a temptative line such as the point is considered an inlier.
		  * \param min_inliers_for_valid_line  The minimum number of supporting inliers to consider a line as valid.
		  */
		template <typename NUMTYPE>
		void BASE_IMPEXP ransac_detect_2D_lines(
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
			std::vector<std::pair<size_t,TLine2D> >   &out_detected_lines,
			const double           threshold,
			const size_t           min_inliers_for_valid_line = 5
			);


		/** A stub for ransac_detect_3D_planes() with the points given as a mrpt::slam::CPointsMap
		  */
		template <class POINTSMAP>
		inline void ransac_detect_3D_planes(
			const POINTSMAP * points_map,
			std::vector<std::pair<size_t,TPlane> >   &out_detected_planes,
			const double           threshold,
			const size_t           min_inliers_for_valid_plane
			)
		{
			vector_float xs,ys,zs;
			points_map->getAllPoints(xs,ys,zs);
			ransac_detect_3D_planes(xs,ys,zs,out_detected_planes,threshold,min_inliers_for_valid_plane);
		}

		/** @} */
		/** @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
