/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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


		/** A stub for ransac_detect_3D_planes() with the points given as a mrpt::maps::CPointsMap
		  */
		template <class POINTSMAP>
		inline void ransac_detect_3D_planes(
			const POINTSMAP * points_map,
			std::vector<std::pair<size_t,TPlane> >   &out_detected_planes,
			const double           threshold,
			const size_t           min_inliers_for_valid_plane
			)
		{
			CVectorFloat xs,ys,zs;
			points_map->getAllPoints(xs,ys,zs);
			ransac_detect_3D_planes(xs,ys,zs,out_detected_planes,threshold,min_inliers_for_valid_plane);
		}

		/** @} */
		/** @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
