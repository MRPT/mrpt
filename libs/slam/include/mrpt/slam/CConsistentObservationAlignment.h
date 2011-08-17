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
#ifndef CCONSISTENTOBSERVATIONALIGNMENT_H
#define CCONSISTENTOBSERVATIONALIGNMENT_H

#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/COccupancyGridMap2D.h>

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixTemplateObjects.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::math;

		/** An algorithm for globally, consistent alignment of a
		 *    sequence of observations.
		 *  This algorithm is based on the work of Lu & Milios
		 *    [Globally Consistent Range Scan Alignment for Environment Mapping, 1997]
		 *    for a global optimal estimation of laser range scan poses, but in
		 *    this case it has been extended to include any type of
		 *    observations as long as points-map-like operators are implemented over them.
		 *    <br>
 		 *    <b>This class work in the following way:</b><br>
		 *      The input is a set of observations with associated "global" poses. This is
		 *       supplied with a "CSimpleMap" object, but the probabilistic poses
		 *       are ignored since only the mean values for the pose of each node are taken.<br>
		 *      After invoking the algorithm with CConsistentObservationAlignment::execute(),
		 *       a new "CSimpleMap" object is returned, where the
		 *      NOTE: The first node on the input map is used as reference and therefore
		 *    		  its pose is the only one which will never change.
		 *    
	     * \note This class is superseded by more modern implementations of graph-slam. See mrpt::graphslam
		 *
		 * \sa CSimpleMap, CPosePDF, CObservation, utils::CDebugOutputCapable   \ingroup mrpt_slam_grp
		 */
		class SLAM_IMPEXP CConsistentObservationAlignment : public mrpt::utils::CDebugOutputCapable
		{
		protected:
			/** A sequence of probabilistic poses:
			  */
			typedef std::vector<CPosePDFGaussianPtr>	vector_posesPdf;

		public:

			CConsistentObservationAlignment();

			/** The options for the method.
			  */
			struct SLAM_IMPEXP TOptions
			{
				/** Initialization:
				  */
				TOptions();

				/** If set to true (default), the matching will be performed against grid maps, instead of points maps:
				  */
				bool		matchAgainstGridmap;

				/** The resolution of the grid maps (default = 0.02m)
				  */
				float		gridMapsResolution;

				/** The options for building temporary maps.
				  */
				CPointsMap::TInsertionOptions			pointsMapOptions;

				/** The options for building temporary maps.
				  */
				COccupancyGridMap2D::TInsertionOptions	gridInsertOptions;

				/** The options for the ICP algorithm.
				  */
				CICP::TConfigParams				icpOptions;

			} options;

			/** Executes the algorithm. See description in CConsistentObservationAlignment.
			 *
			 * \param inputMap The input to the algorithm: a set of nodes situated (with global coordinates) and observations from each node.
			 * \param outputMap The globally consistent map, where probabilitic poses are filled with gaussian PDFs, where the mean is the globally optimal estimation and the covariance is also computed.
			 */
			void  execute(
				CSimpleMap		&inputMap,
				CSimpleMap		&outputMap );

			/** This alternate method provides the basic consistent alignment algorithm to any user-supplied matrix of pose constrainsts, returning the optimal poses of all the nodes relative to the first one.
			 * \param in_PoseConstraints This is a NxN matrix where element M(i,j) is the pose constrainst between node "i" and "j". Please, fill out only the upper-triangle part of the matrix (diagonal and lowe-part entries are not used).
			 * \param out_OptimalPoses The 1xN vector with the consistent global poses of all nodes, where the first node is always at (0,0,0deg).
			 */
			static void  optimizeUserSuppliedData(
				math::CMatrixTemplateObjects<CPosePDFGaussian>		&in_PoseConstraints,
				math::CMatrixTemplateObjects<CPosePDFGaussian>		&out_OptimalPoses );

			/** A textual description for the implemented algorithm.
			 */
			std::string		getAlgorithmName();

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
