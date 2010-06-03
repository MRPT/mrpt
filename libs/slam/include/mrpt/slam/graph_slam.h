/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <mrpt/poses/CNetworkOfPoses.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/** An algorithm for optimizing a network of 2D or 3D pose links based on Levenberg-Marquardt error minimization.
		  *  It is computed the list of optimal, consistent global coordinates for each node in the graph. Relative poses
		  *  are represented by Gaussians with a mean and a covariance, which is also taken into account in the optimization.
		  *
		  *  The algorithm can be seen as an extension to the work of Lu & Milios
		  *    - Globally Consistent Range Scan Alignment for Environment Mapping, 1997.
		  *
		  * \param pose_graph [IN] The graph of pose constraints. It can be of either type CNetworkOfPoses2D or CNetworkOfPoses3D.
		  * \param optimal_poses [OUT] The consistent, global coordinates of all the pose nodes in the graph.
		  * \param origin_pose [IN] Due to the degrees of freedom, one arbitrary pose is set to the origin (0,0,0). This parameter defines the pose ID of the pose to be taken as the origin, and the default value (ID:-1) will select the reference pose in the first link in pose_graph.
		  * \param max_iterations [IN] The maximum number of iterations. If it is set to 0, the global poses computed as initial values from Dijkstra will be returned.
		  * 
		  * \note Output covariances should be not considered, they are not computed yet.
		  * \return The average square root error in the optimized pose network.
		  * \sa optimizePoseGraph_stogradesc
		  */
		template <class CPOSE>
		double SLAM_IMPEXP optimizePoseGraph_levmarq(
			const CNetworkOfPoses<CPOSE>  &pose_graph,
			std::map<size_t,CPOSE>	      &optimal_poses,
			const size_t                   max_iterations = 100,
			const size_t                   origin_pose = static_cast<size_t>(-1)
			);

		/** An algorithm for optimizing a network of 2D or 3D pose links based on stochastic gradient descent.
		  *  It is computed the list of optimal, consistent global coordinates for each node in the graph. Relative poses
		  *  are represented by Gaussians with a mean and a covariance, which is also taken into account in the optimization.
		  *
		  *  This class is a C++ implementation of the work proposed in the paper:
		  *    - Edwin Olson, John Leonard, Seth Teller, "Fast Iterative Optimization of Pose Graphs with Poor Initial Estimates", ICRA 2006.
		  *
		  * \param pose_graph [IN] The graph of pose constraints. It can be of either type CNetworkOfPoses2D or CNetworkOfPoses3D.
		  * \param optimal_poses [OUT] The consistent, global coordinates of all the pose nodes in the graph.
		  * \param origin_pose [IN] Due to the degrees of freedom, one arbitrary pose is set to the origin (0,0,0). This parameter defines the pose ID of the pose to be taken as the origin, and the default value (ID:-1) will select the reference pose in the first link in pose_graph.
		  * \param max_iterations [IN] The maximum number of iterations. If it is set to 0, the global poses computed as initial values from Dijkstra will be returned.
		  * 
		  * \return The average square root error in the optimized pose network.
		  * \sa optimizePoseGraph_levmarq
		  */
		template <class CPOSE>
		double SLAM_IMPEXP optimizePoseGraph_stogradesc(
			const CNetworkOfPoses<CPOSE>  &pose_graph,
			std::map<size_t,CPOSE>	      &optimal_poses,
			const size_t                   max_iterations = 100,
			const size_t                   origin_pose = static_cast<size_t>(-1)
			);

	} // End of namespace
} // End of namespace

#endif
