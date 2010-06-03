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

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/poses/CNetworkOfPoses.h>
#include <mrpt/slam/graph_slam.h>

#include <mrpt/math/dijkstra.h>
#include <mrpt/math/jacobians.h>
#include <mrpt/math/CLevenbergMarquardt.h>

#include <list>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace std;



/*---------------------------------------------------------------
					optimizePoseGraph_stogradesc
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace slam
	{


	}
}




template <class CPOSE>
double mrpt::slam::optimizePoseGraph_stogradesc(
	const CNetworkOfPoses<CPOSE>  &pose_graph,
	std::map<size_t,CPOSE>	      &optimal_poses,
	const size_t                   max_iterations,
	const size_t                   origin_pose
	)
{
	MRPT_START

	//return LM_info.final_sqr_err;

	return 0;

	MRPT_END
}


// Explicit instantations:
template double SLAM_IMPEXP mrpt::slam::optimizePoseGraph_stogradesc(
	const CNetworkOfPoses<CPosePDFGaussian> &pose_graph,
	std::map<size_t,CPosePDFGaussian>	    &optimal_poses,
	const size_t                   max_iterations,
	const size_t  origin_pose );

template double SLAM_IMPEXP mrpt::slam::optimizePoseGraph_stogradesc(
	const CNetworkOfPoses<CPose3DPDFGaussian> &pose_graph,
	std::map<size_t,CPose3DPDFGaussian>	    &optimal_poses,
	const size_t                   max_iterations,
	const size_t  origin_pose );

//MRPT_TODO("Add specializations for graphs of 3D poses with quaternions as well.")


