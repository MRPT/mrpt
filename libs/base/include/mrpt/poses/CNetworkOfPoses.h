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
#ifndef CONSTRAINED_POSE_NETWORK_H
#define CONSTRAINED_POSE_NETWORK_H

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/graphs.h>

namespace mrpt
{
	namespace poses
	{
		/** A network of links constraining the relative pose of pairs of nodes, indentified by their numeric IDs (of type size_t).
		  *  A link between nodes "i" and "j", that is, the pose \f$ p_{ij} \f$ or relative position of "j" with respect to "i", 
		  *   is maintained as a multivariate Gaussian distribution.
		  *
		  * Valid values for the argument CPOSE are CPosePDFGaussian and CPose3DPDFGaussian, which correspond to the 
		  *  typedefs CNetworkOfPoses2D and CNetworkOfPoses3D.
		  *
		  * Access to all the links can be done through normal std::map methods, plus the new method "insertLink"
		  *
		  * \sa CPosePDFGaussian,CPose3DPDFGaussian,CPose3DQuatPDFGaussian, mrpt::slam::optimizePoseGraph_levmarq
		  */
		template<class CPOSE>
		class CNetworkOfPoses : public mrpt::math::CDirectedGraph< CPOSE >
		{
		public:
			typedef CPOSE type_poses;  //!< The type of PDF poses on the network links
			typedef std::map<size_t,CPOSE> type_global_poses;  //!< A map of pose IDs to their global coordinates - can be used as second parameter of optimizePoseGraph_levmarq
		};

		typedef CNetworkOfPoses<CPosePDFGaussian> CNetworkOfPoses2D;	//!< The specialization of CNetworkOfPoses for poses of type CPosePDFGaussian
		typedef CNetworkOfPoses<CPose3DPDFGaussian> CNetworkOfPoses3D;	//!< The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussian

	} // End of namespace
} // End of namespace

#endif
