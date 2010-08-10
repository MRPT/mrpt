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
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/math/graphs.h>

#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
	namespace poses
	{
		/** A network of links constraining the relative pose of pairs of nodes, indentified by their numeric IDs (of type TPoseID, actually a size_t).
		  *  A link between nodes "i" and "j", that is, the pose \f$ p_{ij} \f$ or relative position of "j" with respect to "i",
		  *   is maintained as a multivariate Gaussian distribution.
		  *
		  * Valid values for the argument CPOSE are CPosePDFGaussian and CPose3DPDFGaussian, which correspond to the
		  *  typedefs CNetworkOfPoses2D and CNetworkOfPoses3D. There are also "information-form" versions which
		  *  hold the inverse covariance matrices (see CNetworkOfPoses2DInf and CNetworkOfPoses3DInf).
		  *
		  * Access to the list of all edges can be done through normal std::map methods, plus the method mrpt::math::CDirectedGraph::insertEdge().
		  *  See the base class mrpt::math::CDirectedGraph for a list of other utility methods.
		  *
		  *  This class is the base for representing networks of poses, which are the main data type of a series
		  *   of SLAM algorithms implemented in the library mrpt-slam, in the namespace mrpt::graphslam.
		  *
		  * \sa CPosePDFGaussian,CPose3DPDFGaussian,CPose3DQuatPDFGaussian, mrpt::graphslam
		  */
		template<class CPOSE>
		class CNetworkOfPoses : public mrpt::math::CDirectedGraph< CPOSE >
		{
		public:
			/** @name Typedef's
			    @ { */

			typedef mrpt::math::CDirectedGraph< CPOSE > BASE; //!< The base class "CDirectedGraph<CPOSE>"
			typedef CPOSE type_poses;  //!< The type of PDF poses on the network links
			typedef std::map<typename BASE::TNodeID,CPOSE> type_global_poses;  //!< A map of pose IDs to their global coordinates - can be used as second parameter of optimizePoseGraph_levmarq

			/** @} */

		};


		// Define serializable versions of the template above for each specific kind of "edge":

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses2D )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses3D )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses2DInf )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses3DInf )

		/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussian, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class CNetworkOfPoses2D : public CNetworkOfPoses<CPosePDFGaussian>, public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( CNetworkOfPoses2D )	// This must be added to any CSerializable derived class:
		public:

		};

		/** The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussian, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class CNetworkOfPoses3D : public CNetworkOfPoses<CPose3DPDFGaussian>, public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( CNetworkOfPoses3D )	// This must be added to any CSerializable derived class:
		public:

		};

		/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussianInf, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class CNetworkOfPoses2DInf : public CNetworkOfPoses<CPosePDFGaussianInf>, public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( CNetworkOfPoses2DInf )	// This must be added to any CSerializable derived class:
		public:

		};

		/** The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussianInf, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class CNetworkOfPoses3DInf : public CNetworkOfPoses<CPose3DPDFGaussianInf>, public mrpt::utils::CSerializable
		{
			DEFINE_SERIALIZABLE( CNetworkOfPoses3DInf )	// This must be added to any CSerializable derived class:
		public:

		};


	} // End of namespace
} // End of namespace

#endif
