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
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>

namespace mrpt
{
	namespace poses
	{
		using mrpt::utils::TNodeID;
		template<class CPOSE> class CNetworkOfPoses;	// Forward decl. needed by detail functions.

		/** Internal functions for MRPT */
		namespace detail
		{
			template<class CPOSE> void BASE_IMPEXP save_graph_of_poses_from_text_file(const CNetworkOfPoses<CPOSE> *g, const std::string &fil);
			template<class CPOSE> void BASE_IMPEXP load_graph_of_poses_from_text_file(CNetworkOfPoses<CPOSE>*g, const std::string &fil);
		}

		/** A network of links constraining the relative pose of pairs of nodes, indentified by their numeric IDs (of type TPoseID).
		  *  A link between nodes "i" and "j", that is, the pose \f$ p_{ij} \f$ or relative position of "j" with respect to "i",
		  *   is maintained as a multivariate Gaussian distribution.
		  *
		  *  Two main members store all the information in this class:
		  *		- \a edge  (in base class mrpt::math::CDirectedGraph<>::edge): A map from pairs of node ID -> pose constraints with uncertainty.
		  *		- \a nodes : A map from node ID -> estimated pose of that node.
		  *
		  * Valid values for the argument CPOSE are CPosePDFGaussian and CPose3DPDFGaussian, which correspond to the
		  *  typedefs CNetworkOfPoses2D and CNetworkOfPoses3D. There are also "information-form" versions which
		  *  hold the inverse covariance matrices (see CNetworkOfPoses2DInf and CNetworkOfPoses3DInf).
		  *  These information-form classes are <b>preferred for efficiency</b>.
		  *
		  *  This class is the base for representing networks of poses, which are the main data type of a series
		  *   of SLAM algorithms implemented in the library mrpt-slam, in the namespace mrpt::graphslam.
		  *
		  *  For tools to visualize graphs as 2D/3D plots, see the namespace mrpt::opengl::graph_tools in the library mrpt-opengl.
		  *
		  * \sa CPosePDFGaussian,CPose3DPDFGaussian,CPose3DQuatPDFGaussian, mrpt::graphslam
		  */
		template<class CPOSE>
		class CNetworkOfPoses : public mrpt::math::CDirectedGraph< CPOSE >
		{
		public:
			/** @name Typedef's
			    @{ */

			/** The base class "CDirectedGraph<CPOSE>" */
			typedef mrpt::math::CDirectedGraph< CPOSE > BASE;

			/** The type of PDF poses in the contraints (edges) */
			typedef CPOSE                               contraint_t;

			/** A map from pose IDs to their global coordinates estimates, with uncertainty */
			typedef std::map<TNodeID,CPOSE>             global_poses_pdf_t;

			/** A map from pose IDs to their global coordinates estimates, without uncertainty (the "most-likely value") */
			typedef std::map<TNodeID,typename CPOSE::type_value> global_poses_t;

			/** @} */


			/** @name Data members
			    @{ */

			/** The nodes (vertices) of the graph, with their estimated "global" (with respect to \a root) position, without an associated covariance. */
			global_poses_t  nodes;

			/** The ID of the node that is the origin of coordinates, used as reference by all coordinates in \nodes. By default, root is the ID "0". */
			TNodeID         root;

			/** @} */


			/** @name I/O file methods
			    @{ */

			/** Saves the graph to a file using MRPT's binary serialization format.
			  * \sa loadFromBinaryFile, saveToTextFile
			  * \exception On any error
			  */
			void saveToBinaryFile( const std::string &fileName ) const {
				mrpt::utils::CFileOutputStream fil(fileName);
				fil.WriteObject(this);
			}

			/** Loads the graph from a binary file.
			  * \sa saveToBinaryFile, loadFromTextFile
			  * \exception On any error
			  */
			void loadFromBinaryFile( const std::string &fileName ) {
				mrpt::utils::CFileInputStream fil(fileName);
				fil.ReadObject(this);
			}

			/** Saves to a text file in the format used by TORO & HoG-man (more on the format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a> )
			  *  For 2D graphs only VERTEX2 & EDGE2 entries will be saved, and VERTEX3 & EDGE3 entries for 3D graphs.
			  *  Note that EQUIV entries will not be saved, but instead several EDGEs will be stored between the same node IDs.
			  * \sa saveToBinaryFile, loadFromTextFile
			  * \exception On any error
			  */
			void saveToTextFile( const std::string &fileName ) const {
				mrpt::poses::detail::save_graph_of_poses_from_text_file(this,fileName);
			}

			/** Loads from a text file in the format used by TORO & HoG-man (more on the format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a> )
			  *   Recognized line entries are: VERTEX2, VERTEX3, EDGE2, EDGE3, EQUIV.
			  *   If an unknown entry is found, a warning is dumped to std::cerr (only once for each unknown keyword).
			  *   An exception will be raised if trying to load a 3D graph into a 2D class (in the opposite case, missing 3D data will default to zero).
			  * \sa loadFromBinaryFile, saveToTextFile
			  * \exception On any error, as a malformed line or loading a 3D graph in a 2D graph.
			  */
			void loadFromTextFile( const std::string &fileName ) {
				mrpt::poses::detail::load_graph_of_poses_from_text_file(this,fileName);
			}

			/** @} */

			/** @name Other methods
			    @{ */

			/** Empty all edges, nodes and set root to ID 0. */
			inline void clear() {
				BASE::edges.clear();
				nodes.clear();
				root = 0;
			}

			/** Return number of nodes in the list \nodes of global coordinates (may be differente that all nodes appearing in edges)
			  * \sa mrpt::math::CDirectedGraph::countDifferentNodesInEdges
			  */
			inline size_t nodeCount() const { return nodes.size(); }

			/**  @} */

			/** Default constructor (just sets root to "0") */
			inline CNetworkOfPoses() : root(0) { }
			virtual ~CNetworkOfPoses() { }
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
