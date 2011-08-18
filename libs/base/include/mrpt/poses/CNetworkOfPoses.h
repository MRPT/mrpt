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
#ifndef CONSTRAINED_POSE_NETWORK_H
#define CONSTRAINED_POSE_NETWORK_H

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/math/graphs.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

#include <mrpt/utils/stl_extensions.h>

// The main class in this file is CNetworkOfPoses, a generic basic template for
//  predefined 2D/3D graphs of pose contraints.

namespace mrpt
{
	namespace poses
	{
		using mrpt::utils::TNodeID;
		template<class CPOSE,class MAPS_IMPLEMENTATION > class CNetworkOfPoses;	// Forward decl. needed by detail functions.

		/** Internal functions for MRPT */
		namespace detail
		{
			template<class CPOSE,class MAPS_IMPLEMENTATION> void BASE_IMPEXP save_graph_of_poses_from_text_file(const CNetworkOfPoses<CPOSE,MAPS_IMPLEMENTATION> *g, const std::string &fil);
			template<class CPOSE,class MAPS_IMPLEMENTATION> void BASE_IMPEXP load_graph_of_poses_from_text_file(CNetworkOfPoses<CPOSE,MAPS_IMPLEMENTATION>*g, const std::string &fil);
			template<class CPOSE,class MAPS_IMPLEMENTATION> void BASE_IMPEXP graph_of_poses_dijkstra_init(CNetworkOfPoses<CPOSE,MAPS_IMPLEMENTATION>*g);
			template<class CPOSE,class MAPS_IMPLEMENTATION> size_t BASE_IMPEXP graph_of_poses_collapse_dup_edges(CNetworkOfPoses<CPOSE,MAPS_IMPLEMENTATION>*g);
			template<class CPOSE,class MAPS_IMPLEMENTATION> double BASE_IMPEXP graph_edge_sqerror(const CNetworkOfPoses<CPOSE,MAPS_IMPLEMENTATION>*g, const typename mrpt::math::CDirectedGraph<CPOSE>::edges_map_t::const_iterator &itEdge, bool ignoreCovariances );
		}

		/** A directed graph of pose constraints, with edges being the relative pose between pairs of nodes indentified by their numeric IDs (of type TNodeID).
		  *  A link or edge between two nodes "i" and "j", that is, the pose \f$ p_{ij} \f$, holds the relative position of "j" with respect to "i".
		  *   These poses are stored in the edges in the format specified by the template argument CPOSE. Users should employ the following derived classes
		  *   depending on the desired representation of edges:
		  *      - mrpt::poses::CNetworkOfPoses2D    : 2D edges as a simple CPose2D (x y phi)
		  *      - mrpt::poses::CNetworkOfPoses3D    : 3D edges as a simple CPose3D (x y z yaw pitch roll)
		  *      - mrpt::poses::CNetworkOfPoses2DInf : 2D edges as a Gaussian PDF with information matrix ( CPosePDFGaussianInf )
		  *      - mrpt::poses::CNetworkOfPoses3DInf : 3D edges as a Gaussian PDF with information matrix ( CPose3DPDFGaussianInf )
		  *      - mrpt::poses::CNetworkOfPoses2DCov : 2D edges as a Gaussian PDF with covariance matrix ( CPosePDFGaussian ). It's more efficient to use the information matrix version instead!
		  *      - mrpt::poses::CNetworkOfPoses3DCov : 3D edges as a Gaussian PDF with covariance matrix ( CPose3DPDFGaussian ). It's more efficient to use the information matrix version instead!
		  *
		  *  Two main members store all the information in this class:
		  *		- \a edge  (in the base class mrpt::math::CDirectedGraph::edge): A map from pairs of node ID -> pose constraints.
		  *		- \a nodes : A map from node ID -> estimated pose of that node (actually, read below on the template argument MAPS_IMPLEMENTATION).
		  *
		  *  Graphs can be loaded and saved to text file in the format used by TORO & HoG-man (more on the format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a> ),
		  *   using \a loadFromTextFile and \a saveToTextFile.
		  *
		  *  This class is the base for representing networks of poses, which are the main data type of a series
		  *   of SLAM algorithms implemented in the library mrpt-slam, in the namespace mrpt::graphslam.
		  *
		  *  For tools to visualize graphs as 2D/3D plots, see the namespace mrpt::opengl::graph_tools in the library mrpt-opengl.
		  *
		  *  The template arguments are:
		  *		- CPOSE: The type of the edges, which hold a relative pose (2D/3D, just a value or a Gaussian, etc.)
		  *		- MAPS_IMPLEMENTATION: Can be either mrpt::utils::map_traits_stdmap or mrpt::utils::map_traits_map_as_vector. Determines the type of the list of global poses (member \a nodes).
		  *
		  * \sa mrpt::graphslam
		  * \ingroup graph_utils_grp poses_grp poses_pdf_grp
		  */
		template<class CPOSE, class MAPS_IMPLEMENTATION = map_traits_stdmap >
		class CNetworkOfPoses : public mrpt::math::CDirectedGraph< CPOSE >
		{
		public:
			/** @name Typedef's
			    @{ */

			/** The base class "CDirectedGraph<CPOSE>" */
			typedef mrpt::math::CDirectedGraph< CPOSE > BASE;

			/** The type of PDF poses in the contraints (edges) (=CPOSE template argument) */
			typedef CPOSE                               constraint_t;

			/** The type of map's implementation (=MAPS_IMPLEMENTATION template argument) */
			typedef MAPS_IMPLEMENTATION                 maps_implementation_t;

			/** A map from pose IDs to their global coordinates estimates, with uncertainty */
			typedef typename MAPS_IMPLEMENTATION::template map<TNodeID,CPOSE>     global_poses_pdf_t;

			/** A map from pose IDs to their global coordinates estimates, without uncertainty (the "most-likely value") */
			typedef typename MAPS_IMPLEMENTATION::template map<TNodeID,typename CPOSE::type_value> global_poses_t;

			/** @} */


			/** @name Data members
			    @{ */

			/** The nodes (vertices) of the graph, with their estimated "global" (with respect to \a root) position, without an associated covariance.
			  * \sa dijkstra_nodes_estimate
			  */
			global_poses_t  nodes;

			/** The ID of the node that is the origin of coordinates, used as reference by all coordinates in \nodes. By default, root is the ID "0". */
			TNodeID         root;

			/** False (default) if an edge i->j stores the normal relative pose of j as seen from i: \f$ \Delta_i^j = j \ominus i \f$
			  * True if an edge i->j stores the inverse relateive pose, that is, i as seen from j: \f$ \Delta_i^j = i \ominus j \f$
			  */
			bool            edges_store_inverse_poses;

			/** @} */


			/** @name I/O file methods
			    @{ */

			/** Saves to a text file in the format used by TORO & HoG-man (more on the format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a> )
			  *  For 2D graphs only VERTEX2 & EDGE2 entries will be saved, and VERTEX3 & EDGE3 entries for 3D graphs.
			  *  Note that EQUIV entries will not be saved, but instead several EDGEs will be stored between the same node IDs.
			  * \sa saveToBinaryFile, loadFromTextFile
			  * \exception On any error
			  */
			inline void saveToTextFile( const std::string &fileName ) const {
				mrpt::poses::detail::save_graph_of_poses_from_text_file(this,fileName);
			}

			/** Loads from a text file in the format used by TORO & HoG-man (more on the format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a> )
			  *   Recognized line entries are: VERTEX2, VERTEX3, EDGE2, EDGE3, EQUIV.
			  *   If an unknown entry is found, a warning is dumped to std::cerr (only once for each unknown keyword).
			  *   An exception will be raised if trying to load a 3D graph into a 2D class (in the opposite case, missing 3D data will default to zero).
			  * \param fileName The file to load.
			  * \param collapse_dup_edges If true, \a collapseDuplicatedEdges will be called automatically after loading (note that this operation may take significant time for very large graphs).
			  * \sa loadFromBinaryFile, saveToTextFile
			  * \exception On any error, as a malformed line or loading a 3D graph in a 2D graph.
			  */
			inline void loadFromTextFile( const std::string &fileName, bool collapse_dup_edges = true ) {
				mrpt::poses::detail::load_graph_of_poses_from_text_file(this,fileName);
				if (collapse_dup_edges) this->collapseDuplicatedEdges();
			}

			/** @} */

			/** @name Utility methods
			    @{ */

			/** Spanning tree computation of a simple estimation of the global coordinates of each node just from the information in all edges, sorted in a Dijkstra tree based on the current "root" node.
			  *  Note that "global" coordinates are with respect to the node with the ID specified in \a root.
			  * \note This method takes into account the value of \a edges_store_inverse_poses
			  * \sa node, root
			  */
			inline void dijkstra_nodes_estimate() { detail::graph_of_poses_dijkstra_init(this); }

			/** Look for duplicated edges (even in opposite directions) between all pairs of nodes and fuse them.
			  *  Upon return, only one edge remains between each pair of nodes with the mean & covariance (or information matrix) corresponding to the Bayesian fusion of all the Gaussians.
			  * \return Overall number of removed edges.
			  */
			inline size_t collapseDuplicatedEdges() { return detail::graph_of_poses_collapse_dup_edges(this); }

			/** Computes the overall square error from all the pose constraints (edges) with respect to the global poses in \nodes
			  *  If \a ignoreCovariances is false, the squared Mahalanobis distance will be computed instead of the straight square error.
			  * \sa getEdgeSquareError
			  * \exception std::exception On global poses not in \a nodes
			  */
			double getGlobalSquareError(bool ignoreCovariances = true) const {
				double sqErr=0;
				const typename BASE::edges_map_t::const_iterator last_it=BASE::edges.end();
				for (typename BASE::edges_map_t::const_iterator itEdge=BASE::edges.begin();itEdge!=last_it;++itEdge)
					sqErr+=detail::graph_edge_sqerror(this,itEdge,ignoreCovariances);
				return sqErr;
			}

			/** Computes the square error of one pose constraints (edge) with respect to the global poses in \nodes
			  *  If \a ignoreCovariances is false, the squared Mahalanobis distance will be computed instead of the straight square error.
			  * \exception std::exception On global poses not in \a nodes
			  */
			inline double getEdgeSquareError(const typename BASE::edges_map_t::const_iterator &itEdge, bool ignoreCovariances = true) const { return detail::graph_edge_sqerror(this,itEdge,ignoreCovariances); }

			/** Computes the square error of one pose constraints (edge) with respect to the global poses in \nodes
			  *  If \a ignoreCovariances is false, the squared Mahalanobis distance will be computed instead of the straight square error.
			  * \exception std::exception On edge not existing or global poses not in \a nodes
			  */
			double getEdgeSquareError(const TNodeID from_id, const TNodeID to_id, bool ignoreCovariances = true ) const
			{
				const typename BASE::edges_map_t::const_iterator itEdge = BASE::edges.find( std::make_pair(from_id,to_id) );
				ASSERTMSG_(itEdge!=BASE::edges.end(),format("Request for edge %u->%u that doesn't exist in graph.",static_cast<unsigned int>(from_id),static_cast<unsigned int>(to_id)));
				return getEdgeSquareError(itEdge,ignoreCovariances);
			}

			/** Empty all edges, nodes and set root to ID 0. */
			inline void clear() {
				BASE::edges.clear();
				nodes.clear();
				root = 0;
				edges_store_inverse_poses = false;
			}

			/** Return number of nodes in the list \nodes of global coordinates (may be differente that all nodes appearing in edges)
			  * \sa mrpt::math::CDirectedGraph::countDifferentNodesInEdges
			  */
			inline size_t nodeCount() const { return nodes.size(); }

			/**  @} */

			/** @name Ctors & Dtors
			    @{ */

			/** Default constructor (just sets root to "0" and edges_store_inverse_poses to "false") */
			inline CNetworkOfPoses() : root(0), edges_store_inverse_poses(false) { }
			~CNetworkOfPoses() { }
			/** @} */
		};

#define DEFINE_SERIALIZABLE_GRAPH  \
		protected: \
			virtual void  writeToStream(CStream &out, int *version) const { \
				if (version) *version = 0; \
				else out << nodes << edges << root;  \
			} \
			virtual void readFromStream(CStream &in, int version) { \
				switch(version) \
				{ \
				case 0: { in >> nodes >> edges >> root; } break; \
				default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version) \
				}; \
			}



		/** \addtogroup graph_utils_grp
		    @{ */

		// Define serializable versions of the template above for each specific kind of "edge":

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses2D )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses3D )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses2DCov )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses3DCov )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses2DInf )
		DEFINE_SERIALIZABLE_PRE( CNetworkOfPoses3DInf )

		/** The specialization of CNetworkOfPoses for poses of type CPose2D (not a PDF!), also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses2D : public CNetworkOfPoses<CPose2D,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses2D )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** The specialization of CNetworkOfPoses for poses of type CPose3D (not a PDF!), also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses3D : public CNetworkOfPoses<CPose3D,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses3D )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussian, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses2DCov : public CNetworkOfPoses<CPosePDFGaussian,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses2DCov )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussian, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses3DCov : public CNetworkOfPoses<CPose3DPDFGaussian,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses3DCov )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussianInf, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses2DInf : public CNetworkOfPoses<CPosePDFGaussianInf,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses2DInf )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussianInf, also implementing serialization.
		  * \sa CNetworkOfPoses, CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf
		  */
		class BASE_IMPEXP CNetworkOfPoses3DInf : public CNetworkOfPoses<CPose3DPDFGaussianInf,map_traits_stdmap>, public mrpt::utils::CSerializable
		{
			DEFINE_MRPT_OBJECT( CNetworkOfPoses3DInf )	// Should be DEFINE_SERIALIZABLE but the next macro defines what that macro only declared.
			DEFINE_SERIALIZABLE_GRAPH
		};

		/** @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
