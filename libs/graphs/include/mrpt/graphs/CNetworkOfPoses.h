/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

/** \file The main class in this file is mrpt::poses::CNetworkOfPoses<>, a
   generic
		   basic template for predefined 2D/3D graphs of pose contraints.
*/

#include <iostream>

#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/TParameters.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/math/utils.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/graphs/dijkstra.h>
#include <mrpt/graphs/TNodeAnnotations.h>
#include <mrpt/graphs/TMRSlamNodeAnnotations.h>
#include <mrpt/graphs/THypothesis.h>

#include <iterator>
#include <algorithm>
#include <type_traits>  // is_base_of()
#include <memory>

namespace mrpt
{
namespace graphs
{
/** Internal functions for MRPT */
namespace detail
{
template <class GRAPH_T>
struct graph_ops;

// forward declaration of CVisualizer
template <
	class CPOSE,  // Type of edges
	class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
class CVisualizer;
// forward declaration of CMRVisualizer
template <
	class CPOSE,  // Type of edges
	class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
class CMRVisualizer;
}  // namespace detail

/** A directed graph of pose constraints, with edges being the relative poses
 *between pairs of nodes identified by their numeric IDs (of type
 *mrpt::graphs::TNodeID).
 *  A link or edge between two nodes "i" and "j", that is, the pose \f$ p_{ij}
 *\f$, holds the relative position of "j" with respect to "i".
 *   These poses are stored in the edges in the format specified by the template
 *argument CPOSE. Users should employ the following derived classes
 *   depending on the desired representation of edges:
 *      - mrpt::graphs::CNetworkOfPoses2D    : 2D edges as a simple CPose2D (x y
 *phi)
 *      - mrpt::graphs::CNetworkOfPoses3D    : 3D edges as a simple
 *mrpt::poses::CPose3D (x y z yaw pitch roll)
 *      - mrpt::graphs::CNetworkOfPoses2DInf : 2D edges as a Gaussian PDF with
 *information matrix (CPosePDFGaussianInf)
 *      - mrpt::graphs::CNetworkOfPoses3DInf : 3D edges as a Gaussian PDF with
 *information matrix (CPose3DPDFGaussianInf)
 *      - mrpt::graphs::CNetworkOfPoses2DCov : 2D edges as a Gaussian PDF with
 *covariance matrix (CPosePDFGaussian). It's more efficient to use the
 *information matrix version instead!
 *      - mrpt::graphs::CNetworkOfPoses3DCov : 3D edges as a Gaussian PDF with
 *covariance matrix (CPose3DPDFGaussian). It's more efficient to use the
 *information matrix version instead!
 *
 *  Two main members store all the information in this class:
 *		- \a edges  (in the base class mrpt::graphs::CDirectedGraph::edges): A
 *map
 *from pairs of node ID -> pose constraints.
 *		- \a nodes : A map from node ID -> estimated pose of that node
 *(actually,
 *read below on the template argument MAPS_IMPLEMENTATION).
 *
 *  Graphs can be loaded and saved to text file in the format used by TORO &
 *HoG-man (more on the format <a
 *href="http://www.mrpt.org/Robotics_file_formats" >here</a>),
 *   using \a loadFromTextFile and \a saveToTextFile.
 *
 *  This class is the base for representing networks of poses, which are the
 *main data type of a series
 *   of SLAM algorithms implemented in the library mrpt-slam, in the namespace
 *mrpt::graphslam.
 *
 *  The template arguments are:
 *		- CPOSE: The type of the edges, which hold a relative pose (2D/3D, just
 *a
 *value or a Gaussian, etc.)
 *		- MAPS_IMPLEMENTATION: Can be either mrpt::containers::map_traits_stdmap
 *or mrpt::containers::map_traits_map_as_vector. Determines the type of the list
 *of global poses (member \a nodes).
 *
 * \sa mrpt::graphslam
 * \ingroup mrpt_graphs_grp
 */
template <
	class CPOSE,  // Type of edges
	class MAPS_IMPLEMENTATION =
		mrpt::containers::map_traits_stdmap,  // Use std::map<> vs.
	// std::vector<>
	class NODE_ANNOTATIONS = mrpt::graphs::detail::TNodeAnnotationsEmpty,
	class EDGE_ANNOTATIONS = mrpt::graphs::detail::edge_annotations_empty>
class CNetworkOfPoses
	: public mrpt::graphs::CDirectedGraph<CPOSE, EDGE_ANNOTATIONS>
{
   public:
	/** @name Typedef's
		@{ */
	/** The base class "CDirectedGraph<CPOSE,EDGE_ANNOTATIONS>" */
	using BASE = mrpt::graphs::CDirectedGraph<CPOSE, EDGE_ANNOTATIONS>;
	/** My own type */
	using self_t = CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;

	/** The type of PDF poses in the contraints (edges) (=CPOSE template
	 * argument) */
	using constraint_t = CPOSE;
	/** The extra annotations in nodes, apart from a \a constraint_no_pdf_t */
	using node_annotations_t = NODE_ANNOTATIONS;
	/** The extra annotations in edges, apart from a \a constraint_t */
	using edge_annotations_t = EDGE_ANNOTATIONS;

	/** The type of map's implementation (=MAPS_IMPLEMENTATION template
	 * argument) */
	using maps_implementation_t = MAPS_IMPLEMENTATION;
	/** The type of edges or their means if they are PDFs (that is, a simple
	 * "edge" value) */
	using constraint_no_pdf_t = typename CPOSE::type_value;

	constexpr static auto getClassName()
	{
		using namespace mrpt::typemeta;
		return literal("mrpt::graphs::CNetworkOfPoses<") +
			   TTypeName<CPOSE>::get() + literal(",") +
			   TTypeName<MAPS_IMPLEMENTATION>::get() + literal(",") +
			   TTypeName<NODE_ANNOTATIONS>::get() + literal(",") +
			   TTypeName<EDGE_ANNOTATIONS>::get() + literal(">");
	}

	/** The type of each global pose in \a nodes: an extension of the \a
	 * constraint_no_pdf_t pose with any optional user-defined data
	 */
	struct global_pose_t : public constraint_no_pdf_t, public NODE_ANNOTATIONS
	{
		using self_t = typename CNetworkOfPoses<
			CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS,
			EDGE_ANNOTATIONS>::global_pose_t;

		constexpr static auto getClassName()
		{
			using namespace mrpt::typemeta;
			return literal("global_pose_t<") +
				   TTypeName<constraint_no_pdf_t>::get() + literal(",") +
				   TTypeName<NODE_ANNOTATIONS>::get() + literal(">");
		}

		/**\brief Potential class constructors
		 */
		/**\{ */
		inline global_pose_t() : constraint_no_pdf_t() {}
		template <typename ARG1>
		inline global_pose_t(const ARG1& a1) : constraint_no_pdf_t(a1)
		{
		}
		template <typename ARG1, typename ARG2>
		inline global_pose_t(const ARG1& a1, const ARG2& a2)
			: constraint_no_pdf_t(a1, a2)
		{
		}
		/**\} */

		/**\brief Copy constructor - delegate copying to the NODE_ANNOTATIONS
		 * struct
		 */
		inline global_pose_t(const global_pose_t& other)
			: constraint_no_pdf_t(other), NODE_ANNOTATIONS(other)
		{
		}

		inline bool operator==(const global_pose_t& other) const
		{
			return (
				static_cast<const constraint_no_pdf_t>(*this) ==
					static_cast<const constraint_no_pdf_t>(other) &&
				static_cast<const NODE_ANNOTATIONS>(*this) ==
					static_cast<const NODE_ANNOTATIONS>(other));
		}
		inline bool operator!=(const global_pose_t& other) const
		{
			return (!(*this == other));
		}

		inline friend std::ostream& operator<<(
			std::ostream& o, const self_t& global_pose)
		{
			o << global_pose.asString() << "| "
			  << global_pose.retAnnotsAsString();
			return o;
		}
	};

	/** A map from pose IDs to their global coordinate estimates, with
	 * uncertainty */
	using global_poses_pdf_t = typename MAPS_IMPLEMENTATION::template map<
		mrpt::graphs::TNodeID, CPOSE>;

	/** A map from pose IDs to their global coordinate estimates, without
	 * uncertainty (the "most-likely value") */
	using global_poses_t = typename MAPS_IMPLEMENTATION::template map<
		mrpt::graphs::TNodeID, global_pose_t>;

	/** @} */

	/** @name Data members
		@{ */

	/** The nodes (vertices) of the graph, with their estimated "global"
	 * (with respect to \a root) position, without an associated covariance.
	 * \sa dijkstra_nodes_estimate
	 */
	global_poses_t nodes;

	/** The ID of the node that is the origin of coordinates, used as
	 * reference by all coordinates in \a nodes. By default, root is the ID
	 * "0". */
	mrpt::graphs::TNodeID root{0};

	/** False (default) if an edge i->j stores the normal relative pose of j
	 * as seen from i: \f$ \Delta_i^j = j \ominus i \f$ True if an edge i->j
	 * stores the inverse relateive pose, that is, i as seen from j: \f$
	 * \Delta_i^j = i \ominus j \f$
	 */
	bool edges_store_inverse_poses{false};

	/** @} */

	/** @name I/O methods
		@{ */

	/** Saves to a text file in the format used by TORO, HoG-man, G2O.
	 * See: https://www.mrpt.org/Graph-SLAM_maps
	 * \sa saveToBinaryFile, loadFromTextFile, writeAsText
	 * \exception On any error
	 */
	void saveToTextFile(const std::string& fileName) const
	{
		detail::graph_ops<self_t>::save_graph_of_poses_to_text_file(
			this, fileName);
	}
	/** Writes as text in the format used by TORO, HoG-man, G2O.
	 * See: https://www.mrpt.org/Graph-SLAM_maps
	 * \sa saveToBinaryFile, loadFromTextFile, saveToTextFile, readAsText
	 * \exception On any error
	 */
	void writeAsText(std::ostream& o) const
	{
		detail::graph_ops<self_t>::save_graph_of_poses_to_ostream(this, o);
	}

	/** Loads from a text file in the format used by TORO & HoG-man (more on the
	 * format <a href="http://www.mrpt.org/Robotics_file_formats" >here</a>)
	 *   Recognized line entries are: VERTEX2, VERTEX3, EDGE2, EDGE3, EQUIV.
	 *   If an unknown entry is found, a warning is dumped to std::cerr (only
	 * once for each unknown keyword).
	 *   An exception will be raised if trying to load a 3D graph into a 2D
	 * class (in the opposite case, missing 3D data will default to zero).
	 *
	 * \param[in] fileName The file to load.
	 * \param[in] collapse_dup_edges If true, \a collapseDuplicatedEdges will be
	 * called automatically after loading (note that this operation may take
	 * significant time for very large graphs).
	 *
	 * \sa loadFromBinaryFile, saveToTextFile
	 *
	 * \exception On any error, as a malformed line or loading a 3D graph in
	 * a 2D graph.
	 */
	inline void loadFromTextFile(
		const std::string& fileName, bool collapse_dup_edges = true)
	{
		detail::graph_ops<self_t>::load_graph_of_poses_from_text_file(
			this, fileName);
		if (collapse_dup_edges) this->collapseDuplicatedEdges();
	}

	/** Reads as text in the format used by TORO, HoG-man, G2O.
	 * See: https://www.mrpt.org/Graph-SLAM_maps
	 * \sa saveToBinaryFile, loadFromTextFile, saveToTextFile
	 * \exception On any error
	 */
	void readAsText(std::istream& i)
	{
		detail::graph_ops<self_t>::load_graph_of_poses_from_text_stream(
			this, i);
	}

	/** @} */

	/** @name Utility methods
		@{ */
	/**\brief Return 3D Visual Representation of the edges and nodes in the
	 * network of poses
	 *
	 * Method makes the call to the corresponding method of the CVisualizer
	 * class instance.
	 */
	inline void getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr object,
		const mrpt::system::TParametersDouble& viz_params) const
	{
		using visualizer_t = mrpt::graphs::detail::CVisualizer<
			CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;
		using visualizer_multirobot_t = mrpt::graphs::detail::CMRVisualizer<
			CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;

		bool is_multirobot = false;
		std::unique_ptr<visualizer_t> viz;
		is_multirobot =
			(std::is_base_of_v<
				mrpt::graphs::detail::TMRSlamNodeAnnotations, global_pose_t>);
		if (is_multirobot)
		{
			viz.reset(new visualizer_multirobot_t(*this));
		}
		else
		{
			viz.reset(new visualizer_t(*this));
		}
		viz->getAs3DObject(object, viz_params);
	}

	/** Spanning tree computation of a simple estimation of the global
	 * coordinates of each node just from the information in all edges,
	 * sorted in a Dijkstra tree based on the current "root" node.
	 *
	 * \note The "global" coordinates are with respect to the node with the
	 * ID specified in \a root.
	 *
	 * \note This method takes into account the
	 * value of \a edges_store_inverse_poses
	 *
	 * \sa node, root
	 */
	inline void dijkstra_nodes_estimate()
	{
		detail::graph_ops<self_t>::graph_of_poses_dijkstra_init(this);
	}

	/** Look for duplicated edges (even in opposite directions) between all
	 * pairs of nodes and fuse them.  Upon return, only one edge remains
	 * between each pair of nodes with the mean & covariance (or information
	 * matrix) corresponding to the Bayesian fusion of all the Gaussians.
	 *
	 * \return Overall number of removed edges.
	 */
	inline size_t collapseDuplicatedEdges()
	{
		return detail::graph_ops<self_t>::graph_of_poses_collapse_dup_edges(
			this);
	}

	/** Returns the total chi-squared error of the graph. Shortcut for
	 * getGlobalSquareError(false). */
	double chi2() const { return getGlobalSquareError(false); }
	/** Evaluates the graph total square error (ignoreCovariances=true) or
	 * chi2 (ignoreCovariances=false) from all the pose constraints (edges)
	 * with respect to the global poses in \a nodes.
	 * \sa getEdgeSquareError
	 * \exception std::exception On global poses not in \a nodes
	 */
	double getGlobalSquareError(bool ignoreCovariances = true) const
	{
		double sqErr = 0;
		for (auto it = BASE::edges.begin(); it != BASE::edges.end(); ++it)
			sqErr += detail::graph_ops<self_t>::graph_edge_sqerror(
				this, it, ignoreCovariances);
		return sqErr;
	}

	/**\brief Find the edges between the nodes in the node_IDs set and fill
	 * given graph pointer accordingly.
	 *
	 * \param[in] node_IDs Set of nodes, between which, edges should be found
	 * and inserted in the given sub_graph pointer
	 * \param[in] root_node_in Node ID to be used as the root node of
	 * sub_graph. If this is not given, the lowest nodeID is to be used.
	 * \param[out] CNetworkOfPoses pointer that is to be filled.
	 *
	 * \param[in] auto_expand_set If true and in case the node_IDs set
	 * contains non-consecutive nodes the returned set is expanded with the
	 * in-between nodes. This makes sure that the final graph is always
	 * connected.
	 * If auto_expand_set is false  but there exist
	 * non-consecutive nodes, virtual edges are inserted in the parts that
	 * the graph is not connected
	 */
	void extractSubGraph(
		const std::set<TNodeID>& node_IDs, self_t* sub_graph,
		const TNodeID root_node_in = INVALID_NODEID,
		const bool& auto_expand_set = true) const
	{
		using namespace std;
		using namespace mrpt;
		using namespace mrpt::math;
		using namespace mrpt::graphs::detail;

		using dijkstra_t = CDijkstra<self_t, MAPS_IMPLEMENTATION>;

		// assert that the given pointers are valid
		ASSERTMSG_(
			sub_graph,
			"\nInvalid pointer to a CNetworkOfPoses instance is given. "
			"Exiting..\n");
		sub_graph->clear();

		// assert that the root_node actually exists in the given node_IDs set
		TNodeID root_node = root_node_in;
		if (root_node != INVALID_NODEID)
		{
			ASSERTMSG_(
				node_IDs.find(root_node) != node_IDs.end(),
				"\nRoot_node does not exist in the given node_IDs set. "
				"Exiting.\n");
		}

		// ask for at least 2 nodes
		ASSERTMSG_(
			node_IDs.size() >= 2,
			format(
				"Very few nodes [%lu] for which to extract a subgraph. "
				"Exiting\n",
				static_cast<unsigned long>(node_IDs.size())));

		// find out if querry contains non-consecutive nodes.
		// Assumption: Set elements are in standard, ascending order.
		bool is_fully_connected_graph = true;
		std::set<TNodeID> node_IDs_real;  // actual set of nodes to be used.
		if (*node_IDs.rbegin() - *node_IDs.begin() + 1 == node_IDs.size())
		{
			node_IDs_real = node_IDs;
		}
		else
		{  // contains non-consecutive nodes
			is_fully_connected_graph = false;

			if (auto_expand_set)
			{  // set auto-expansion
				for (TNodeID curr_node_ID = *node_IDs.begin();
					 curr_node_ID != *node_IDs.rbegin(); ++curr_node_ID)
				{
					node_IDs_real.insert(curr_node_ID);
				}
			}
			else
			{  // virtual_edge_addition strategy
				node_IDs_real = node_IDs;
			}
		}

		// add all the nodes of the node_IDs_real set to sub_graph
		for (unsigned long node_IDs_it : node_IDs_real)
		{
			// assert that current node exists in *own* graph
			typename global_poses_t::const_iterator own_it;
			for (own_it = nodes.begin(); own_it != nodes.end(); ++own_it)
			{
				if (node_IDs_it == own_it->first)
				{
					break;  // I throw exception afterwards
				}
			}
			ASSERTMSG_(
				own_it != nodes.end(),
				format(
					"NodeID [%lu] can't be found in the initial graph.",
					static_cast<unsigned long>(node_IDs_it)));

			global_pose_t curr_node(nodes.at(node_IDs_it));
			sub_graph->nodes.insert(make_pair(node_IDs_it, curr_node));
		}
		// cout << "Extracting subgraph for nodeIDs: " <<
		// getSTLContainerAsString(node_IDs_real) << endl;

		// set the root of the extracted graph
		if (root_node == INVALID_NODEID)
		{
			// smallest nodeID by default: already ordered inside std::map
			root_node = sub_graph->nodes.begin()->first;
		}
		sub_graph->root = root_node;

		// find all edges (in the initial graph), that exist in the given set
		// of nodes; add them to the given graph
		sub_graph->clearEdges();
		for (const auto& e : BASE::edges)
		{
			const TNodeID& from = e.first.first;
			const TNodeID& to = e.first.second;
			const typename BASE::edge_t& curr_edge = e.second;

			// if both nodes exist in the given set, add the corresponding edge
			if (sub_graph->nodes.find(from) != sub_graph->nodes.end() &&
				sub_graph->nodes.find(to) != sub_graph->nodes.end())
			{
				sub_graph->insertEdge(from, to, curr_edge);
			}
		}

		if (!auto_expand_set && !is_fully_connected_graph)
		{
			// Addition of virtual edges between non-connected graph parts is
			// necessary

			// make sure that the root nodeID is connected to at least one node
			{
				std::set<TNodeID> root_neighbors;
				sub_graph->getNeighborsOf(sub_graph->root, root_neighbors);

				if (root_neighbors.empty())
				{
					// add an edge between the root and an adjacent nodeID
					typename global_poses_t::iterator root_it =
						sub_graph->nodes.find(sub_graph->root);
					ASSERT_(root_it != sub_graph->nodes.end());
					if ((*root_it == *sub_graph->nodes.rbegin()))
					{  // is the last nodeID
						// add with previous node
						TNodeID next_to_root = (--root_it)->first;
						self_t::addVirtualEdge(
							sub_graph, next_to_root, sub_graph->root);
						// cout << "next_to_root = " << next_to_root;
					}
					else
					{
						TNodeID next_to_root = (++root_it)->first;
						// cout << "next_to_root = " << next_to_root;
						self_t::addVirtualEdge(
							sub_graph, sub_graph->root, next_to_root);
					}
				}
			}

			// as long as the graph is unconnected (as indicated by Dijkstra)
			// add a virtual edge between
			bool dijkstra_runs_successfully = false;

			// loop until the graph is fully connected (i.e. I can reach every
			// node of the graph starting from its root)
			while (!dijkstra_runs_successfully)
			{
				try
				{
					dijkstra_t dijkstra(*sub_graph, sub_graph->root);
					dijkstra_runs_successfully = true;
				}
				catch (const mrpt::graphs::detail::NotConnectedGraph& ex)
				{
					dijkstra_runs_successfully = false;

					set<TNodeID> unconnected_nodeIDs;
					ex.getUnconnectedNodeIDs(&unconnected_nodeIDs);
					// cout << "Unconnected nodeIDs: " <<
					// mrpt::math::getSTLContainerAsString(unconnected_nodeIDs)
					// << endl;
					// mainland: set of nodes that the root nodeID is in
					// island: set of nodes that the Dijkstra graph traversal
					// can't
					// reach starting from the root.
					// [!] There may be multiple sets of these nodes

					// set::rend() is the element with the highest value
					// set::begin() is the element with the lowest value
					const TNodeID& island_highest =
						*unconnected_nodeIDs.rbegin();
					const TNodeID& island_lowest = *unconnected_nodeIDs.begin();
					// cout << "island_highest: " << island_highest << endl;
					// cout << "island_lowest: " << island_lowest << endl;
					// cout << "root: " << sub_graph->root << endl;

					// find out which nodes are in the same partition with the
					// root
					// (i.e. mainland)
					std::set<TNodeID> mainland;
					// for all nodes in sub_graph
					for (typename global_poses_t::const_iterator n_it =
							 sub_graph->nodes.begin();
						 n_it != sub_graph->nodes.end(); ++n_it)
					{
						bool is_there = false;

						// for all unconnected nodes
						for (unsigned long unconnected_nodeID :
							 unconnected_nodeIDs)
						{
							if (n_it->first == unconnected_nodeID)
							{
								is_there = true;
								break;
							}
						}

						if (!is_there)
						{
							mainland.insert(n_it->first);
						}
					}

					bool is_single_island =
						(island_highest - island_lowest + 1 ==
						 unconnected_nodeIDs.size());

					if (is_single_island)
					{  // single island
						// Possible scenarios:
						// | island                       |
						// | mainland                                   |
						// | <low nodeIDs>  island_highest|  --- <virtual_edge>
						// --->>  | mainland_lowest <high nodeIDs> ... root ...|
						// --- OR ---
						// | mainland                       |
						// | island                       |
						// | <low nodeIDs>  mainland_highest|  ---
						// <virtual_edge> --->>  | island_lowest <high nodeIDs>
						// |

						const std::set<TNodeID>& island = unconnected_nodeIDs;
						this->connectGraphPartitions(
							sub_graph, island, mainland);
					}
					else
					{  // multiple islands
						// add a virtual edge between the last group before the
						// mainland and the mainland

						// split the unconnected_nodeIDs to smaller groups of
						// nodes
						// we only care about the nodes that are prior to the
						// root
						std::vector<std::set<TNodeID>> vec_of_islands;
						std::set<TNodeID> curr_island;
						TNodeID prev_nodeID = *unconnected_nodeIDs.begin();
						curr_island.insert(
							prev_nodeID);  // add the initial node;
						for (auto it = ++unconnected_nodeIDs.begin();
							 *it < sub_graph->root &&
							 it != unconnected_nodeIDs.end();
							 ++it)
						{
							if (!(absDiff(*it, prev_nodeID) == 1))
							{
								vec_of_islands.push_back(curr_island);
								curr_island.clear();
							}
							curr_island.insert(*it);

							// update the previous nodeID
							prev_nodeID = *it;
						}
						vec_of_islands.push_back(curr_island);

						// cout << "last_island: " <<
						// getSTLContainerAsString(vec_of_islands.back()) <<
						// endl;
						// cout << "mainland: " <<
						// getSTLContainerAsString(mainland) << endl;
						this->connectGraphPartitions(
							sub_graph, vec_of_islands.back(), mainland);
					}
				}
			}
		}

		// estimate the node positions according to the edges - root is (0, 0,
		// 0)
		// just execute dijkstra once for grabbing the updated node positions.
		sub_graph->dijkstra_nodes_estimate();

	}  // end of extractSubGraph

	/**\brief Integrate given graph into own graph using the list of provided
	 * common THypotheses. Nodes of the other graph are renumbered upon
	 * integration in own graph.
	 *
	 * \param[in] other Graph (of the same type) that is to be integrated with
	 * own graph.
	 * \param[in] Hypotheses that join own and other graph.
	 * \param[in] hypots_from_other_to_self Specify the direction of the
	 * THypothesis objects in the common_hypots. If true (default) they are
	 * directed from other to own graph (other \rightarrow own),
	 *
	 * \param[out] old_to_new_nodeID_mappings_out Map from the old nodeIDs
	 * that are in the given graph to the new nodeIDs that have been inserted
	 * (by this method) in own graph.
	 */
	inline void mergeGraph(
		const self_t& other,
		const typename std::vector<detail::THypothesis<self_t>>& common_hypots,
		const bool hypots_from_other_to_self = true,
		std::map<TNodeID, TNodeID>* old_to_new_nodeID_mappings_out = nullptr)
	{
		MRPT_START;
		using namespace mrpt::graphs;
		using namespace mrpt::graphs::detail;
		using namespace std;

		using hypots_cit_t =
			typename vector<THypothesis<self_t>>::const_iterator;
		using nodes_cit_t = typename global_poses_t::const_iterator;

		const self_t& graph_from = (hypots_from_other_to_self ? other : *this);
		const self_t& graph_to = (hypots_from_other_to_self ? *this : other);

		// assert that both own and other graph have at least two nodes.
		ASSERT_(graph_from.nodes.size() >= 2);
		ASSERT_(graph_to.nodes.size() >= 2);

		// Assert that from-nodeIds, to-nodeIDs in common_hypots exist in own
		// and other graph respectively
		for (hypots_cit_t h_cit = common_hypots.begin();
			 h_cit != common_hypots.end(); ++h_cit)
		{
			ASSERTMSG_(
				graph_from.nodes.find(h_cit->from) != graph_from.nodes.end(),
				format("NodeID %lu is not found in (from) graph", h_cit->from));
			ASSERTMSG_(
				graph_to.nodes.find(h_cit->to) != graph_to.nodes.end(),
				format("NodeID %lu is not found in (to) graph", h_cit->to));
		}

		// find the max nodeID in existing graph
		TNodeID max_nodeID = 0;
		for (nodes_cit_t n_cit = this->nodes.begin();
			 n_cit != this->nodes.end(); ++n_cit)
		{
			if (n_cit->first > max_nodeID)
			{
				max_nodeID = n_cit->first;
			}
		}
		TNodeID renum_start = max_nodeID + 1;
		size_t renum_counter = 0;
		// cout << "renum_start: " << renum_start << endl;

		// Renumber nodeIDs of other graph so that they don't overlap with own
		// graph nodeIDs
		std::map<TNodeID, TNodeID>* old_to_new_nodeID_mappings;

		// map of TNodeID->TNodeID correspondences to address to if the
		// old_to_new_nodeID_mappings_out is not given.
		// Handy for not having to allocate old_to_new_nodeID_mappings in the
		// heap
		std::map<TNodeID, TNodeID> mappings_tmp;

		// If given, use the old_to_new_nodeID_mappings map.
		if (old_to_new_nodeID_mappings_out)
		{
			old_to_new_nodeID_mappings = old_to_new_nodeID_mappings_out;
		}
		else
		{
			old_to_new_nodeID_mappings = &mappings_tmp;
		}
		old_to_new_nodeID_mappings->clear();

		// add all nodes of other graph - Take care of renumbering them
		// cout << "Adding nodes of other graph" << endl;
		// cout << "====================" << endl;
		for (nodes_cit_t n_cit = other.nodes.begin();
			 n_cit != other.nodes.end(); ++n_cit)
		{
			TNodeID new_nodeID = renum_start + renum_counter++;
			old_to_new_nodeID_mappings->insert(
				make_pair(n_cit->first, new_nodeID));
			this->nodes.insert(make_pair(new_nodeID, n_cit->second));

			// cout << "Adding nodeID: " << new_nodeID << endl;
		}

		// add common constraints
		// cout << "Adding common constraints" << endl;
		// cout << "====================" << endl;
		for (hypots_cit_t h_cit = common_hypots.begin();
			 h_cit != common_hypots.end(); ++h_cit)
		{
			TNodeID from, to;
			if (hypots_from_other_to_self)
			{
				from = old_to_new_nodeID_mappings->at(h_cit->from);
				to = h_cit->to;
			}
			else
			{
				from = h_cit->from;
				to = old_to_new_nodeID_mappings->at(h_cit->to);
			}
			this->insertEdge(from, to, h_cit->getEdge());
			// cout << from << " -> " << to << " => " << h_cit->getEdge() <<
			// endl;
		}

		// add all constraints of the other graph
		// cout << "Adding constraints of other graph" << endl;
		// cout << "====================" << endl;
		for (typename self_t::const_iterator g_cit = other.begin();
			 g_cit != other.end(); ++g_cit)
		{
			TNodeID new_from =
				old_to_new_nodeID_mappings->at(g_cit->first.first);
			TNodeID new_to =
				old_to_new_nodeID_mappings->at(g_cit->first.second);
			this->insertEdge(new_from, new_to, g_cit->second);

			// cout << "[" << new_from << "] -> [" << new_to << "]" << " => " <<
			// g_cit->second << endl;
		}

		// run Dijkstra to update the node positions
		this->dijkstra_nodes_estimate();

		MRPT_END;
	}

	/**\brief Add an edge between the last node of the group with the lower
	 * nodeIDs and the first node of the higher nodeIDs.
	 *
	 * Given groups of nodes should only contain consecutive nodeIDs and
	 * there should be no overlapping between them
	 *
	 * \note It is assumed that the sets of nodes are \b already in ascending
	 * order (default std::set behavior.
	 */
	inline static void connectGraphPartitions(
		self_t* sub_graph, const std::set<TNodeID>& groupA,
		const std::set<TNodeID>& groupB)
	{
		using namespace mrpt::math;

		ASSERTMSG_(
			sub_graph,
			"\nInvalid pointer to a CNetworkOfPoses instance is given. "
			"Exiting..\n");
		ASSERTMSG_(!groupA.empty(), "\ngroupA is empty.");
		ASSERTMSG_(!groupB.empty(), "\ngroupB is empty.");

		// assertion - non-overlapping groups
		ASSERTMSG_(
			*groupA.rend() < *groupB.rbegin() ||
				*groupA.rbegin() > *groupB.rend(),
			"Groups A, B contain overlapping nodeIDs");

		// decide what group contains the low/high nodeIDs
		// just compare any two nodes of the sets (they are non-overlapping
		const std::set<TNodeID>& low_nodeIDs =
			*groupA.rbegin() < *groupB.rbegin() ? groupA : groupB;
		const std::set<TNodeID>& high_nodeIDs =
			*groupA.rbegin() > *groupB.rbegin() ? groupA : groupB;

		// add virtual edge
		const TNodeID& from_nodeID = *low_nodeIDs.rbegin();
		const TNodeID& to_nodeID = *high_nodeIDs.begin();
		self_t::addVirtualEdge(sub_graph, from_nodeID, to_nodeID);
	}

	/** Computes the square error of one pose constraints (edge) with respect
	 * to the global poses in \a nodes If \a ignoreCovariances is false, the
	 * squared Mahalanobis distance will be computed instead of the straight
	 * square error.
	 *
	 * \exception std::exception On global poses not in \a nodes
	 */
	inline double getEdgeSquareError(
		const typename BASE::edges_map_t::const_iterator& itEdge,
		bool ignoreCovariances = true) const
	{
		return detail::graph_ops<self_t>::graph_edge_sqerror(
			this, itEdge, ignoreCovariances);
	}

	/** Computes the square error of one pose constraints (edge) with respect
	 * to the global poses in \a nodes If \a ignoreCovariances is false, the
	 * squared Mahalanobis distance will be computed instead of the straight
	 * square error.
	 *
	 * \exception std::exception On edge not existing or global poses not in
	 * \a nodes
	 */
	double getEdgeSquareError(
		const mrpt::graphs::TNodeID from_id, const mrpt::graphs::TNodeID to_id,
		bool ignoreCovariances = true) const
	{
		const typename BASE::edges_map_t::const_iterator itEdge =
			BASE::edges.find(std::make_pair(from_id, to_id));
		ASSERTMSG_(
			itEdge != BASE::edges.end(),
			format(
				"Request for edge %u->%u that doesn't exist in graph.",
				static_cast<unsigned int>(from_id),
				static_cast<unsigned int>(to_id)));
		return getEdgeSquareError(itEdge, ignoreCovariances);
	}

	/** Empty all edges, nodes and set root to ID 0. */
	inline void clear()
	{
		BASE::edges.clear();
		nodes.clear();
		root = 0;
		edges_store_inverse_poses = false;
	}

	/** Return number of nodes in the list \a nodes of global coordinates
	 * (may be different that all nodes appearing in edges)
	 *
	 * \sa mrpt::graphs::CDirectedGraph::countDifferentNodesInEdges
	 */
	inline size_t nodeCount() const { return nodes.size(); }
	/**  @} */

   private:
	/**\brief Add a virtual edge between two nodes in the given graph.
	 *
	 * Edge is called virtual as its value will be determined solely on the
	 * pose difference of the given nodeIDs
	 */
	inline static void addVirtualEdge(
		self_t* graph, const TNodeID& from, const TNodeID& to)
	{
		ASSERTMSG_(
			graph, "Invalid pointer to the graph instance was provided.");

		typename self_t::global_pose_t& p_from = graph->nodes.at(from);
		typename self_t::global_pose_t& p_to = graph->nodes.at(to);
		const typename BASE::edge_t& virt_edge(p_to - p_from);

		graph->insertEdge(from, to, virt_edge);
	}
};

/** Binary serialization (write) operator "stream << graph" */
template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out,
	const CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>& obj)
{
	using graph_t = CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;
	detail::graph_ops<graph_t>::save_graph_of_poses_to_binary_file(&obj, out);
	return out;
}

/** Binary serialization (read) operator "stream >> graph" */
template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in,
	CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>& obj)
{
	using graph_t = CNetworkOfPoses<
		CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;
	detail::graph_ops<graph_t>::read_graph_of_poses_from_binary_file(&obj, in);
	return in;
}

/** \addtogroup mrpt_graphs_grp
 * \name Handy typedefs for CNetworkOfPoses commonly used types
 @{ */

/** The specialization of CNetworkOfPoses for poses of type CPose2D (not a
 * PDF!), also implementing serialization. */
using CNetworkOfPoses2D = CNetworkOfPoses<mrpt::poses::CPose2D>;
/** The specialization of CNetworkOfPoses for poses of type mrpt::poses::CPose3D
 * (not a PDF!), also implementing serialization. */
using CNetworkOfPoses3D = CNetworkOfPoses<mrpt::poses::CPose3D>;
/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussian,
 * also implementing serialization. */
using CNetworkOfPoses2DCov = CNetworkOfPoses<mrpt::poses::CPosePDFGaussian>;
/** The specialization of CNetworkOfPoses for poses of type CPose3DPDFGaussian,
 * also implementing serialization. */
using CNetworkOfPoses3DCov = CNetworkOfPoses<mrpt::poses::CPose3DPDFGaussian>;
/** The specialization of CNetworkOfPoses for poses of type CPosePDFGaussianInf,
 * also implementing serialization. */
using CNetworkOfPoses2DInf = CNetworkOfPoses<mrpt::poses::CPosePDFGaussianInf>;
/** The specialization of CNetworkOfPoses for poses of type
 * CPose3DPDFGaussianInf, also implementing serialization. */
using CNetworkOfPoses3DInf =
	CNetworkOfPoses<mrpt::poses::CPose3DPDFGaussianInf>;

/**\brief Specializations of CNetworkOfPoses for graphs whose nodes inherit from
 * TMRSlamNodeAnnotations struct */
/**\{ */
using CNetworkOfPoses2DInf_NA = CNetworkOfPoses<
	mrpt::poses::CPosePDFGaussianInf, mrpt::containers::map_traits_stdmap,
	mrpt::graphs::detail::TMRSlamNodeAnnotations>;
using CNetworkOfPoses3DInf_NA = CNetworkOfPoses<
	mrpt::poses::CPose3DPDFGaussianInf, mrpt::containers::map_traits_stdmap,
	mrpt::graphs::detail::TMRSlamNodeAnnotations>;
/**\} */

/** @} */  // end of grouping

}  // namespace graphs

// Specialization of TTypeName must occur in the same namespace:
// These ones are here instead of into mrpt-containers, to avoid
// the dependency mrpt-containers -> mrpt-typemeta
namespace typemeta
{
MRPT_DECLARE_TTYPENAME(mrpt::containers::map_traits_stdmap)
MRPT_DECLARE_TTYPENAME(mrpt::containers::map_traits_map_as_vector)
}  // namespace typemeta

}  // namespace mrpt

// Implementation of templates (in a separate file for clarity)
#include "CNetworkOfPoses_impl.h"

// Visualization related template classes
#include <mrpt/graphs/CVisualizer.h>
#include <mrpt/graphs/CMRVisualizer.h>
