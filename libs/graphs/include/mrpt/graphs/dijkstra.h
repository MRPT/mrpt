/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/graphs/CDirectedGraph.h>
#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/containers/traits_map.h>
#include <mrpt/math/utils.h>

#include <limits>
#include <iostream>  // TODO - remove me
#include <vector>
#include <utility>
#include <exception>
#include <functional>

namespace mrpt::graphs
{
namespace detail
{
/**\brief Custom exception class that passes information in case an
 * unconnected graph is passed to a Dijkstra instance
 */
class NotConnectedGraph : public std::exception
{
   public:
	explicit NotConnectedGraph(
		const std::set<mrpt::graphs::TNodeID>& unconnected_nodeIDs,
		std::string err)
		: m_unconnected_nodeIDs(unconnected_nodeIDs), m_err(err + "\n\n")
	{
	}

	using std::exception::what;  // supress clang warning
	const char* what() { return m_err.c_str(); }
	~NotConnectedGraph() noexcept override = default;
	/**\brief Fill set with the nodeIDs Dijkstra algorithm could not reach
	 * starting from the root node.
	 */
	void getUnconnectedNodeIDs(
		std::set<mrpt::graphs::TNodeID>* set_nodeIDs) const
	{
		ASSERTMSG_(set_nodeIDs, "\nSet of nodes pointer is invalid\n");

		// fil the given set
		set_nodeIDs->clear();
		for (unsigned long m_unconnected_nodeID : m_unconnected_nodeIDs)
		{
			set_nodeIDs->insert(m_unconnected_nodeID);
		}
	}

   private:
	std::set<mrpt::graphs::TNodeID> m_unconnected_nodeIDs;
	std::string m_err;
};
}  // namespace detail

/** The Dijkstra algorithm for finding the shortest path between a given
 * source node in a (weighted) directed graph and all other nodes in the
 * form of a tree.
 *
 *  The constructor takes as input the graph (the set of directed edges)
 *  computes all the needed data, then successive calls to \a
 *  getShortestPathTo return the paths efficiently from the root.
 *
 *  The entire generated tree can be also retrieved with \a getTreeGraph.
 *
 *  Input graphs are represented by instances of (or classes derived from)
 *  mrpt::graphs::CDirectedGraph, and node's IDs are uint64_t values,
 *  although the type mrpt::graphs::TNodeID is also provided for clarity in
 *  the code.
 *
 *  The second template argument MAPS_IMPLEMENTATION allows choosing
 *  between a sparse std::map<> representation (using *
 *  mrpt::containers::map_traits_stdmap) for several intermediary and final
 *  results, and an alternative (using
 *  mrpt::containers::map_traits_map_as_vector as argument) dense
 *  implementation which is much faster, but can be only used if the
 *  TNodeID's start in 0 or a low value.
 *
 * See <a
 * href="http://www.mrpt.org/Example:Dijkstra_optimal_path_search_in_graphs"
 * > this page </a> for a complete example.
 *
 * \ingroup mrpt_graphs_grp
 */
template <
	class TYPE_GRAPH,
	class MAPS_IMPLEMENTATION = mrpt::containers::map_traits_stdmap>
class CDijkstra
{
   protected:
	/** Auxiliary struct for topological distances from root node */
	struct TDistance
	{
		double dist;
		inline TDistance() : dist(std::numeric_limits<double>::max()) {}
		inline TDistance(const double D) : dist(D) {}
		inline const TDistance& operator=(const double D)
		{
			dist = D;
			return *this;
		}
	};

	/** Auxiliary struct for backward paths */
	struct TPrevious
	{
		inline TPrevious() : id(INVALID_NODEID) {}
		TNodeID id;
	};

	// Cached input data:
	const TYPE_GRAPH& m_cached_graph;
	const TNodeID m_source_node_ID;

	// Private typedefs:
	/** A std::map (or a similar container according to MAPS_IMPLEMENTATION)
	 * with all the neighbors of every node. */
	using list_all_neighbors_t =
		typename MAPS_IMPLEMENTATION::template map<TNodeID, std::set<TNodeID>>;
	using id2pairIDs_map_t =
		typename MAPS_IMPLEMENTATION::template map<TNodeID, TPairNodeIDs>;
	using id2dist_map_t =
		typename MAPS_IMPLEMENTATION::template map<TNodeID, TDistance>;
	using id2id_map_t =
		typename MAPS_IMPLEMENTATION::template map<TNodeID, TPrevious>;

	// Intermediary and final results:
	/** All the distances */
	id2dist_map_t m_distances;
	std::map<TNodeID, TDistance>
		m_distances_non_visited;  // Use a std::map here in all cases.
	id2id_map_t m_prev_node;
	id2pairIDs_map_t m_prev_arc;
	std::set<TNodeID> m_lstNode_IDs;
	list_all_neighbors_t m_allNeighbors;

   public:
	/** @name Useful typedefs
		@{ */

	/** The type of the graph, typically a mrpt::graphs::CDirectedGraph<> or any
	 * other derived class */
	using graph_t = TYPE_GRAPH;
	/** The type of edge data in graph_t */
	using edge_t = typename graph_t::edge_t;
	/** A list of edges used to describe a path on the graph */
	using edge_list_t = std::list<TPairNodeIDs>;

	using functor_edge_weight_t = std::function<double(
		const graph_t& graph, const TNodeID id_from, const TNodeID id_to,
		const edge_t& edge)>;

	using functor_on_progress_t =
		std::function<void(const graph_t& graph, size_t visitedCount)>;

	/** @} */

	/** Constructor which takes the input graph and executes the entire
	 * Dijkstra algorithm from the given root node ID.
	 *
	 *  The graph is given by the set of directed edges, stored in a
	 *  mrpt::graphs::CDirectedGraph class.
	 *
	 *  If a function \a functor_edge_weight is provided, it will be used to
	 *  compute the weight of edges.  Otherwise, all edges weight the unity.
	 *
	 *  After construction, call \a getShortestPathTo to get the shortest
	 *  path to a node or \a getTreeGraph for the tree representation.
	 *
	 * \sa getShortestPathTo, getTreeGraph
	 *
	 * \exception std::exception If the source nodeID is not found in the
	 * graph
	 */
	CDijkstra(
		const graph_t& graph, const TNodeID source_node_ID,
		functor_edge_weight_t functor_edge_weight = functor_edge_weight_t(),
		functor_on_progress_t functor_on_progress = functor_on_progress_t())
		: m_cached_graph(graph), m_source_node_ID(source_node_ID)
	{
		/*
		1  function Dijkstra(G, w, s)
		2     for each vertex v in V[G]                        //
		Initializations
		3           m_distances[v] := infinity
		4           m_prev_node[v] := undefined
		5     m_distances[s] := 0
		6     S := empty set
		7     Q := V[G]
		8     while Q is not an empty set                      // The algorithm
		itself
		9           u := Extract_Min(Q)
		10           S := S union {u}
		11           for each edge (u,v) outgoing from u
		12                  if m_distances[u] + w(u,v) < m_distances[v]
		// Relax (u,v)
		13                        m_distances[v] := m_distances[u] + w(u,v)
		14                        m_prev_node[v] := u
		*/

		// Make a list of all the nodes in the graph:
		graph.getAllNodes(m_lstNode_IDs);
		const size_t nNodes = m_lstNode_IDs.size();

		if (m_lstNode_IDs.find(source_node_ID) == m_lstNode_IDs.end())
		{
			THROW_EXCEPTION_FMT(
				"Cannot find the source node_ID=%lu in the graph",
				static_cast<unsigned long>(source_node_ID));
		}

		// Init:
		// m_distances: already initialized to infinity by default.
		// m_prev_node: idem
		// m_prev_arc: idem
		// m_visited: idem
		size_t visitedCount = 0;
		m_distances[source_node_ID] = 0;
		m_distances_non_visited[source_node_ID] = 0;

		// Precompute all neighbors of all the nodes in the given graph:
		graph.getAdjacencyMatrix(m_allNeighbors);

		using namespace std;

		TNodeID u;
		// as long as there are nodes not yet visited.
		do
		{  // The algorithm:
			// Find the nodeID with the minimum known distance so far
			// considered:
			double min_d = std::numeric_limits<double>::max();
			u = INVALID_NODEID;

			// No need to check if the min. distance node is not visited yet,
			// since we
			// keep two lists: m_distances_non_visited & m_distances
			for (auto itDist = m_distances_non_visited.begin();
				 itDist != m_distances_non_visited.end(); ++itDist)
			{
				// TODO - remove these
				// cout << "Current min distance: " << min_d << endl;
				// cout << "itDist->first: " << itDist->first << endl;
				// cout << "itDist->second (distance to this node): " <<
				// itDist->second.dist << endl;

				if (itDist->second.dist < min_d)
				{
					u = itDist->first;
					min_d = itDist->second.dist;
					// cout << "updating u, min_d... " << endl;
				}
				// cout << "finished for." << endl;
			}

			// make sure we have found the next nodeID from the available
			// non-visited distances
			if (u == INVALID_NODEID)
			{
				std::set<TNodeID> nodeIDs_unconnected;

				// for all the nodes in the graph
				for (auto n_it = graph.nodes.begin(); n_it != graph.nodes.end();
					 ++n_it)
				{
					// have I already visited this node in Dijkstra?
					bool have_traversed = false;
					for (auto d_it = m_distances.begin();
						 d_it != m_distances.end(); ++d_it)
					{
						if (n_it->first == d_it->first)
						{
							have_traversed = true;
							break;
						}
					}

					if (!have_traversed)
					{
						nodeIDs_unconnected.insert(n_it->first);
					}
				}

				std::string err_str =
					mrpt::format("Graph is not fully connected!");
				throw mrpt::graphs::detail::NotConnectedGraph(
					nodeIDs_unconnected, err_str);
			}

			// Update distance (for possible future reference...) and remove
			// this node from "non-visited":
			m_distances[u] = m_distances_non_visited[u];
			m_distances_non_visited.erase(u);

			visitedCount++;

			// Let the user know about our progress...
			if (functor_on_progress) functor_on_progress(graph, visitedCount);

			// For each arc from "u":
			const std::set<TNodeID>& neighborsOfU =
				m_allNeighbors[u];  // graph.getNeighborsOf(u,neighborsOfU);
			for (unsigned long i : neighborsOfU)
			{
				if (i == u) continue;  // ignore self-loops...

				// the "edge_ui" may be searched here or a bit later, so the
				// "bool" var will tell us.
				typename graph_t::const_iterator edge_ui;
				bool edge_ui_reverse = false;
				bool edge_ui_found = false;

				// Get weight of edge u<->i
				double edge_ui_weight;
				if (!functor_edge_weight)
					edge_ui_weight = 1.;
				else
				{  // edge may be i->u or u->i:
					edge_ui = graph.edges.find(std::make_pair(u, i));
					if (edge_ui == graph.edges.end())
					{
						edge_ui = graph.edges.find(std::make_pair(i, u));
						edge_ui_reverse = true;
					}
					ASSERT_(edge_ui != graph.edges.end());
					edge_ui_weight = functor_edge_weight(
						graph, edge_ui->first.first, edge_ui->first.second,
						edge_ui->second);
					edge_ui_found = true;
				}

				if ((min_d + edge_ui_weight) < m_distances[i].dist)
				{  // the [] creates the entry if needed
					// update m_distances, m_distances_non_visited
					m_distances_non_visited[i].dist = min_d + edge_ui_weight;
					m_distances[i].dist = min_d + edge_ui_weight;

					m_prev_node[i].id = u;
					// If still not done above, detect the direction of the arc
					// now:
					if (!edge_ui_found)
					{
						edge_ui = graph.edges.find(std::make_pair(u, i));
						if (edge_ui == graph.edges.end())
						{
							edge_ui = graph.edges.find(std::make_pair(i, u));
							edge_ui_reverse = true;
						}
						ASSERT_(edge_ui != graph.edges.end());
					}

					if (!edge_ui_reverse)
						m_prev_arc[i] = std::make_pair(u, i);  // *u -> *i
					else
						m_prev_arc[i] = std::make_pair(i, u);  // *i -> *u
				}
			}
		} while (visitedCount < nNodes);

	}  // end Dijkstra

	/** @name Query Dijkstra results
	  @{ */

	/** Return the distance from the root node to any other node using the
	 * Dijkstra-generated tree
	 *
	 * \exception std::exception On unknown node ID
	 */
	inline double getNodeDistanceToRoot(const TNodeID id) const
	{
		typename id2dist_map_t::const_iterator it = m_distances.find(id);
		if (it == m_distances.end())
			THROW_EXCEPTION(
				"Node was not found in the graph when running Dijkstra");
		return it->second.dist;
	}

	/** Return the set of all known node IDs (actually, a const ref to the
	 * internal set object). */
	inline const std::set<TNodeID>& getListOfAllNodes() const
	{
		return m_lstNode_IDs;
	}

	/** Return the node ID of the tree root, as passed in the constructor */
	inline TNodeID getRootNodeID() const { return m_source_node_ID; }
	/** Return the adjacency matrix of the input graph, which is cached at
	 * construction so if needed later just use this copy to avoid
	 * recomputing it
	 *
	 * \sa  mrpt::graphs::CDirectedGraph::getAdjacencyMatrix
	 * */
	inline const list_all_neighbors_t& getCachedAdjacencyMatrix() const
	{
		return m_allNeighbors;
	}

	/** Returns the shortest path between the source node passed in the
	 * constructor and the given target node. The reconstructed path
	 * contains a list of arcs (all of them exist in the graph with the given
	 * direction), such as the the first edge starts at the origin passed in
	 * the constructor, and the last one contains the given target.
	 *
	 * \note An empty list of edges is returned when target equals the source
	 * node.
	 *
	 * \sa getTreeGraph
	 */
	void getShortestPathTo(
		const TNodeID target_node_ID, edge_list_t& out_path) const
	{
		out_path.clear();
		if (target_node_ID == m_source_node_ID) return;

		TNodeID nod = target_node_ID;
		do
		{
			typename id2pairIDs_map_t::const_iterator it = m_prev_arc.find(nod);
			ASSERT_(it != m_prev_arc.end());
			out_path.push_front(it->second);
			nod = m_prev_node.find(nod)->second.id;
		} while (nod != m_source_node_ID);

	}  // end of getShortestPathTo

	/** Type for graph returned by \a getTreeGraph: a graph like the original
	 * input graph, but with edge data being pointers to the original data
	 * (to save copy time & memory)
	 */
	using tree_graph_t = CDirectedTree<const edge_t*>;

	/** Returns a tree representation of the graph, as determined by the
	 * Dijkstra shortest paths from the root node.
	 * Note that the annotations on each edge in the tree are "const pointers"
	 * to the original graph edge data, so
	 * it's mandatory for the original input graph not to be deleted as long as
	 * this tree is used.
	 * \sa getShortestPathTo
	 */
	void getTreeGraph(tree_graph_t& out_tree) const
	{
		using TreeEdgeInfo = typename tree_graph_t::TEdgeInfo;

		out_tree.clear();
		out_tree.root = m_source_node_ID;
		// For each saved arc in "m_prev_arc", recover the original data in the
		// input graph and save it to the output tree structure.
		for (auto itArcs = m_prev_arc.begin(); itArcs != m_prev_arc.end();
			 ++itArcs)
		{
			const TNodeID id = itArcs->first;
			const TNodeID id_from = itArcs->second.first;
			const TNodeID id_to = itArcs->second.second;

			std::list<TreeEdgeInfo>& edges =
				out_tree.edges_to_children[id == id_from ? id_to : id_from];
			TreeEdgeInfo newEdge(id);
			newEdge.reverse = (id == id_from);  // true: root towards leafs.
			auto itEdgeData =
				m_cached_graph.edges.find(std::make_pair(id_from, id_to));
			ASSERTMSG_(
				itEdgeData != m_cached_graph.edges.end(),
				format(
					"Edge %u->%u is in Dijkstra paths but not in original "
					"graph!",
					static_cast<unsigned int>(id_from),
					static_cast<unsigned int>(id_to)));
			newEdge.data = &itEdgeData->second;
			edges.push_back(newEdge);
		}

	}  // end getTreeGraph

	/** @} */

};  // end class

}  // namespace mrpt::graphs
