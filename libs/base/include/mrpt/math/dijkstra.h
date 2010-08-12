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
#ifndef  MRPT_DIJKSTRA_H
#define  MRPT_DIJKSTRA_H

#include <mrpt/math/graphs.h>

#include <list>

namespace mrpt
{
	namespace math
	{
	    using namespace std;
		using namespace mrpt::utils;

		/** The Dijkstra algorithm for finding the shortest path between a given source node in a (weighted) directed graph and all other nodes.
		  *  The constructor takes as input the graph (the set of directed edges) computes all the needed data, then
		  *   successive calls to "getShortestPathTo" return the paths very efficiently.
		  *
		  *  Graphs are represented by instances of (or classes derived from) mrpt::math::CDirectedGraph , and node's IDs are uint64_t values,
		  *   although the type mrpt::utils::TNodeID is also provided for clarity.
		  *
		  * See <a href="http://www.mrpt.org/Example:Dijkstra_optimal_path_search_in_graphs" > this page on the wiki</a> for a complete example.
		  */
		template<typename TYPE_EDGES>
		class CDijkstra
		{
		public:
			typedef std::list<std::pair<TNodeID,TNodeID> >   TListEdges; //!< A list of edges used to describe a path on the graph

			static const TNodeID INVALID_TNODEID = static_cast<TNodeID>(-1);

		protected:

			// Variables:
			struct TDistance
			{
				unsigned dist;

				TDistance() : dist( std::numeric_limits<unsigned>::max() ) { }
				TDistance(const unsigned &D) : dist(D) { }
				const TDistance & operator =(const unsigned &D) { dist = D; return *this;}
			};

			struct TPrevious
			{
				TPrevious() : id( INVALID_TNODEID ) { }
				TNodeID  id;
			};


			std::map<TNodeID,TDistance>					d;		// distance
			std::map<TNodeID,TPrevious>					previous;
			std::map<TNodeID, std::pair<TNodeID,TNodeID> > previous_arcs;
			std::map<TNodeID, bool>						visited;
			TNodeID		m_source_node_ID;

		public:
			/** Constructor, which takes the input graph and executes the entire Dijkstra algorithm.
			  *
			  *  The graph is given by its directed edges with a class such as:
			  *
			  *  \code
			  *    map< pair<size_t,size_t>, TYPE_EDGES>   myGraph;
			  *  \endcode
			  *
			  *  If a function "functor_edge_weight" is provided which returns the weight of any given edge, then it will be used.
			  *  Otherwise, all edges will weight equally.
			  *
			  * \sa getShortestPathTo
			  * \exception std::exception If the source nodeID is not found in the graph
			  */
			CDijkstra(
				const mrpt::math::CDirectedGraph<TYPE_EDGES> &graph,
				const TNodeID	&source_node_ID,
				double (*functor_edge_weight)(const TYPE_EDGES &edge) =  NULL
				)
				: m_source_node_ID(source_node_ID)
			{
				MRPT_START;

				/*
				1  function Dijkstra(G, w, s)
				2     for each vertex v in V[G]                        // Initializations
				3           d[v] := infinity
				4           previous[v] := undefined
				5     d[s] := 0
				6     S := empty set
				7     Q := V[G]
				8     while Q is not an empty set                      // The algorithm itself
				9           u := Extract_Min(Q)
				10           S := S union {u}
				11           for each edge (u,v) outgoing from u
				12                  if d[u] + w(u,v) < d[v]             // Relax (u,v)
				13                        d[v] := d[u] + w(u,v)
				14                        previous[v] := u
				*/

				// Makea list of all the nodes in the graph:
				std::set<TNodeID> lstNode_IDs;
				graph.getAllNodes( lstNode_IDs );
				const size_t nNodes = lstNode_IDs.size();

				if ( lstNode_IDs.find(source_node_ID)==lstNode_IDs.end() )
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot find the source node_ID=%u in the graph",static_cast<unsigned int>(source_node_ID));

				// Init:
				// d: already initialized to infinity by default.
				// previous: idem
				// previous_arcs: idem
				// visited: idem
				size_t visitedCount = 0;
				d[source_node_ID] = 0;

				std::set<TNodeID>::const_iterator u;

				do  // The algorithm:
				{
					// Find the index of the minimum:
					unsigned min_d = std::numeric_limits<unsigned>::max();
					u = lstNode_IDs.end();

					for (std::set<TNodeID>::const_iterator i=lstNode_IDs.begin();i!=lstNode_IDs.end();++i)
					{
						if (d[*i].dist < min_d && !visited[*i])
						{
							u = i;
							min_d = d[*u].dist;
						}
					}

					ASSERT_(u!=lstNode_IDs.end());

					visited[*u] = true;
					visitedCount++;

					// For each arc from "u":
					std::set<TNodeID>	neighborsOfU;
					graph.getNeighborsOf(*u,neighborsOfU);
					for (std::set<TNodeID>::const_iterator i=neighborsOfU.begin();i!=neighborsOfU.end();++i)
					{
						if ( (min_d+1) < d[*i].dist)
						{
							d[*i].dist = min_d+1;
							previous[*i].id = *u;
							typename mrpt::math::CDirectedGraph<TYPE_EDGES>::const_iterator the_edge =
								graph.edges.find( make_pair(*u,*i) );
							if ( the_edge!=graph.end() )
							{	// *u -> *i
								previous_arcs[*i] = make_pair(*u,*i);
							}
							else
							{	// *i -> *u
								the_edge = graph.edges.find( make_pair<TNodeID,TNodeID>(*i,*u));
								ASSERT_(the_edge!=graph.end());
								previous_arcs[*i] = make_pair(*i,*u);
							}
						}
					}

				} while ( visitedCount<nNodes );

				MRPT_END;
			} // end Dijkstra

			/** Returns the shortest path between the source node passed in the constructor and the given target node.
			  * The reconstructed path contains a list of arcs (all of them exist in the graph with the given direction), such as the
			  *  the first edge starts at the origin passed in the constructor, and the last one contains the given target.
			  *
			  * \note An empty list of edges is returned when target equals the source node.
			  */
			void getShortestPathTo(
				const TNodeID   target_node_ID,
				TListEdges &out_path
				) const
			{
				out_path.clear();
				if (target_node_ID==m_source_node_ID) return;

				TNodeID nod = target_node_ID;
				do
				{
					std::map<TNodeID, std::pair<TNodeID,TNodeID> >::const_iterator it = previous_arcs.find(nod);
					ASSERT_(it!=previous_arcs.end())

					out_path.push_front( it->second );
					nod = previous.find(nod)->second.id;
				} while (nod!=m_source_node_ID);

			} // end of getShortestPathTo

		}; // end class

	} // End of namespace
} // End of namespace
#endif
