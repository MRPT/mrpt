/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_DIRECTEDGRAPH_H
#define  MRPT_DIRECTEDGRAPH_H

#include <mrpt/utils/utils_defs.h>
//#include <mrpt/math/utils.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace graphs
	{
		using mrpt::utils::TNodeID;
		using mrpt::utils::TPairNodeIDs;

		/** \addtogroup mrpt_graphs_grp
		    @{ */

		namespace detail
		{
			/** An empty structure */
			struct edge_annotations_empty {  };
		}

		/** A directed graph with the argument of the template specifying the type of the annotations in the edges.
		  *  This class only keeps a list of edges (in the member \a edges), so there is no information stored for each node but its existence referred by a node_ID.
		  *
		  *  Note that edges are stored as a std::multimap<> to allow <b>multiple edges</b> between the same pair of nodes.
		  *
		  * \sa mrpt::graphs::CDijkstra, mrpt::graphs::CNetworkOfPoses, mrpt::graphs::CDirectedTree
		 * \ingroup mrpt_graphs_grp
		  */
		template<class TYPE_EDGES, class EDGE_ANNOTATIONS = detail::edge_annotations_empty>
		class CDirectedGraph
		{
		public:
			/** The type of each global pose in \a nodes: an extension of the \a TYPE_EDGES pose with any optional user-defined data */
			struct edge_t : public TYPE_EDGES, public EDGE_ANNOTATIONS
			{
				// Replicate possible constructors:
				inline edge_t () : TYPE_EDGES() { }
				template <typename ARG1> inline edge_t (const ARG1 &a1) : TYPE_EDGES(a1) { }
				template <typename ARG1,typename ARG2> inline edge_t (const ARG1 &a1,const ARG2 &a2) : TYPE_EDGES(a1,a2) { }
			};


			typedef typename mrpt::aligned_containers<TPairNodeIDs,edge_t>::multimap_t	edges_map_t;  //!< The type of the member \a edges
			typedef typename edges_map_t::iterator         iterator;
			typedef typename edges_map_t::const_iterator   const_iterator;

			/** The public member with the directed edges in the graph */
			edges_map_t   edges;


			inline CDirectedGraph(const edges_map_t &obj) : edges(obj) { }  //!< Copy constructor from a multimap<pair< >, >
			inline CDirectedGraph() : edges() {}  //!< Default constructor

			inline iterator begin() { return edges.begin(); }
			inline iterator end() { return edges.end(); }
			inline const_iterator begin() const { return edges.begin(); }
			inline const_iterator end() const { return edges.end(); }

			/** @name Edges/nodes utility methods
				@{ */

			inline size_t edgeCount() const { return edges.size(); }  //!< The number of edges in the graph
			inline void clearEdges() { edges.clear(); } //!< Erase all edges


			/** Insert an edge (from -> to) with the given edge value. \sa insertEdgeAtEnd */
			inline void insertEdge(TNodeID from_nodeID, TNodeID to_nodeID,const edge_t &edge_value )
			{
				EIGEN_ALIGN16 typename edges_map_t::value_type entry(
					std::make_pair(from_nodeID,to_nodeID),
					edge_value);
				edges.insert(entry);
			}

			/** Insert an edge (from -> to) with the given edge value (more efficient version to be called if you know that the end will go at the end of the sorted std::multimap). \sa insertEdge */
			inline void insertEdgeAtEnd(TNodeID from_nodeID, TNodeID to_nodeID,const edge_t &edge_value )
			{
				EIGEN_ALIGN16 typename edges_map_t::value_type entry(
					std::make_pair(from_nodeID,to_nodeID),
					edge_value);
				edges.insert(edges.end(), entry);
			}

			/** Test is the given directed edge exists. */
			inline bool edgeExists(TNodeID from_nodeID, TNodeID to_nodeID) const
			{ return edges.find(std::make_pair(from_nodeID,to_nodeID))!=edges.end(); }

			/** Return a reference to the content of a given edge.
			  *  If several edges exist between the given nodes, the first one is returned.
			  * \exception std::exception if the given edge does not exist
			  * \sa getEdges
			  */
			edge_t & getEdge(TNodeID from_nodeID, TNodeID to_nodeID)
			{
				iterator it = edges.find(std::make_pair(from_nodeID,to_nodeID));
				if (it==edges.end())
					THROW_EXCEPTION( format("Edge %u->%u does not exist",(unsigned)from_nodeID,(unsigned)to_nodeID) )
				else return it->second;
			}

			/** Return a reference to the content of a given edge.
			  *  If several edges exist between the given nodes, the first one is returned.
			  * \exception std::exception if the given edge does not exist
			  * \sa getEdges
			  */
			const edge_t & getEdge(TNodeID from_nodeID, TNodeID to_nodeID) const
			{
				const_iterator it = edges.find(std::make_pair(from_nodeID,to_nodeID));
				if (it==edges.end())
					THROW_EXCEPTION( format("Edge %u->%u does not exist",(unsigned)from_nodeID,(unsigned)to_nodeID) )
				else return it->second;
			}

			/** Return a pair<first,last> of iterators to the range of edges between two given nodes. \sa getEdge  */
			std::pair<iterator,iterator> getEdges(TNodeID from_nodeID, TNodeID to_nodeID) {
				return edges.equal_range( std::make_pair(from_nodeID,to_nodeID) );
			}
			/** Return a pair<first,last> of const iterators to the range of edges between two given nodes.  \sa getEdge */
			std::pair<const_iterator,const_iterator> getEdges(TNodeID from_nodeID, TNodeID to_nodeID) const {
				return edges.equal_range( std::make_pair(from_nodeID,to_nodeID) );
			}

			/** Erase all edges between the given nodes (it has no effect if no edge existed)
			  */
			inline void eraseEdge(TNodeID from_nodeID, TNodeID to_nodeID) {
				edges.erase(std::make_pair(from_nodeID,to_nodeID));
			}

			/** Return a list of all the node_ID's of the graph, generated from all the nodes that appear in the list of edges
			  */
			void getAllNodes( std::set<TNodeID> &lstNode_IDs) const
			{
				lstNode_IDs.clear();
				for (typename edges_map_t::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					lstNode_IDs.insert(it->first.first);
					lstNode_IDs.insert(it->first.second);
				}
			}

			/** Less efficient way to get all nodes that returns a copy of the set object \sa getAllNodes( std::set<TNodeID> &lstNode_IDs) */
			inline std::set<TNodeID> getAllNodes() const { std::set<TNodeID> lst; getAllNodes(lst); return lst; }

			/** Count how many different node IDs appear in the graph edges.
			  */
			size_t countDifferentNodesInEdges() const
			{
				std::set<TNodeID> aux;
				for (typename edges_map_t::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					aux.insert(it->first.first);
					aux.insert(it->first.second);
				}
				return aux.size();
			}

			/** Return the list of all neighbors of "nodeID", by creating a list of their node IDs. \sa getAdjacencyMatrix */
			void getNeighborsOf(const TNodeID nodeID, std::set<TNodeID> &neighborIDs) const
			{
				neighborIDs.clear();
				for (typename edges_map_t::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					if (it->first.first==nodeID)
						neighborIDs.insert(it->first.second);
					else if (it->first.second==nodeID)
						neighborIDs.insert(it->first.first);
				}
			}

			/** Return a map from node IDs to all its neighbors (that is, connected nodes, regardless of the edge direction)
			  *  This is a much more efficient method than calling getNeighborsOf() for each node in the graph.
			  *  Possible values for the template argument MAP_NODEID_SET_NODEIDS are:
			  *    - std::map<TNodeID, std::set<TNodeID> >
			  *    - mrpt::utils::map_as_vector<TNodeID, std::set<TNodeID> >
			  * \sa getNeighborsOf
			  */
			template <class MAP_NODEID_SET_NODEIDS>
			void getAdjacencyMatrix( MAP_NODEID_SET_NODEIDS  &outAdjacency ) const
			{
				outAdjacency.clear();
				for (typename edges_map_t::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					outAdjacency[it->first.first].insert(it->first.second);
					outAdjacency[it->first.second].insert(it->first.first);
				}
			}

			/** Just like \a getAdjacencyMatrix but return only the adjacency for those node_ids in the set \a onlyForTheseNodes
			  *  (both endings nodes of an edge must be within the set for it to be returned) */
			template <class MAP_NODEID_SET_NODEIDS,class SET_NODEIDS>
			void getAdjacencyMatrix( MAP_NODEID_SET_NODEIDS  &outAdjacency, const SET_NODEIDS &onlyForTheseNodes ) const
			{
				outAdjacency.clear();
				const typename SET_NODEIDS::const_iterator setEnd = onlyForTheseNodes.end();
				for (typename edges_map_t::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					if (onlyForTheseNodes.find(it->first.first)==setEnd || onlyForTheseNodes.find(it->first.second)==setEnd)
						continue;
					outAdjacency[it->first.first].insert(it->first.second);
					outAdjacency[it->first.second].insert(it->first.first);
				}
			}

			/** @} */  // end of edge/nodes utilities


			/** @name I/O utilities
				@{ */
			/** Save the graph in a Graphviz (.dot files) text format; useful for quickly rendering the graph with "dot" 
			  * \param node_names If provided, these textual names will be used for naming the nodes instead of their numeric IDs given in the edges.
			  * \return false on any error */
			bool saveAsDot(std::ostream &o, const std::map<TNodeID,std::string> *node_names = NULL) const
			{
				o << "digraph G {\n";
				for (const_iterator it=begin();it!=end();++it)
				{
					const TNodeID id1 = it->first.first;
					const TNodeID id2 = it->first.second;
					std::string s1,s2;
					if (node_names)
					{
						std::map<TNodeID,std::string>::const_iterator itNam1=node_names->find(id1);
						if (itNam1!=node_names->end()) s1 =itNam1->second;
						std::map<TNodeID,std::string>::const_iterator itNam2=node_names->find(id2);
						if (itNam2!=node_names->end()) s2 =itNam2->second;
					}
					if (s1.empty()) s1 = mrpt::format("%u",static_cast<unsigned int>(id1));
					if (s2.empty()) s2 = mrpt::format("%u",static_cast<unsigned int>(id2));
					o << " \"" << s1 << "\" -> \"" << s2 << "\";\n";
				}
				return !((o << "}\n").fail());
			}
			
			/** \overload */
			bool saveAsDot(const std::string &fileName, const std::map<TNodeID,std::string> *node_names = NULL) const
			{
				std::ofstream f(fileName.c_str());
				if (!f.is_open()) return false;
				return saveAsDot(f,node_names);
			}
			/** @} */

		}; // end class CDirectedGraph

		/** @} */
	} // End of namespace
} // End of namespace
#endif
