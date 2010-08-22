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
#ifndef  MRPT_GRAPHS_H
#define  MRPT_GRAPHS_H

#include <mrpt/math/utils.h>
#include <set>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		using mrpt::utils::TNodeID;

		/** @name Graph-related classes
		    @{ */

		/** A directed graph with the argument of the template specifying the type of the annotations in the edges.
		  *  This class only keeps a list of edges (in the member \a edges), so there is no information stored for each node but its existence referred by a node_ID.
		  *
		  *  Note that edges are stored as a std::multimap<> to allow <b>multiple edges</b> between the same pair of nodes.
		  *
		  * \sa mrpt::math::CDijkstra, mrpt::poses::CNetworkOfPoses, mrpt::math::CDirectedTree
		  */
		template<class TYPE_EDGES>
		class CDirectedGraph
		{
		public:
			typedef TYPE_EDGES                                            edge_t;  //!< The type of the graph edges
			typedef std::multimap< std::pair<TNodeID,TNodeID>, edge_t >   edges_map_t;  //!< The type of the member \a edges
			typedef typename edges_map_t::iterator                        iterator;
			typedef typename edges_map_t::const_iterator                  const_iterator;

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
			{ edges.insert(std::make_pair(std::make_pair(from_nodeID,to_nodeID),edge_value) ); }

			/** Insert an edge (from -> to) with the given edge value (more efficient version to be called if you know that the end will go at the end of the sorted std::multimap). \sa insertEdge */
			inline void insertEdgeAtEnd(TNodeID from_nodeID, TNodeID to_nodeID,const edge_t &edge_value )
			{ edges.insert(edges.end(),std::make_pair(std::make_pair(from_nodeID,to_nodeID),edge_value)); }

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

			/** @} */  // end of edge/nodes utilities

		}; // end class CDirectedGraph


		/** A special kind of graph in the form of a tree with directed edges and optional edge annotations of templatized type "TYPE_EDGES".
		  *  The tree is represented by means of:
		  *		- \a root: The ID of the root node.
		  *		- \a edges_to_children: A map from node ID to all the edges to its children.
		  *
		  *  This class is less general than CDirectedGraph but more efficient to traverse (see \a visitDepthFirst and \a visitBreadthFirst).
		  *
		  *  If annotations in edges are not required, you can leave TYPE_EDGES to its default type "uint8_t".
		  *  \sa CDirectedGraph, CDijkstra, mrpt::poses::CNetworkOfPoses,
		  */
		template <class TYPE_EDGES = uint8_t>
		class CDirectedTree
		{
		public:
			struct TEdgeInfo
			{
				TNodeID    id;      //!< The ID of the child node.
				bool       reverse; //!< True if edge direction is child->parent, false if it's parent->child.
				TYPE_EDGES data;    //!< User data for this edge.
			};
			typedef std::list<TEdgeInfo>          TListEdges;
			typedef std::map<TNodeID,TListEdges>  TMapNode2ListEdges;

			/** @name Data
			    @{ */
			TNodeID            root;               //!< The root of the tree
			TMapNode2ListEdges edges_to_children;  //!< The edges of each node
			/** @} */

			/** @name Utilities
			    @{ */

			/** Empty all edge data and set "root" to INVALID_NODEID */
			void clear() { edges_to_children.clear(); root = INVALID_NODEID; }

			/** Virtual base class for user-defined visitors */
			struct Visitor
			{
				typedef CDirectedTree<TYPE_EDGES> tree_t;

				/** Virtual method to be implemented by the user and which will be called during the visit to a graph with visitDepthFirst or visitBreadthFirst
				  *  Specifically, the method will be called once for each <b>edge</b> in the tree.
				  * \param parent [IN] The ID of the parent node.
				  * \param edge_to_child [IN] The edge information from the parent to "edge_to_child.id"
				  * \param depth_level [IN] The "depth level" of the child node "edge_to_child.id" (root node is at 0, its children are at 1, etc.).
				  */
				virtual void OnVisitNode( const TNodeID parent, const typename tree_t::TEdgeInfo &edge_to_child, const size_t depth_level ) = 0;
			};

			/** Depth-first visit of all children nodes of a given root (itself excluded from the visit), invoking a user-provided function for each node/edge. \sa visitBreadthFirst */
			void visitDepthFirst( const TNodeID root, Visitor & user_visitor, const size_t root_depth_level =0 ) const
			{
				const size_t next_depth_level = root_depth_level+1;
				typename TMapNode2ListEdges::const_iterator itChildren = edges_to_children.find(root);
				if (itChildren==edges_to_children.end()) return; // No children
				const TListEdges &children = itChildren->second;
				for (typename TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
				{
					user_visitor.OnVisitNode(root,*itEdge,next_depth_level);
					visitDepthFirst(itEdge->id,user_visitor, next_depth_level); // Recursive depth-first call.
				}
			}

			/** Breadth-first visit of all children nodes of a given root (itself excluded from the visit), invoking a user-provided function for each node/edge. \sa visitDepthFirst */
			void visitBreadthFirst( const TNodeID root, Visitor & user_visitor, const size_t root_depth_level =0  ) const
			{
				const size_t next_depth_level = root_depth_level+1;
				typename TMapNode2ListEdges::const_iterator itChildren = edges_to_children.find(root);
				if (itChildren==edges_to_children.end()) return; // No children
				const TListEdges &children = itChildren->second;
				for (typename TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
					user_visitor.OnVisitNode(root,*itEdge,next_depth_level);
				for (typename TListEdges::const_iterator itEdge=children.begin();itEdge!=children.end();++itEdge)
					visitDepthFirst(itEdge->id,user_visitor,next_depth_level); // Recursive breath-first call.
			}

			/** Return a text representation of the tree spanned in a depth-first view, as in this example:
			  *  \code
			  *    0
			  *     -> 1
			  *     -> 2
			  *         -> 4
			  *         -> 5
			  *     -> 3
			  *  \endcode
			  */
			std::string getAsTextDescription() const
			{
				std::ostringstream s;
				struct CMyVisitor : public mrpt::math::CDirectedTree<TYPE_EDGES>::Visitor
				{
					std::ostringstream  &m_s;
					CMyVisitor(std::ostringstream &s) : m_s(s) { }
					virtual void OnVisitNode( const TNodeID parent, const typename mrpt::math::CDirectedTree<TYPE_EDGES>::Visitor::tree_t::TEdgeInfo &edge_to_child, const size_t depth_level ) {
						m_s << std::string(depth_level*5, ' ') << (edge_to_child.reverse ? "<-" : "->" ) //;
							<< std::setw(3) << edge_to_child.id << std::endl;
					}
				};
				CMyVisitor myVisitor(s);
				s << std::setw(3) << root << std::endl;
				visitDepthFirst( root, myVisitor );
				return s.str();
			}

			/** @} */

		};

		/** @} */
	} // End of namespace
} // End of namespace
#endif
