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
		/** A directed graph with the argument of the template specifying the type of the annotations in the edges.
		  *  This class only keeps a list of edges (in the member "edges"), so there is no information stored for each node but its existence referred by a node_ID.
		  * \sa mrpt::math::CDijkstra, mrpt::poses::CNetworkOfPoses
		  */
		template<class TYPE_EDGES>
		class CDirectedGraph
		{
		public:
			typedef size_t TNodeID;  //!< The type for node IDs
			static const TNodeID INVALID_TNODEID = static_cast<TNodeID>(-1);

			typedef TYPE_EDGES type_edges;  //!< The type of the graph edges
			typedef std::map< std::pair<TNodeID,TNodeID>, TYPE_EDGES > type_edges_map;  //!< The type of the member "edges"
			typedef typename type_edges_map::const_iterator const_iterator;
			typedef typename type_edges_map::iterator iterator;



			/** The public member with the directed edges in the graph */
			type_edges_map edges;

			CDirectedGraph(const type_edges_map &obj) : edges(obj) { }  //!< Copy constructor from a map<pair< >, >
			CDirectedGraph() : edges() {}  //!< Default constructor

			size_t edgeCount() const { return edges.size(); }  //!< The number of edges in the graph
			void clearEdges() { edges.clear(); } //!< Erase all edges

			iterator begin() { return edges.begin(); }
			iterator end() { return edges.end(); }
			const_iterator begin() const { return edges.begin(); }
			const_iterator end() const { return edges.end(); }

			/** Insert an edge (from -> to) with the given edge value. */
			inline void insertEdge(TNodeID from_nodeID, TNodeID to_nodeID,const TYPE_EDGES &edge_value )
			{ edges.insert(std::make_pair(std::make_pair(from_nodeID,to_nodeID),edge_value) ); }

			/** Test is the given directed edge exists. */
			inline bool edgeExists(TNodeID from_nodeID, TNodeID to_nodeID) const
			{ return edges.find(std::make_pair(from_nodeID,to_nodeID))!=edges.end(); }

			/** Return a reference to the content of a given edge.
			  * \exception std::exception if the given edge does not exist
			  */
			TYPE_EDGES & getEdge(TNodeID from_nodeID, TNodeID to_nodeID)
			{
				iterator it = edges.find(std::make_pair(from_nodeID,to_nodeID));
				if (it==edges.end())
					THROW_EXCEPTION( format("Edge %u->%u does not exist",(unsigned)from_nodeID,(unsigned)to_nodeID) )
				else return it->second;
			}

			/** Return a reference to the content of a given edge.
			  * \exception std::exception if the given edge does not exist
			  */
			const TYPE_EDGES & getEdge(TNodeID from_nodeID, TNodeID to_nodeID) const
			{
				const_iterator it = edges.find(std::make_pair(from_nodeID,to_nodeID));
				if (it==edges.end())
					THROW_EXCEPTION( format("Edge %u->%u does not exist",(unsigned)from_nodeID,(unsigned)to_nodeID) )
				else return it->second;
			}

			/** Erase a given edge (it has no effect if the edge didn't exist)
			  */
			inline void eraseEdge(TNodeID from_nodeID, TNodeID to_nodeID)
			{
				edges.erase(std::make_pair(from_nodeID,to_nodeID));
			}

			/** Return a list of all the node_ID's of the graph, generated from all the nodes that appear in the list of edges
			  */
			void getAllNodes( std::set<TNodeID> &lstNode_IDs) const
			{
				lstNode_IDs.clear();
				for (typename type_edges_map::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					lstNode_IDs.insert(it->first.first);
					lstNode_IDs.insert(it->first.second);
				}
			}

			/** Return the list of all neighbors of "nodeID", by creating a list of their node IDs. */
			void getNeighborsOf(const TNodeID nodeID, std::set<TNodeID> &neighborIDs) const
			{
				neighborIDs.clear();
				for (typename type_edges_map::const_iterator it=edges.begin();it!=edges.end();++it)
				{
					if (it->first.first==nodeID)
						neighborIDs.insert(it->first.second);
					else if (it->first.second==nodeID)
						neighborIDs.insert(it->first.first);
				}
			}

		};

	} // End of namespace
} // End of namespace
#endif
