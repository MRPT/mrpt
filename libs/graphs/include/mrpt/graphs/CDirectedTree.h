/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_DIRECTED_TREE_H
#define  MRPT_DIRECTED_TREE_H

#include <mrpt/utils/utils_defs.h>
#include <list>

namespace mrpt
{
	namespace graphs
	{
		using mrpt::utils::TNodeID; //!< Make available this typedef in this namespace too

		/** A special kind of graph in the form of a tree with directed edges and optional edge annotations of templatized type "TYPE_EDGES".
		  *  The tree is represented by means of:
		  *		- \a root: The ID of the root node.
		  *		- \a edges_to_children: A map from node ID to all the edges to its children.
		  *
		  *  Note that nodes are *not* explicitly listed anywhere: their existence is only inferred from their ID numbers in the list
		  *  of edges in the \a edges_to_children data structure. If you want to include information for each node, derive from this class
		  *  and create a separte container for that data.
		  *
		  *  This class is less general than CDirectedGraph but more efficient to traverse (see \a visitDepthFirst and \a visitBreadthFirst).
		  *
		  *  If annotations in edges are not required, you can leave TYPE_EDGES to its default type "uint8_t".
		  *
		  *  Example of insertion of a new edge:
		  *  \code
		  *  typedef CDirectedTree<edge_t>  my_tree_t;
		  *  my_tree_t  tree;
		  *  TNodeID id_root = XXX;
		  *  TNodeID id_child = XXX;
		  *  my_tree_t::TListEdges & edges_of_root = tree.edges_to_children[id_root];
		  *  edges_of_root.push_back( my_tree_t::TEdgeInfo(id_child,false, edge_t(...) ) );
		  *  \endcode
		  *
		  *  \sa CDirectedGraph, CDijkstra, mrpt::graphs::CNetworkOfPoses
		 * \ingroup mrpt_graphs_grp
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

				/** Edge constructor from data */
				TEdgeInfo(TNodeID child_id_, bool direction_child_to_parent=false, const TYPE_EDGES & edge_data = TYPE_EDGES() ) : id(child_id_), reverse(direction_child_to_parent), data(edge_data) { }
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
				struct CMyVisitor : public mrpt::graphs::CDirectedTree<TYPE_EDGES>::Visitor
				{
					std::ostringstream  &m_s;
					CMyVisitor(std::ostringstream &s) : m_s(s) { }
					virtual void OnVisitNode( const TNodeID parent, const typename mrpt::graphs::CDirectedTree<TYPE_EDGES>::Visitor::tree_t::TEdgeInfo &edge_to_child, const size_t depth_level ) MRPT_OVERRIDE {
						m_s << std::string(depth_level*5, ' ') << (edge_to_child.reverse ? "<-" : "->" ) //;
							<< edge_to_child.id << std::endl;
					}
				};
				CMyVisitor myVisitor(s);
				s <<  root << std::endl;
				visitDepthFirst( root, myVisitor );
				return s.str();
			}

		};

		/** @} */
	} // End of namespace
} // End of namespace
#endif
