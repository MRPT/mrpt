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
#ifndef opengl_COctreePointRenderer_H
#define opengl_COctreePointRenderer_H

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt
{
	namespace opengl
	{
		/** Template class that implements the data structure and algorithms for Octree-based efficient rendering.
		  *  \sa mrpt::opengl::CPointCloud, mrpt::opengl::CPointCloudColoured
		  */
		template <class Derived>
		class COctreePointRenderer
		{
		public:
			/** Default ctor */
			COctreePointRenderer() :
				m_octree_has_to_rebuild_all(true)
			{ }

			/** Copy ctor */
			COctreePointRenderer(const COctreePointRenderer &) :
				m_octree_has_to_rebuild_all(true)
			{ }


			enum { OCTREE_ROOT_NODE = 0 };
			enum { OCTREE_MAX_ELEMENTS_PER_NODE = 10000 };

		protected:
			// Helper methods in any CRTP template
			inline       Derived & octree_derived()       { return *static_cast<Derived*>(this); }
			inline const Derived & octree_derived() const { return *static_cast<const Derived*>(this); }

			/** Must be called at children class' render() previously to \a octree_recursive_render() */
			inline void octree_assure_uptodate() const
			{
				const_cast<COctreePointRenderer<Derived>*>(this)->internal_octree_assure_uptodate();
			}

			/** Render a given node.
			  * Should be called from children's render() method with node_idx=OCTREE_ROOT_NODE  to start the recursion.
			  */
			void octree_recursive_render(size_t node_idx, const mrpt::opengl::CRenderizable::TRenderInfo &ri ) const
			{
				const TNode &node = m_octree_nodes[node_idx];

				// Is this node visible?
				bool is_visible = true;

				// Render:
				if (is_visible)
				{
					if (node.is_leaf)
					{	// Render this leaf node:
						float largest_node_size_in_pixels = 100;
						octree_derived().render_subset(node.all,node.pts,largest_node_size_in_pixels);
					}
					else
					{	// Render children nodes:
						for (int i=0;i<8;i++)
							octree_recursive_render(node.child_id[i],ri);
					}
				}
			}

		private:
			/** The structure for each octree spatial node. Each node can either be a leaf of has 8 children nodes.
			  *  Instead of pointers, children are referenced by their indices in \a m_octree_nodes
			  */
			struct OPENGL_IMPEXP TNode
			{
				bool                  is_leaf;     //!< true: it's a leaf and \a pts has valid indices; false: \a children is valid.

				// Fields used if is_leaf=true
				std::vector<size_t>   pts;         //!< Point indices in the derived class that fall into this node.
				bool                  all;         //!< true: All elements in the reference object; false: only those in \a pts

				// Fields used if is_leaf=false
				mrpt::math::TPoint3Df center;      //!< [is_leaf=false] The center of the node, whose coordinates are used to decide between the 8 children nodes.
				size_t                child_id[8]; //!< [is_leaf=false] The indices in \a m_octree_nodes of the 8 children.
			};

			bool  m_octree_has_to_rebuild_all;
			std::deque<TNode>  m_octree_nodes; //!< First one [0] is always the root node

			// The actual implementation (and non-const version) of octree_assure_uptodate()
			void internal_octree_assure_uptodate()
			{
				if (!m_octree_has_to_rebuild_all) return;
				m_octree_has_to_rebuild_all = false;

				// Reset list of nodes:
				m_octree_nodes.resize(1);

				// recursive decide:
				internal_recursive_split( OCTREE_ROOT_NODE, true );
			}

			// Check the node "node_id" and create its children if needed, by looking at its list
			//  of elements (or all derived object's elements if "all_pts"=true, which will only happen
			//  for the root node)
			void internal_recursive_split(const size_t node_id, const bool all_pts = false)
			{
				TNode &node = m_octree_nodes[node_id];
				const size_t N = all_pts ? octree_derived().size() : node.pts.size();

				if (N<=OCTREE_MAX_ELEMENTS_PER_NODE)
				{
					// No need to split this node:
					node.is_leaf = true;
					node.all     = all_pts;
				}
				else
				{
					// We have to split the node.
					// Compute the mean of all elements:
					mrpt::math::TPoint3Df mean(0,0,0);
					if (all_pts)
						 for (size_t i=0;i<N;i++) mean+= octree_derived().getPointf(i);
					else for (size_t i=0;i<N;i++) mean+= octree_derived().getPointf(node.pts[i]);

					// Save my split point:
					node.is_leaf = false;
					node.center  = mean * (1.0f/N);

					// Allocate my 8 children structs:
					const size_t children_idx_base = m_octree_nodes.size();
					m_octree_nodes.resize(children_idx_base + 8 );
					for (int i=0;i<8;i++)
						node.child_id[i] = children_idx_base + i;


					// Divide elements among children:
					const mrpt::math::TPoint3Df &c = node.center; // to make notation clearer
					for (size_t j=0;j<N;j++)
					{
						const size_t i = all_pts ? i : node.pts[i];
						const TPoint3Df p = octree_derived().getPointf(i);
						if (p.z<c.z)
						{
							if (p.y<c.y)
							{
								if (p.x<c.x)
								      m_octree_nodes[children_idx_base+ 0 ].pts.push_back(i);
								else  m_octree_nodes[children_idx_base+ 1 ].pts.push_back(i);
							}
							else
							{
								if (p.x<c.x)
								      m_octree_nodes[children_idx_base+ 2 ].pts.push_back(i);
								else  m_octree_nodes[children_idx_base+ 3 ].pts.push_back(i);
							}
						}
						else
						{
							if (p.y<c.y)
							{
								if (p.x<c.x)
								      m_octree_nodes[children_idx_base+ 4 ].pts.push_back(i);
								else  m_octree_nodes[children_idx_base+ 5 ].pts.push_back(i);
							}
							else
							{
								if (p.x<c.x)
								      m_octree_nodes[children_idx_base+ 6 ].pts.push_back(i);
								else  m_octree_nodes[children_idx_base+ 7 ].pts.push_back(i);
							}
						}
					}

					// Clear list of elements (they're now in our children):
					{
						std::vector<size_t> emptyVec;
						node.pts.swap(emptyVec);  // This is THE way of really clearing a std::vector
					}

					// Recursive call on children:
					for (int i=0;i<8;i++)
						internal_recursive_split( node.child_id[i] );
				}
			} // end of internal_recursive_split

		public:
			/** Used for debug only */
			void octree_debug_dump_tree(std::ostream &o) const
			{
				o << "Octree nodes: " << m_octree_nodes.size() << std::endl;
				for (size_t i=0;i<m_octree_nodes.size();i++)
				{
					const TNode & node = m_octree_nodes[i];

					o << "Node #" << i << ": ";
					if (node.is_leaf)
					{
						o << "leaf, ";
						if (node.all) o << "(all)\n";
						else o << node.pts.size() << " elements.\n";
					}
					else
					{
						o << "parent, center=(" << node.center.x << "," << node.center.y<<","<<node.center.z<<"), children: "
						  << node.child_id[0] << ","<< node.child_id[1] << ","<< node.child_id[2] << ","<< node.child_id[3] << ","
						  << node.child_id[4] << ","<< node.child_id[5] << ","<< node.child_id[6] << ","<< node.child_id[7] << "\n";
					}
				}
			} // end of octree_debug_dump_tree

		}; // end of class COctreePointRenderer

	} // end namespace
} // End of namespace
#endif
