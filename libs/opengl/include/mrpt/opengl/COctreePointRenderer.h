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
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CBox.h>


namespace mrpt
{
	namespace opengl
	{
		using namespace mrpt::utils;

		/** Template class that implements the data structure and algorithms for Octree-based efficient rendering.
		  *  \sa mrpt::opengl::CPointCloud, mrpt::opengl::CPointCloudColoured
		  */
		template <class Derived>
		class COctreePointRenderer
		{
		public:
			/** Default ctor */
			COctreePointRenderer() :
				m_octree_has_to_rebuild_all(true),
				m_visible_octree_nodes(0),
				m_visible_octree_nodes_ongoing(0),
				m_nonempty_nodes(0),
				m_nonempty_nodes_ongoing(0)
			{ }

			/** Copy ctor */
			COctreePointRenderer(const COctreePointRenderer &) :
				m_octree_has_to_rebuild_all(true)
			{ }


			enum { OCTREE_ROOT_NODE = 0 };
			enum { OCTREE_MAX_ELEMENTS_PER_NODE = 100000 };

		protected:
			// Helper methods in any CRTP template
			inline       Derived & octree_derived()       { return *static_cast<Derived*>(this); }
			inline const Derived & octree_derived() const { return *static_cast<const Derived*>(this); }

			/** Must be called at children class' render() previously to \a octree_render() */
			inline void octree_assure_uptodate() const
			{
				const_cast<COctreePointRenderer<Derived>*>(this)->internal_octree_assure_uptodate();
			}

			/** Called from the derived class if we have to rebuild the entire node tree. */
			inline void octree_mark_as_outdated() { m_octree_has_to_rebuild_all=true; }

			/** Render the entire octree recursively.
			  * Should be called from children's render() method.
			  */
			void octree_render(const mrpt::opengl::CRenderizable::TRenderInfo &ri ) const
			{
				m_visible_octree_nodes_ongoing = 0;
				m_nonempty_nodes_ongoing       = 0;

				TPixelCoordf cr_px[8];
				float        cr_z[8];
				octree_recursive_render(OCTREE_ROOT_NODE,ri, cr_px, cr_z, false /* corners are not computed for this first iteration */ );

				m_visible_octree_nodes = m_visible_octree_nodes_ongoing;
				m_nonempty_nodes       = m_nonempty_nodes_ongoing;
			}

		private:
			/** The structure for each octree spatial node. Each node can either be a leaf of has 8 children nodes.
			  *  Instead of pointers, children are referenced by their indices in \a m_octree_nodes
			  */
			struct OPENGL_IMPEXP TNode
			{
				TNode() :
					bb_min( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() ),
					bb_max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max() )
				{ }

				bool                  is_leaf;     //!< true: it's a leaf and \a pts has valid indices; false: \a children is valid.

				// In all cases, the bounding_box:
				mrpt::math::TPoint3Df  bb_min, bb_max;

				// Fields used if is_leaf=true
				std::vector<size_t>   pts;         //!< Point indices in the derived class that fall into this node.
				bool                  all;         //!< true: All elements in the reference object; false: only those in \a pts

				// Fields used if is_leaf=false
				mrpt::math::TPoint3Df center;      //!< [is_leaf=false] The center of the node, whose coordinates are used to decide between the 8 children nodes.
				size_t                child_id[8]; //!< [is_leaf=false] The indices in \a m_octree_nodes of the 8 children.

				/** update bounding box with a new point: */
				inline void update_bb(const mrpt::math::TPoint3Df &p)
				{
					keep_min(bb_min.x, p.x); keep_min(bb_min.y, p.y); keep_min(bb_min.z, p.z);
					keep_max(bb_max.x, p.x); keep_max(bb_max.y, p.y); keep_max(bb_max.z, p.z);
				}

				inline float getCornerX(int i) const { return (i & 0x01)==0 ? bb_min.x : bb_max.x; }
				inline float getCornerY(int i) const { return (i & 0x02)==0 ? bb_min.y : bb_max.y; }
				inline float getCornerZ(int i) const { return (i & 0x04)==0 ? bb_min.z : bb_max.z; }

				void setBBFromOrderInParent(const TNode &parent, int my_child_index)
				{
					// Coordinate signs are relative to the parent center (split point):
					switch (my_child_index)
					{
					case 0:  // x-, y-, z-
						bb_min = parent.bb_min;
						bb_max = parent.center;
						break;
					case 1:  // x+, y-, z-
						bb_min.x = parent.center.x; bb_max.x = parent.bb_max.x;
						bb_min.y = parent.bb_min.y; bb_max.y = parent.center.y;
						bb_min.z = parent.bb_min.z; bb_max.z = parent.center.z;
						break;
					case 2:  // x-, y+, z-
						bb_min.x = parent.bb_min.x; bb_max.x = parent.center.x;
						bb_min.y = parent.center.y; bb_max.y = parent.bb_max.y;
						bb_min.z = parent.bb_min.z; bb_max.z = parent.center.z;
						break;
					case 3:  // x+, y+, z-
						bb_min.x = parent.center.x; bb_max.x = parent.bb_max.x;
						bb_min.y = parent.center.y; bb_max.y = parent.bb_max.y;
						bb_min.z = parent.bb_min.z; bb_max.z = parent.center.z;
						break;
					case 4:  // x-, y-, z+
						bb_min.x = parent.bb_min.x; bb_max.x = parent.center.x;
						bb_min.y = parent.bb_min.y; bb_max.y = parent.center.y;
						bb_min.z = parent.center.z; bb_max.z = parent.bb_max.z;
						break;
					case 5:  // x+, y-, z+
						bb_min.x = parent.center.x; bb_max.x = parent.bb_max.x;
						bb_min.y = parent.bb_min.y; bb_max.y = parent.center.y;
						bb_min.z = parent.center.z; bb_max.z = parent.bb_max.z;
						break;
					case 6:  // x-, y+, z+
						bb_min.x = parent.bb_min.x; bb_max.x = parent.center.x;
						bb_min.y = parent.center.y; bb_max.y = parent.bb_max.y;
						bb_min.z = parent.center.z; bb_max.z = parent.bb_max.z;
						break;
					case 7:  // x+, y+, z+
						bb_min = parent.center;
						bb_max = parent.bb_max;
						break;
					default: throw std::runtime_error("my_child_index!=[0,7]");
					}
				}
			};

			bool  m_octree_has_to_rebuild_all;
			std::deque<TNode>  m_octree_nodes; //!< First one [0] is always the root node

			// Counters of visible octrees for each render:
			volatile mutable size_t m_visible_octree_nodes, m_visible_octree_nodes_ongoing;
			volatile mutable size_t m_nonempty_nodes, m_nonempty_nodes_ongoing;

			/** Render a given node. */
			void octree_recursive_render(
				size_t node_idx,
				const mrpt::opengl::CRenderizable::TRenderInfo &ri,
				TPixelCoordf cr_px[8],
				float        cr_z[8],
				bool         corners_are_all_computed = true
				) const
			{
				const TNode &node = m_octree_nodes[node_idx];

				if (!corners_are_all_computed)
				{
					for (int i=0;i<8;i++)
					{
						// project point:
						ri.projectPointPixels(
							node.getCornerX(i),node.getCornerY(i),node.getCornerZ(i),
							cr_px[i].x,cr_px[i].y,cr_z[i]);
					}
				}

				// If all "Z"'s are negative, the entire node's box is BEHIND the image plane:
				if (cr_z[0]<0 &&cr_z[1]<0 &&cr_z[2]<0 &&cr_z[3]<0 &&cr_z[4]<0 &&cr_z[5]<0 &&cr_z[6]<0 &&cr_z[7]<0)
					return;		// Stop recursion.

				// Keep the on-screen bounding box of this node:
				TPixelCoordf px_min( std::numeric_limits<float>::max(),std::numeric_limits<float>::max()), px_max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
				for (int i=0;i<8;i++)
				{
					keep_min(px_min.x,cr_px[i].x); keep_min(px_min.y,cr_px[i].y);
					keep_max(px_max.x,cr_px[i].x); keep_max(px_max.y,cr_px[i].y);
				}

				// Check if the node has points and is visible:
				if (node.is_leaf)
				{	// Render this leaf node:
					if (!node.pts.empty())
					{
						m_nonempty_nodes_ongoing++;

						// Check visibility of the node:
						// 1) If the eye is within the box, go on:
						bool is_visible =
								ri.camera_position.x>node.bb_min.x && ri.camera_position.x<node.bb_max.x &&
								ri.camera_position.y>node.bb_min.y && ri.camera_position.y<node.bb_max.y &&
								ri.camera_position.z>node.bb_min.z && ri.camera_position.z<node.bb_max.z;

						// 2) Is at least a part of the octree node WITHIN the image?
						if (!is_visible)
							is_visible = !( px_min.x>=ri.vp_width || px_min.y>=ri.vp_height || px_max.x<0 || px_max.y<0);

						if (is_visible)
						{
							m_visible_octree_nodes_ongoing++;

							const float render_area_sqpixels = std::abs(px_min.x-px_max.x) * std::abs(px_min.y-px_max.y);
							octree_derived().render_subset(node.all,node.pts,render_area_sqpixels);
						}
					}
				}
				else
				{	// Render children nodes:
					// If ALL my 8 corners are invisible, stop recursion:
					if ( px_min.x>=ri.vp_width || px_min.y>=ri.vp_height || px_max.x<0 || px_max.y<0)
						return;

					// Precompute the 19 (3*9-8) intermediary points so children don't have to compute them several times:
					const TPoint3Df p_Xm_Ym_Zm ( node.bb_min.x, node.bb_min.y, node.bb_min.z ); // 0
					const TPoint3Df p_X0_Ym_Zm ( node.center.x, node.bb_min.y, node.bb_min.z );
					const TPoint3Df p_Xp_Ym_Zm ( node.bb_max.x, node.bb_min.y, node.bb_min.z ); // 1
					const TPoint3Df p_Xm_Y0_Zm ( node.bb_min.x, node.center.y, node.bb_min.z );
					const TPoint3Df p_X0_Y0_Zm ( node.center.x, node.center.y, node.bb_min.z );
					const TPoint3Df p_Xp_Y0_Zm ( node.bb_max.x, node.center.y, node.bb_min.z );
					const TPoint3Df p_Xm_Yp_Zm ( node.bb_min.x, node.bb_max.y, node.bb_min.z ); // 2
					const TPoint3Df p_X0_Yp_Zm ( node.center.x, node.bb_max.y, node.bb_min.z );
					const TPoint3Df p_Xp_Yp_Zm ( node.bb_max.x, node.bb_max.y, node.bb_min.z ); // 3

					const TPoint3Df p_Xm_Ym_Z0 ( node.bb_min.x, node.bb_min.y, node.center.z );
					const TPoint3Df p_X0_Ym_Z0 ( node.center.x, node.bb_min.y, node.center.z );
					const TPoint3Df p_Xp_Ym_Z0 ( node.bb_max.x, node.bb_min.y, node.center.z );
					const TPoint3Df p_Xm_Y0_Z0 ( node.bb_min.x, node.center.y, node.center.z );
					const TPoint3Df p_X0_Y0_Z0 ( node.center.x, node.center.y, node.center.z );
					const TPoint3Df p_Xp_Y0_Z0 ( node.bb_max.x, node.center.y, node.center.z );
					const TPoint3Df p_Xm_Yp_Z0 ( node.bb_min.x, node.bb_max.y, node.center.z );
					const TPoint3Df p_X0_Yp_Z0 ( node.center.x, node.bb_max.y, node.center.z );
					const TPoint3Df p_Xp_Yp_Z0 ( node.bb_max.x, node.bb_max.y, node.center.z );

					const TPoint3Df p_Xm_Ym_Zp ( node.bb_min.x, node.bb_min.y, node.bb_max.z ); // 4
					const TPoint3Df p_X0_Ym_Zp ( node.center.x, node.bb_min.y, node.bb_max.z );
					const TPoint3Df p_Xp_Ym_Zp ( node.bb_min.x, node.bb_min.y, node.bb_max.z ); // 5
					const TPoint3Df p_Xm_Y0_Zp ( node.bb_min.x, node.center.y, node.bb_max.z );
					const TPoint3Df p_X0_Y0_Zp ( node.center.x, node.center.y, node.bb_max.z );
					const TPoint3Df p_Xp_Y0_Zp ( node.bb_max.x, node.center.y, node.bb_max.z );
					const TPoint3Df p_Xm_Yp_Zp ( node.bb_min.x, node.bb_max.y, node.bb_max.z ); // 6
					const TPoint3Df p_X0_Yp_Zp ( node.center.x, node.bb_max.y, node.bb_max.z );
					const TPoint3Df p_Xp_Yp_Zp ( node.bb_max.x, node.bb_max.y, node.bb_max.z ); // 7

					// Project all these points:
#define PROJ_SUB_NODE(POSTFIX) \
					TPixelCoordf px_##POSTFIX; \
					float        depth_##POSTFIX; \
					ri.projectPointPixels( p_##POSTFIX.x, p_##POSTFIX.y, p_##POSTFIX.z, px_##POSTFIX.x,px_##POSTFIX.y,depth_##POSTFIX);

#define PROJ_SUB_NODE_ALREADY_DONE(INDEX, POSTFIX) \
					const TPixelCoordf px_##POSTFIX = cr_px[INDEX]; \
					float        depth_##POSTFIX = cr_z[INDEX];

					PROJ_SUB_NODE_ALREADY_DONE(0,Xm_Ym_Zm)
					PROJ_SUB_NODE(X0_Ym_Zm)
					PROJ_SUB_NODE_ALREADY_DONE(1, Xp_Ym_Zm)

					PROJ_SUB_NODE(Xm_Y0_Zm)
					PROJ_SUB_NODE(X0_Y0_Zm)
					PROJ_SUB_NODE(Xp_Y0_Zm)

					PROJ_SUB_NODE_ALREADY_DONE(2, Xm_Yp_Zm)
					PROJ_SUB_NODE(X0_Yp_Zm)
					PROJ_SUB_NODE_ALREADY_DONE(3, Xp_Yp_Zm)

					PROJ_SUB_NODE(Xm_Ym_Z0)
					PROJ_SUB_NODE(X0_Ym_Z0)
					PROJ_SUB_NODE(Xp_Ym_Z0)
					PROJ_SUB_NODE(Xm_Y0_Z0)
					PROJ_SUB_NODE(X0_Y0_Z0)
					PROJ_SUB_NODE(Xp_Y0_Z0)
					PROJ_SUB_NODE(Xm_Yp_Z0)
					PROJ_SUB_NODE(X0_Yp_Z0)
					PROJ_SUB_NODE(Xp_Yp_Z0)

					PROJ_SUB_NODE_ALREADY_DONE(4, Xm_Ym_Zp)
					PROJ_SUB_NODE(X0_Ym_Zp)
					PROJ_SUB_NODE_ALREADY_DONE(5, Xp_Ym_Zp)

					PROJ_SUB_NODE(Xm_Y0_Zp)
					PROJ_SUB_NODE(X0_Y0_Zp)
					PROJ_SUB_NODE(Xp_Y0_Zp)

					PROJ_SUB_NODE_ALREADY_DONE(6, Xm_Yp_Zp)
					PROJ_SUB_NODE(X0_Yp_Zp)
					PROJ_SUB_NODE_ALREADY_DONE(7, Xp_Yp_Zp)

					// Recursive call children nodes:
#define DO_RECURSE_CHILD(INDEX, SEQ0,SEQ1,SEQ2,SEQ3,SEQ4,SEQ5,SEQ6,SEQ7) \
					{ \
						TPixelCoordf child_cr_px[8] = { px_##SEQ0,px_##SEQ1,px_##SEQ2,px_##SEQ3,px_##SEQ4,px_##SEQ5,px_##SEQ6,px_##SEQ7 }; \
						float        child_cr_z[8]  = { depth_##SEQ0,depth_##SEQ1,depth_##SEQ2,depth_##SEQ3,depth_##SEQ4,depth_##SEQ5,depth_##SEQ6,depth_##SEQ7 }; \
						this->octree_recursive_render(node.child_id[INDEX],ri,child_cr_px, child_cr_z); \
					}

					//                     0         1         2         3          4         5        6         7
					DO_RECURSE_CHILD(0, Xm_Ym_Zm, X0_Ym_Zm, Xm_Y0_Zm, X0_Y0_Zm, Xm_Ym_Z0, X0_Ym_Z0, Xm_Y0_Z0, X0_Y0_Z0 )
					DO_RECURSE_CHILD(1, X0_Ym_Zm, Xp_Ym_Zm, X0_Y0_Zm, Xp_Y0_Zm, X0_Ym_Z0, Xp_Ym_Z0, X0_Y0_Z0, Xp_Y0_Z0 )
					DO_RECURSE_CHILD(2, Xm_Y0_Zm, X0_Y0_Zm, Xm_Yp_Zm, X0_Yp_Zm, Xm_Y0_Z0, X0_Y0_Z0, Xm_Yp_Z0, X0_Yp_Z0 )
					DO_RECURSE_CHILD(3, X0_Y0_Zm, Xp_Y0_Zm, X0_Yp_Zm, Xp_Yp_Zm, X0_Y0_Z0, Xp_Y0_Z0, X0_Yp_Z0, Xp_Yp_Z0 )
					DO_RECURSE_CHILD(4, Xm_Ym_Z0, X0_Ym_Z0, Xm_Y0_Z0, X0_Y0_Z0, Xm_Ym_Zp, X0_Ym_Zp, Xm_Y0_Zp, X0_Y0_Zp )
					DO_RECURSE_CHILD(5, X0_Ym_Z0, Xp_Ym_Z0, X0_Y0_Z0, Xp_Y0_Z0, X0_Ym_Zp, Xp_Ym_Zp, X0_Y0_Zp, Xp_Y0_Zp )
					DO_RECURSE_CHILD(6, Xm_Y0_Z0, X0_Y0_Z0, Xm_Yp_Z0, X0_Yp_Z0, Xm_Y0_Zp, X0_Y0_Zp, Xm_Yp_Zp, X0_Yp_Zp )
					DO_RECURSE_CHILD(7, X0_Y0_Z0, Xp_Y0_Z0, X0_Yp_Z0, Xp_Yp_Z0, X0_Y0_Zp, Xp_Y0_Zp, X0_Yp_Zp, Xp_Yp_Zp )

#undef DO_RECURSE_CHILD
#undef PROJ_SUB_NODE
#undef PROJ_SUB_NODE_ALREADY_DONE

				}
			}

			// The actual implementation (and non-const version) of octree_assure_uptodate()
			void internal_octree_assure_uptodate()
			{
				if (!m_octree_has_to_rebuild_all) return;
				m_octree_has_to_rebuild_all = false;

				// Reset list of nodes:
				m_octree_nodes.assign(1, TNode() );

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

				const bool has_to_compute_bb = (node_id ==OCTREE_ROOT_NODE);

				if (N<=OCTREE_MAX_ELEMENTS_PER_NODE)
				{
					// No need to split this node:
					node.is_leaf = true;
					node.all     = all_pts;

					// Update bounding-box:
					if (has_to_compute_bb)
					{
						if (all_pts)
							 for (size_t i=0;i<N;i++) node.update_bb( octree_derived().getPointf(i) );
						else for (size_t i=0;i<N;i++) node.update_bb( octree_derived().getPointf(node.pts[i]) );
					}
				}
				else
				{
					// We have to split the node.
					// Compute the mean of all elements:
					mrpt::math::TPoint3Df mean(0,0,0);
					if (all_pts)
						for (size_t i=0;i<N;i++)
						{
							mrpt::math::TPoint3Df p = octree_derived().getPointf(i);
							mean+= p;
							if (has_to_compute_bb) node.update_bb( p );
						}
					else
						for (size_t i=0;i<N;i++)
						{
							mrpt::math::TPoint3Df p = octree_derived().getPointf(node.pts[i]);
							mean+= p;
							if (has_to_compute_bb) node.update_bb( p );
						}

					// Save my split point:
					node.is_leaf = false;
					node.center  = mean * (1.0f/N);

					// Allocate my 8 children structs
					const size_t children_idx_base = m_octree_nodes.size();
					m_octree_nodes.resize(children_idx_base + 8 );
					for (int i=0;i<8;i++)
						node.child_id[i] = children_idx_base + i;

					// Set the bounding-boxes of my children (we already know them):
					for (int i=0;i<8;i++)
						m_octree_nodes[children_idx_base + i].setBBFromOrderInParent(node,i);

					// Divide elements among children:
					const mrpt::math::TPoint3Df &c = node.center; // to make notation clearer
					for (size_t j=0;j<N;j++)
					{
						const size_t i = all_pts ? j : node.pts[j];
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

			/** Return the number of octree nodes (all of them, including the empty ones) \sa octree_get_nonempty_node_count */
			size_t octree_get_node_count() const { return m_octree_nodes.size(); }

			/** Return the number of non-empty octree nodes (those having at least one rendering element) */
			size_t octree_get_nonempty_node_count() const { return m_nonempty_nodes; }

			/** Return the number of visible octree nodes in the last render event. */
			size_t octree_get_visible_nodes() const { return m_visible_octree_nodes; }

			/** Returns a graphical representation of all the bounding boxes of the octree (leaf) nodes.
			  */
			void octree_get_graphics_boundingboxes(
				mrpt::opengl::CSetOfObjects &gl_bb,
				const double lines_width = 1,
				const TColorf lines_color = TColorf(1,1,1) ) const
			{
				octree_assure_uptodate();
				gl_bb.clear();
				for (size_t i=0;i<m_octree_nodes.size();i++)
				{
					const TNode & node = m_octree_nodes[i];
					if (!node.is_leaf) continue;
					mrpt::opengl::CBoxPtr gl_box = mrpt::opengl::CBox::Create();
					gl_box->setBoxCorners( mrpt::math::TPoint3D(node.bb_min), mrpt::math::TPoint3D(node.bb_max) );
					gl_box->setColor(lines_color);
					gl_box->setLineWidth(lines_width);
					gl_box->setWireframe(true);
					gl_bb.insert(gl_box);
				}
			}


			/** Used for debug only */
			void octree_debug_dump_tree(std::ostream &o) const
			{
				o << "Octree nodes: " << m_octree_nodes.size() << std::endl;
				size_t total_elements = 0;
				for (size_t i=0;i<m_octree_nodes.size();i++)
				{
					const TNode & node = m_octree_nodes[i];

					o << "Node #" << i << ": ";
					if (node.is_leaf)
					{
						o << "leaf, ";
						if (node.all) { o << "(all)\n"; total_elements+=octree_derived().size(); }
						else { o << node.pts.size() << " elements; "; total_elements+=node.pts.size(); }

					}
					else
					{
						o << "parent, center=(" << node.center.x << "," << node.center.y<<","<<node.center.z<<"), children: "
						  << node.child_id[0] << ","<< node.child_id[1] << ","<< node.child_id[2] << ","<< node.child_id[3] << ","
						  << node.child_id[4] << ","<< node.child_id[5] << ","<< node.child_id[6] << ","<< node.child_id[7] << "; ";
					}
					o << " bb: (" << node.bb_min.x << ","<< node.bb_min.y << ","<< node.bb_min.z << ")-("
					              << node.bb_max.x << ","<< node.bb_max.y << ","<< node.bb_max.z << ")\n";
				}
				o << "Total elements in all nodes: " << total_elements << std::endl;
			} // end of octree_debug_dump_tree

		}; // end of class COctreePointRenderer

	} // end namespace
} // End of namespace
#endif
