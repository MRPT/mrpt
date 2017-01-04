/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_COctreePointRenderer_H
#define opengl_COctreePointRenderer_H

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/utils/aligned_containers.h>

namespace mrpt
{
	namespace global_settings
	{
		/** Default value = 0.01 points/px^2. Affects to these classes (read their docs for further details):
		  *		- mrpt::opengl::CPointCloud
		  *		- mrpt::opengl::CPointCloudColoured
		  * \ingroup mrpt_opengl_grp
		  */
		extern OPENGL_IMPEXP float OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL;

		/** Default value = 1e5. Maximum number of elements in each octree node before spliting. Affects to these classes (read their docs for further details):
		  *		- mrpt::opengl::CPointCloud
		  *		- mrpt::opengl::CPointCloudColoured
		  * \ingroup mrpt_opengl_grp
		  */
		extern OPENGL_IMPEXP size_t OCTREE_RENDER_MAX_POINTS_PER_NODE;
	}


	namespace opengl
	{
		/** Template class that implements the data structure and algorithms for Octree-based efficient rendering.
		  *  \sa mrpt::opengl::CPointCloud, mrpt::opengl::CPointCloudColoured, http://www.mrpt.org/Efficiently_rendering_point_clouds_of_millions_of_points
		  * \ingroup mrpt_opengl_grp
		  */
		template <class Derived>
		class COctreePointRenderer
		{
		public:
			/** Default ctor */
			COctreePointRenderer() :
				m_octree_has_to_rebuild_all(true),
				m_visible_octree_nodes(0),
				m_visible_octree_nodes_ongoing(0)
			{ }

			/** Copy ctor */
			COctreePointRenderer(const COctreePointRenderer &) :
				m_octree_has_to_rebuild_all(true),
				m_visible_octree_nodes(0),
				m_visible_octree_nodes_ongoing(0)
			{ }


			enum { OCTREE_ROOT_NODE = 0 };

		protected:
			// Helper methods in any CRTP template
			inline       Derived & octree_derived()       { return *static_cast<Derived*>(this); }
			inline const Derived & octree_derived() const { return *static_cast<const Derived*>(this); }

			/** Must be called at children class' render() previously to \a octree_render() */
			inline void octree_assure_uptodate() const
			{
				const_cast<COctreePointRenderer<Derived>*>(this)->internal_octree_assure_uptodate();
			}

			/** Render the entire octree recursively.
			  * Should be called from children's render() method.
			  */
			void octree_render(const mrpt::opengl::gl_utils::TRenderInfo &ri ) const
			{
				m_visible_octree_nodes_ongoing = 0;

				// Stage 1: Build list of visible octrees
				m_render_queue.clear();
				m_render_queue.reserve(m_octree_nodes.size());

				mrpt::utils::TPixelCoordf cr_px[8];
				float        cr_z[8];
				octree_recursive_render(OCTREE_ROOT_NODE,ri, cr_px, cr_z, false /* corners are not computed for this first iteration */ );

				m_visible_octree_nodes = m_visible_octree_nodes_ongoing;

				// Stage 2: Render them all
				for (size_t i=0;i<m_render_queue.size();i++)
				{
					const TNode & node = m_octree_nodes[ m_render_queue[i].node_id ];
					octree_derived().render_subset( node.all,node.pts,m_render_queue[i].render_area_sqpixels);
				}
			}


			void octree_getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
			{
				octree_assure_uptodate();
				if (!m_octree_nodes.empty())
				{
					bb_min = mrpt::math::TPoint3D( m_octree_nodes[0].bb_min );
					bb_max = mrpt::math::TPoint3D( m_octree_nodes[0].bb_max );
				}
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
					mrpt::utils::keep_min(bb_min.x, p.x); mrpt::utils::keep_min(bb_min.y, p.y); mrpt::utils::keep_min(bb_min.z, p.z);
					mrpt::utils::keep_max(bb_max.x, p.x); mrpt::utils::keep_max(bb_max.y, p.y); mrpt::utils::keep_max(bb_max.z, p.z);
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

            public:
				MRPT_MAKE_ALIGNED_OPERATOR_NEW
			};

			struct OPENGL_IMPEXP TRenderQueueElement
			{
				inline TRenderQueueElement(const size_t id, float area_sq) : node_id(id), render_area_sqpixels(area_sq) {  }

				size_t  node_id;              //!< The node ID to render
				float   render_area_sqpixels; //!< The approximate size of the octree on the screen (squared pixels).
			};
			mutable std::vector<TRenderQueueElement>  m_render_queue; //!< The list of elements that really are visible and will be rendered.


			bool  m_octree_has_to_rebuild_all;
			typename mrpt::aligned_containers<TNode>::deque_t  m_octree_nodes; //!< First one [0] is always the root node

			// Counters of visible octrees for each render:
			volatile mutable size_t m_visible_octree_nodes, m_visible_octree_nodes_ongoing;

			/** Render a given node. */
			void octree_recursive_render(
				size_t node_idx,
				const mrpt::opengl::gl_utils::TRenderInfo &ri,
				mrpt::utils::TPixelCoordf cr_px[8],
				float        cr_z[8],
				bool         corners_are_all_computed = true,
				bool         trust_me_youre_visible   = false,
				float        approx_area_sqpixels     = 0
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

				mrpt::utils::TPixelCoordf px_min( std::numeric_limits<float>::max(),std::numeric_limits<float>::max()), px_max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
				if (!trust_me_youre_visible)
				{
					// Keep the on-screen bounding box of this node:
					for (int i=0;i<8;i++)
					{
						mrpt::utils::keep_min(px_min.x,cr_px[i].x); mrpt::utils::keep_min(px_min.y,cr_px[i].y);
						mrpt::utils::keep_max(px_max.x,cr_px[i].x); mrpt::utils::keep_max(px_max.y,cr_px[i].y);
					}

					const bool any_cr_zs_neg = (cr_z[0]<0 ||cr_z[1]<0 ||cr_z[2]<0 ||cr_z[3]<0 ||cr_z[4]<0 ||cr_z[5]<0 ||cr_z[6]<0 ||cr_z[7]<0);
					const bool any_cr_zs_pos = (cr_z[0]>0 ||cr_z[1]>0 ||cr_z[2]>0 ||cr_z[3]>0 ||cr_z[4]>0 ||cr_z[5]>0 ||cr_z[6]>0 ||cr_z[7]>0);
					const bool box_crosses_image_plane = any_cr_zs_pos && any_cr_zs_neg;

					// If all 8 corners are way out of the screen (and all "cr_z" have the same sign),
					// this node and all the children are not visible:
					if (!box_crosses_image_plane && ( px_min.x>=ri.vp_width || px_min.y>=ri.vp_height || px_max.x<0 || px_max.y<0) )
						return; // Not visible
				}

				// Check if the node has points and is visible:
				if (node.is_leaf)
				{	// Render this leaf node:
					if (node.all || !node.pts.empty())
					{
						// If we are here, it seems at least a part of the Box is visible:
						m_visible_octree_nodes_ongoing++;

						const float render_area_sqpixels = trust_me_youre_visible ?
							approx_area_sqpixels
							:
							std::abs(px_min.x-px_max.x) * std::abs(px_min.y-px_max.y);

						// OK: Add to list of rendering-pending:
						m_render_queue.push_back( TRenderQueueElement(node_idx,render_area_sqpixels) );
					}
				}
				else
				{	// Render children nodes:
					// If ALL my 8 corners are within the screen, tell our children that they
					//  won't need to compute anymore, since all of them and their children are visible as well:
					bool children_are_all_visible_for_sure = true;

					if (!trust_me_youre_visible) // Trust my parent... otherwise:
					{
						for (int i=0;i<8;i++)
						{
							if (!( cr_px[i].x>=0 && cr_px[i].y>=0 && cr_px[i].x<ri.vp_width && cr_px[i].y<ri.vp_height ))
							{
								children_are_all_visible_for_sure = false;
								break;
							}
						}
					}

					// If all children are visible, it's easy:
					if (children_are_all_visible_for_sure)
					{
						mrpt::utils::TPixelCoordf child_cr_px[8]; // No need to initialize
						float        child_cr_z[8];  // No need to initialize

						// Approximate area of the children nodes:
						const float approx_child_area = trust_me_youre_visible ?
							approx_area_sqpixels/8.0f
							:
							std::abs(px_min.x-px_max.x) * std::abs(px_min.y-px_max.y) / 8.0f;

						for (int i=0;i<8;i++)
							this->octree_recursive_render(node.child_id[i],ri,child_cr_px, child_cr_z, true,  true, approx_child_area); \
					}
					else
					{
#ifdef __clang__
#pragma clang diagnostic push  // clang complains about unused vars (becase it doesn't realize of the macros?)
#pragma clang diagnostic ignored "-Wunused-variable"
#endif

						// Precompute the 19 (3*9-8) intermediary points so children don't have to compute them several times:
						const mrpt::math::TPoint3Df p_Xm_Ym_Zm ( node.bb_min.x, node.bb_min.y, node.bb_min.z ); // 0
						const mrpt::math::TPoint3Df p_X0_Ym_Zm ( node.center.x, node.bb_min.y, node.bb_min.z );
						const mrpt::math::TPoint3Df p_Xp_Ym_Zm ( node.bb_max.x, node.bb_min.y, node.bb_min.z ); // 1
						const mrpt::math::TPoint3Df p_Xm_Y0_Zm ( node.bb_min.x, node.center.y, node.bb_min.z );
						const mrpt::math::TPoint3Df p_X0_Y0_Zm ( node.center.x, node.center.y, node.bb_min.z );
						const mrpt::math::TPoint3Df p_Xp_Y0_Zm ( node.bb_max.x, node.center.y, node.bb_min.z );
						const mrpt::math::TPoint3Df p_Xm_Yp_Zm ( node.bb_min.x, node.bb_max.y, node.bb_min.z ); // 2
						const mrpt::math::TPoint3Df p_X0_Yp_Zm ( node.center.x, node.bb_max.y, node.bb_min.z );
						const mrpt::math::TPoint3Df p_Xp_Yp_Zm ( node.bb_max.x, node.bb_max.y, node.bb_min.z ); // 3

						const mrpt::math::TPoint3Df p_Xm_Ym_Z0 ( node.bb_min.x, node.bb_min.y, node.center.z );
						const mrpt::math::TPoint3Df p_X0_Ym_Z0 ( node.center.x, node.bb_min.y, node.center.z );
						const mrpt::math::TPoint3Df p_Xp_Ym_Z0 ( node.bb_max.x, node.bb_min.y, node.center.z );
						const mrpt::math::TPoint3Df p_Xm_Y0_Z0 ( node.bb_min.x, node.center.y, node.center.z );
						const mrpt::math::TPoint3Df p_X0_Y0_Z0 ( node.center.x, node.center.y, node.center.z );
						const mrpt::math::TPoint3Df p_Xp_Y0_Z0 ( node.bb_max.x, node.center.y, node.center.z );
						const mrpt::math::TPoint3Df p_Xm_Yp_Z0 ( node.bb_min.x, node.bb_max.y, node.center.z );
						const mrpt::math::TPoint3Df p_X0_Yp_Z0 ( node.center.x, node.bb_max.y, node.center.z );
						const mrpt::math::TPoint3Df p_Xp_Yp_Z0 ( node.bb_max.x, node.bb_max.y, node.center.z );

						const mrpt::math::TPoint3Df p_Xm_Ym_Zp ( node.bb_min.x, node.bb_min.y, node.bb_max.z ); // 4
						const mrpt::math::TPoint3Df p_X0_Ym_Zp ( node.center.x, node.bb_min.y, node.bb_max.z );
						const mrpt::math::TPoint3Df p_Xp_Ym_Zp ( node.bb_min.x, node.bb_min.y, node.bb_max.z ); // 5
						const mrpt::math::TPoint3Df p_Xm_Y0_Zp ( node.bb_min.x, node.center.y, node.bb_max.z );
						const mrpt::math::TPoint3Df p_X0_Y0_Zp ( node.center.x, node.center.y, node.bb_max.z );
						const mrpt::math::TPoint3Df p_Xp_Y0_Zp ( node.bb_max.x, node.center.y, node.bb_max.z );
						const mrpt::math::TPoint3Df p_Xm_Yp_Zp ( node.bb_min.x, node.bb_max.y, node.bb_max.z ); // 6
						const mrpt::math::TPoint3Df p_X0_Yp_Zp ( node.center.x, node.bb_max.y, node.bb_max.z );
						const mrpt::math::TPoint3Df p_Xp_Yp_Zp ( node.bb_max.x, node.bb_max.y, node.bb_max.z ); // 7

						// Project all these points:
#define PROJ_SUB_NODE(POSTFIX) \
						mrpt::utils::TPixelCoordf px_##POSTFIX; \
						float        depth_##POSTFIX; \
						ri.projectPointPixels( p_##POSTFIX.x, p_##POSTFIX.y, p_##POSTFIX.z, px_##POSTFIX.x,px_##POSTFIX.y,depth_##POSTFIX);

#define PROJ_SUB_NODE_ALREADY_DONE(INDEX, POSTFIX) \
						const mrpt::utils::TPixelCoordf px_##POSTFIX = cr_px[INDEX]; \
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
							mrpt::utils::TPixelCoordf child_cr_px[8] = { px_##SEQ0,px_##SEQ1,px_##SEQ2,px_##SEQ3,px_##SEQ4,px_##SEQ5,px_##SEQ6,px_##SEQ7 }; \
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

#ifdef __clang__
#pragma clang diagnostic pop
#endif
					} // end "children_are_all_visible_for_sure"=false
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

				if (N<=mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE)
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
						const mrpt::math::TPoint3Df p = octree_derived().getPointf(i);
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

			/** Return the number of visible octree nodes in the last render event. */
			size_t octree_get_visible_nodes() const { return m_visible_octree_nodes; }

			/** Called from the derived class (or the user) to indicate we have/want to rebuild the entire node tree (for example, after modifying the point cloud or any global octree parameter) */
			inline void octree_mark_as_outdated() { m_octree_has_to_rebuild_all=true; }

			/** Returns a graphical representation of all the bounding boxes of the octree (leaf) nodes.
			  * \param[in] draw_solid_boxes If false, will draw solid boxes of color \a lines_color. Otherwise, wireframe boxes will be drawn.
			  */
			void octree_get_graphics_boundingboxes(
				mrpt::opengl::CSetOfObjects &gl_bb,
				const double lines_width = 1,
				const mrpt::utils::TColorf &lines_color = mrpt::utils::TColorf(1,1,1),
				const bool draw_solid_boxes = false ) const
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
					gl_box->setWireframe(!draw_solid_boxes);
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
