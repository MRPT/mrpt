/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#pragma once

#include <mrpt/opengl.h>

namespace mrpt { namespace srba {

template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::build_opengl_representation(
	const srba::TKeyFrameID root_keyframe,
	const TOpenGLRepresentationOptions &options,
	mrpt::opengl::CSetOfObjectsPtr out_scene,
	mrpt::opengl::CSetOfObjectsPtr out_root_tree
	) const
{
	using namespace std;
	using namespace mrpt::utils;
	using mrpt::poses::CPose3D;

	// Generate 3D scene:
	// ------------------------------
	if (out_scene)
	{
		out_scene->clear();

		// Change coordinate system, so "+Z" points parallel to the ground:
		out_scene->setPose(CPose3D(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90)));

		if (!rba_state.keyframes.empty())
		{
			// Use a spanning tree to estimate the global pose of every node
			//  starting (root) at the given keyframe:

			frameid2pose_map_t  spantree;
			create_complete_spanning_tree(root_keyframe,spantree, options.span_tree_max_depth );

			// For each camera frame, add a 3D corner:
			for (size_t kf_id=0;kf_id<rba_state.keyframes.size();++kf_id)
			{
				CPose3D p;

				if (root_keyframe!=kf_id)
				{
					typename frameid2pose_map_t::const_iterator itP = spantree.find(kf_id);
					if (itP != spantree.end())
						p = itP->second.pose;
				}


				mrpt::opengl::CSetOfObjectsPtr o = mrpt::opengl::stock_objects::CornerXYZSimple(0.2,2.0);
				o->setPose(p);
				out_scene->insert(o);
			}

			// Draw all edges between frames:
			mrpt::opengl::CSetOfLinesPtr gl_edges = mrpt::opengl::CSetOfLines::Create();
			for (typename rba_problem_state_t::k2k_edges_deque_t::const_iterator itEdge = rba_state.k2k_edges.begin();itEdge!=rba_state.k2k_edges.end();++itEdge)
			{
				CPose3D p1;
				if (itEdge->from!=root_keyframe)
				{
					typename frameid2pose_map_t::const_iterator itN1 = spantree.find(itEdge->from);
					if(itN1==spantree.end())
						continue;
					p1 = itN1->second.pose;
				}
				CPose3D p2;
				if (itEdge->to!=root_keyframe)
				{
					typename frameid2pose_map_t::const_iterator itN2 = spantree.find(itEdge->to);
					if(itN2==spantree.end())
						continue;
					p2 = itN2->second.pose;
				}
				gl_edges->appendLine(p1.x(),p1.y(),p1.z(), p2.x(),p2.y(),p2.z());
			}
			out_scene->insert(gl_edges);

			// For each fixed known LM, add a point to a point cloud
			//  and a text label with the landmark ID:
			mrpt::opengl::CPointCloudPtr  gl_lms = mrpt::opengl::CPointCloud::Create();
			gl_lms->setPointSize(5);
			gl_lms->setColor(0,0,1);

			out_scene->insert(gl_lms);

			vector<typename TRelativeLandmarkPosMap::const_iterator> lms_to_draw;
			vector<const typename hessian_traits_t::TSparseBlocksHessian_f::matrix_t *> lms_to_draw_inf_covs;

			for (typename TRelativeLandmarkPosMap::const_iterator itLM = rba_state.known_lms.begin();itLM != rba_state.known_lms.end();++itLM)
			{
				lms_to_draw.push_back(itLM);
				lms_to_draw_inf_covs.push_back( NULL );
			}

			const size_t nKnown = lms_to_draw.size();

			if (options.draw_unknown_feats)
			{
				for (typename TRelativeLandmarkPosMap::const_iterator itLM = rba_state.unknown_lms.begin();itLM != rba_state.unknown_lms.end();++itLM)
				{
					lms_to_draw.push_back(itLM);

					typename hessian_traits_t::landmarks2infmatrix_t::const_iterator it_inf = rba_state.unknown_lms_inf_matrices.find(itLM->first);

					if (it_inf != rba_state.unknown_lms_inf_matrices.end())
							lms_to_draw_inf_covs.push_back( &it_inf->second );
					else	lms_to_draw_inf_covs.push_back(NULL);
				}
			}

			const mrpt::utils::TColorf col_known_lms(1.f,1.f,0.f);
			const mrpt::utils::TColorf col_unknown_lms(1.f,0.f,0.f);

			for (size_t i=0;i<lms_to_draw.size();i++)
			{
				const typename TRelativeLandmarkPosMap::const_iterator &itLM = lms_to_draw[i];
				const bool is_known = (i<nKnown);

				// Don't draw those unknown LMs which hasn't been estimated not even once:
				if (!is_known && lms_to_draw_inf_covs[i]==NULL)
					continue;

				CPose3D base_pose;
				if (itLM->second.id_frame_base!=root_keyframe)
				{
					typename frameid2pose_map_t::const_iterator itBaseNode = spantree.find(itLM->second.id_frame_base);
					if(itBaseNode==spantree.end())
						continue;
					base_pose = itBaseNode->second.pose; // Inverse!
				}
				else
				{
					// It's the origin.
				}

				const TPoint3D p_wrt_base = TPoint3D( itLM->second.getAsRelativeEuclideanLocation() );

				TPoint3D p_global;
				base_pose.composePoint(p_wrt_base,p_global);

				gl_lms->insertPoint(p_global.x,p_global.y,p_global.z);

				// Add text label:
				mrpt::opengl::CText3DPtr  gl_txt = mrpt::opengl::CText3D::Create(
					mrpt::format("%u",static_cast<unsigned int>( itLM->first)),
					"mono", 0.15,
					mrpt::opengl::NICE );
				gl_txt->setPose(CPose3D(p_global.x,p_global.y,p_global.z,DEG2RAD(0),DEG2RAD(0),DEG2RAD(180)));
				gl_txt->setColor( is_known ? col_known_lms : col_unknown_lms );

				out_scene->insert(gl_txt);

				// Uncertainty ellipse?
				if (options.draw_unknown_feats_ellipses && lms_to_draw_inf_covs[i] )
				{
					mrpt::math::CMatrixFixedNumeric<double,LM_DIMS,LM_DIMS> cov;
					lms_to_draw_inf_covs[i]->inv(cov);

					mrpt::opengl::CEllipsoidPtr gl_ellip = mrpt::opengl::CEllipsoid::Create();
					gl_ellip->setQuantiles( options.draw_unknown_feats_ellipses_quantiles );
					gl_ellip->enableDrawSolid3D(false);

					gl_ellip->setCovMatrix(cov);
					CPose3D ellip_pose = base_pose;
					ellip_pose.x(p_global.x);
					ellip_pose.y(p_global.y);
					ellip_pose.z(p_global.z);

					gl_ellip->setPose(ellip_pose);
					out_scene->insert(gl_ellip);
				}
			}
		} // end if graph is not empty

	} // end of "out_scene"


	// Generate 2D tree:
	// ------------------------------
	if (out_root_tree)
	{
		out_root_tree->clear();

		const float NODE_RADIUS = 1;
		const float ROW_HEIGHT  = 4*NODE_RADIUS;
		const float COL_WIDTH   = 3*NODE_RADIUS;

		typename rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_st_root = rba_state.spanning_tree.sym.next_edge.find(root_keyframe);
		ASSERT_(it_st_root != rba_state.spanning_tree.sym.next_edge.end())

		const std::map<TKeyFrameID,TSpanTreeEntry> & st_root = it_st_root->second;

		std::map<TKeyFrameID,topo_dist_t>  children_depths;
		std::map<topo_dist_t,std::vector<TKeyFrameID> > children_by_depth;

		// roots are not in spanning trees ("spanning_tree.sym.next_edge")
		children_depths[root_keyframe]=0;
		children_by_depth[0].push_back(root_keyframe);

		// Go thru the tree to realize of its size:
		//size_t max_nodes_per_level = 1;
		size_t max_depth = 0;
		for (std::map<TKeyFrameID,TSpanTreeEntry>::const_iterator it=st_root.begin();it!=st_root.end();++it)
		{
			const topo_dist_t depth = it->second.distance;
			children_depths[it->first] = depth;
			mrpt::utils::keep_max(max_depth, depth);

			std::vector<TKeyFrameID> & v = children_by_depth[depth];
			v.push_back(it->first);
			//mrpt::utils::keep_max(max_nodes_per_level, v.size());
		}

		// Generate (x,y) coords for each node:
		std::map<TKeyFrameID, mrpt::math::TPoint3Df> node_coords;

		for (size_t d=0;d<=max_depth;d++)
		{
			const float y = -(d*ROW_HEIGHT);

			std::vector<TKeyFrameID> & v = children_by_depth[d];
		 	const float row_width = v.size()*COL_WIDTH;

			for (size_t i=0;i<v.size();i++)
			{
				const float x = - 0.5f*row_width + COL_WIDTH*i;
				node_coords[v[i]] = mrpt::math::TPoint3Df(x,y,0);
			}
		}

		// Draw edges in tree:
		mrpt::opengl::CSetOfLinesPtr gl_edges = mrpt::opengl::CSetOfLines::Create();
		gl_edges->setLineWidth(1.5);
		gl_edges->setColor_u8(mrpt::utils::TColor(0xff,0xff,0x00));
		out_root_tree->insert(gl_edges);

		typename rba_problem_state_t::TSpanningTree::all_edges_maps_t::const_iterator it_edges_from_root = rba_state.spanning_tree.sym.all_edges.find(root_keyframe);
		ASSERT_(it_edges_from_root != rba_state.spanning_tree.sym.all_edges.end())

		const std::map<TKeyFrameID, typename rba_problem_state_t::k2k_edge_vector_t> edges_from_root = it_edges_from_root->second;

		for (typename std::map<TKeyFrameID, typename rba_problem_state_t::k2k_edge_vector_t>::const_iterator it=edges_from_root.begin();it!=edges_from_root.end();++it)
		{
			const typename rba_problem_state_t::k2k_edge_vector_t & edges_to_j = it->second;
			for (size_t k=0;k<edges_to_j.size();k++)
			{
				const TKeyFrameID id1 = edges_to_j[k]->from;
				const TKeyFrameID id2 = edges_to_j[k]->to;

				gl_edges->appendLine(node_coords[id1], node_coords[id2]);
			}
		}

		// And on the top, draw the nodes:
		for (std::map<TKeyFrameID, mrpt::math::TPoint3Df>::const_iterator it=node_coords.begin();it!=node_coords.end();++it)
			gl_aux_draw_node(*out_root_tree, mrpt::format("%u", static_cast<unsigned int>(it->first) ), it->second.x, it->second.y);

	} // end render "2D tree"

}


template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
void RBA_Problem<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::gl_aux_draw_node(mrpt::opengl::CSetOfObjects &soo, const std::string &label, const float x, const float y) const
{
	{
		mrpt::opengl::CDiskPtr obj = mrpt::opengl::CDisk::Create();
		obj->setDiskRadius(1,0);
		obj->setColor_u8(mrpt::utils::TColor(0x00,0x00,0x00, 0xa0));
		obj->setLocation(x,y,0);
		soo.insert(obj);
	}

	{
		mrpt::opengl::CText3DPtr obj = mrpt::opengl::CText3D::Create(label);
		obj->setFont("sans");
		obj->setColor_u8(mrpt::utils::TColor(0xff,0xff,0xff));
		obj->setScale(0.9);
		obj->setLocation(x-0.5,y-0.5,0);
		soo.insert(obj);
	}

}

} }  // end namespaces
