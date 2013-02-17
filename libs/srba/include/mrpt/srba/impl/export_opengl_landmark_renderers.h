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
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/srba/landmark_render_models.h>

namespace mrpt { namespace srba {

// Specializations for rendering each kind of landmark:
template <class LM_TYPE>
struct LandmarkRendererBase { };

/** Landmark renderer for: landmark_rendering_as_point */
template <> struct LandmarkRendererBase<landmark_rendering_as_point>
{
	template <class RBA>
	static void render(
		const RBA &rba,
		const srba::TKeyFrameID root_keyframe,
		const typename RBA::frameid2pose_map_t  &spantree,
		const typename RBA::TOpenGLRepresentationOptions &options,
		mrpt::opengl::CSetOfObjects& scene)
	{
		using namespace mrpt::math;

		// For each fixed known LM, add a point to a point cloud
		//  and a text label with the landmark ID:
		mrpt::opengl::CPointCloudPtr  gl_lms = mrpt::opengl::CPointCloud::Create();
		gl_lms->setPointSize(5);
		gl_lms->setColor(0,0,1);

		scene.insert(gl_lms);

		vector<typename RBA::TRelativeLandmarkPosMap::const_iterator> lms_to_draw;
		vector<const typename RBA::hessian_traits_t::TSparseBlocksHessian_f::matrix_t *> lms_to_draw_inf_covs;

		for (typename RBA::TRelativeLandmarkPosMap::const_iterator itLM = rba.get_rba_state().known_lms.begin();itLM != rba.get_rba_state().known_lms.end();++itLM)
		{
			lms_to_draw.push_back(itLM);
			lms_to_draw_inf_covs.push_back( NULL );
		}

		const size_t nKnown = lms_to_draw.size();

		if (options.draw_unknown_feats)
		{
			for (typename RBA::TRelativeLandmarkPosMap::const_iterator itLM = rba.get_rba_state().unknown_lms.begin();itLM != rba.get_rba_state().unknown_lms.end();++itLM)
			{
				lms_to_draw.push_back(itLM);

				typename RBA::hessian_traits_t::landmarks2infmatrix_t::const_iterator it_inf = rba.get_rba_state().unknown_lms_inf_matrices.find(itLM->first);

				if (it_inf != rba.get_rba_state().unknown_lms_inf_matrices.end())
						lms_to_draw_inf_covs.push_back( &it_inf->second );
				else	lms_to_draw_inf_covs.push_back(NULL);
			}
		}

		const mrpt::utils::TColorf col_known_lms(1.f,1.f,0.f);
		const mrpt::utils::TColorf col_unknown_lms(1.f,0.f,0.f);

		for (size_t i=0;i<lms_to_draw.size();i++)
		{
			const typename RBA::TRelativeLandmarkPosMap::const_iterator &itLM = lms_to_draw[i];
			const bool is_known = (i<nKnown);

			// Don't draw those unknown LMs which hasn't been estimated not even once:
			if (!is_known && lms_to_draw_inf_covs[i]==NULL)
				continue;

			mrpt::poses::CPose3D base_pose;
			if (itLM->second.id_frame_base!=root_keyframe)
			{
				typename RBA::frameid2pose_map_t::const_iterator itBaseNode = spantree.find(itLM->second.id_frame_base);
				if(itBaseNode==spantree.end())
					continue;
				base_pose = itBaseNode->second.pose; // Inverse!
			}
			else
			{
				// It's the origin.
			}

			// If LM_TYPE defines this kind of renderer, we 
			TPoint3D p_wrt_base;
			RBA::lm_type::relativeEuclideanLocation(itLM->second.pos, p_wrt_base);

			TPoint3D p_global;
			base_pose.composePoint(p_wrt_base,p_global);

			gl_lms->insertPoint(p_global.x,p_global.y,p_global.z);

			// Add text label:
			mrpt::opengl::CText3DPtr  gl_txt = mrpt::opengl::CText3D::Create(
				mrpt::format("%u",static_cast<unsigned int>( itLM->first)),
				"mono", 0.15,
				mrpt::opengl::NICE );
			gl_txt->setPose(CPose3D(p_global.x,p_global.y,p_global.z,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(90)));
			gl_txt->setColor( is_known ? col_known_lms : col_unknown_lms );

			scene.insert(gl_txt);

			// Uncertainty ellipse?
			if (options.draw_unknown_feats_ellipses && lms_to_draw_inf_covs[i] )
			{
				double min_inf_diag=std::numeric_limits<double>::max();
				for (size_t k=0;k<RBA::LM_DIMS;k++)
					mrpt::utils::keep_min(min_inf_diag, (*lms_to_draw_inf_covs[i])(k,k));
				if (min_inf_diag<1e-5) continue; // Too large covariance!

				mrpt::math::CMatrixFixedNumeric<double,RBA::LM_DIMS,RBA::LM_DIMS> cov(mrpt::math::UNINITIALIZED_MATRIX);
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
				scene.insert(gl_ellip);
			}
		}
	}
};

/** Landmark renderer for: landmark_rendering_as_pose_constraints */
template <> struct LandmarkRendererBase<landmark_rendering_as_pose_constraints>
{
	template <class RBA>
	static void render(
		const RBA &rba,
		const srba::TKeyFrameID root_keyframe,
		const typename RBA::frameid2pose_map_t  &spantree,
		const typename RBA::TOpenGLRepresentationOptions &options,
		mrpt::opengl::CSetOfObjects& scene)
	{
		using namespace mrpt::math;

		mrpt::opengl::CSetOfLinesPtr gl_edges = mrpt::opengl::CSetOfLines::Create();
		gl_edges->setLineWidth(1);
		gl_edges->setColor(0,0,1);

		scene.insert(gl_edges);

		// For each KF: check all its "observations"
		for (typename RBA::frameid2pose_map_t::const_iterator it=spantree.begin();it!=spantree.end();++it)
		{
			const TKeyFrameID kf_id = it->first;
			const typename RBA::pose_flag_t & pf = it->second;

			const typename RBA::keyframe_info &kfi = rba.get_rba_state().keyframes[kf_id];
			
			for (size_t i=0;i<kfi.adjacent_k2f_edges.size();i++)
			{
				const typename RBA::k2f_edge_t * k2f = kfi.adjacent_k2f_edges[i];
				const TKeyFrameID other_kf_id = k2f->feat_rel_pos->id_frame_base;
				if (kf_id==other_kf_id)
					continue; // It's not an constraint with ANOTHER keyframe

				// Is the other KF in the spanning tree?
				typename RBA::frameid2pose_map_t::const_iterator other_it=spantree.find(other_kf_id);				
				if (other_it==spantree.end()) continue;

				const typename RBA::pose_flag_t & other_pf = other_it->second;
				
				// Add edge between the two KFs to represent the pose constraint:
				mrpt::poses::CPose3D p1 = mrpt::poses::CPose3D(pf.pose);  // Convert to 3D
				mrpt::poses::CPose3D p2 = mrpt::poses::CPose3D(other_pf.pose);
				
				gl_edges->appendLine( p1.x(),p1.y(),p1.z()+0.10, p2.x(),p2.y(),p2.z()+0.10 );
			}

		} // end for each KF
	}
};

/** Landmark renderer for: landmark_rendering_none */
template <> struct LandmarkRendererBase<landmark_rendering_none>
{
	template <class RBA>
	static void render(
		const RBA &rba,
		const srba::TKeyFrameID root_keyframe,
		const typename RBA::frameid2pose_map_t  &spantree,
		const typename RBA::TOpenGLRepresentationOptions &options,
		mrpt::opengl::CSetOfObjects& scene)
	{
		// Nothing to render
	}
};

} }  // end namespaces
