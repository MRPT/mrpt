/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <Eigen/Dense>	// First! to avoid conflicts with X.h
//
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/system/os.h>

#include <map>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::opengl;

// (U,V) normalized screen coordinates:
//
// +----------------------+
// |(-1,+1)        (+1,+1)|
// |                      |
// |                      |
// |                      |
// |(-1,-1)        (+1,-1)|
// +----------------------+
//
namespace
{
std::tuple<mrpt::math::TPoint2Df, float> projectToScreenCoordsAndDepth(
	const mrpt::math::TPoint3Df& localPt,
	const mrpt::opengl::TRenderMatrices& objState)
{
	const Eigen::Vector4f lrp_hm(localPt.x, localPt.y, localPt.z, 1.0f);

	// Looking from the light (1st pass in a shadow mapping), or the camera
	// (regular case):
	const auto& pmv_matrix =
		objState.is1stShadowMapPass ? objState.light_pmv : objState.pmv_matrix;

	const auto lrp_proj = (pmv_matrix.asEigen() * lrp_hm).eval();

	const float depth =
		(lrp_proj(3) != 0) ? lrp_proj(2) / std::abs(lrp_proj(3)) : .001f;

	auto uv = (lrp_proj(3) != 0)
		? mrpt::math::TPoint2Df(
			  lrp_proj(0) / lrp_proj(3), lrp_proj(1) / lrp_proj(3))
		: mrpt::math::TPoint2Df(.001f, .001f);

	if (!objState.is1stShadowMapPass)
	{
		if (depth < -1.0f && objState.is_projective) uv *= -1.0;
	}

	return {uv, depth};
}
}  // namespace

// See docs in .h
std::tuple<double, bool, bool> mrpt::opengl::depthAndVisibleInView(
	const CRenderizable* obj, const mrpt::opengl::TRenderMatrices& objState,
	const bool skipCullChecks)
{
	// This profiler has a too-high impact. Only for very low-level debugging.
	/*#ifdef MRPT_OPENGL_PROFILER
		mrpt::system::CTimeLoggerEntry tle(
			opengl_profiler(), "depthAndVisibleInView");
	#endif*/

	const bool mustCheckWholeBbox = obj->isCompositeObject();

	// Get a representative depth for this object (to sort objects from
	// eye-distance):
	const mrpt::math::TPoint3Df lrp = obj->getLocalRepresentativePoint();

	const auto [lrpUV, lrpDepth] = projectToScreenCoordsAndDepth(lrp, objState);

	if (skipCullChecks || !obj->cullElegible())
	{
		// direct return:
		return {lrpDepth, true, true};
	}

	// If the object is outside of the camera frustrum, do not even send
	// it for rendering:
	// bbox is in local object frame of reference.

	/*#ifdef MRPT_OPENGL_PROFILER
		mrpt::system::CTimeLoggerEntry tle2(
			opengl_profiler(), "depthAndVisibleInView.bbox");
	#endif*/

	mrpt::math::TBoundingBoxf bbox;
	try
	{
		bbox = obj->getBoundingBoxLocalf();
	}
	catch (const std::exception&)
	{
		// use default (0,0,0) bbox
	}

	/*#ifdef MRPT_OPENGL_PROFILER
		tle2.stop();
	#endif*/

	// for each of the 8 corners:
	bool anyVisible = false, allVisible = true;
	bool quadrants[9] = {false, false, false, false, false,
						 false, false, false, false};

	bool anyGoodDepth = false;

	const auto lambdaProcessSample =
		[&anyVisible, &allVisible, &anyGoodDepth, &quadrants](
			const mrpt::math::TPoint2Df& uv, const float sampleDepth)
	{
		// Do not check for "bboxDepth < 1.0f" since that may only mean
		// the object is still visible, but farther away than the
		// farPlane:
		const bool goodDepth = sampleDepth > -1.0f;
		anyGoodDepth = anyGoodDepth | goodDepth;

		const bool inside =
			goodDepth && (uv.x >= -1 && uv.x <= 1 && uv.y >= -1 && uv.y < 1);

		if (inside) anyVisible = true;
		else
			allVisible = false;

		// quadrants:
		int qx = uv.x < -1 ? 0 : (uv.x > 1 ? 2 : 1);
		int qy = uv.y < -1 ? 0 : (uv.y > 1 ? 2 : 1);
		quadrants[qx + 3 * qy] = true;
	};

	// 1st) Process the local-representative-point (~body center) sample:
	// -----------------------------------------------------------------------
	lambdaProcessSample(lrpUV, lrpDepth);

	// 2nd) then, the rest of 4 or 8 bbox corners:
	// ------------------------------------------------
	// dont go thru the (min,max) dimensions of one axis if it's negligible:
	const auto diag = bbox.max - bbox.min;
	const float maxLen = mrpt::max3(diag.x, diag.y, diag.z);
	int numLayersX = 1, numLayersY = 1, numLayersZ = 1;
	if (maxLen > 0)
	{
		numLayersX = (diag.x / maxLen) > 1e-3f ? 2 : 1;
		numLayersY = (diag.y / maxLen) > 1e-3f ? 2 : 1;
		numLayersZ = (diag.z / maxLen) > 1e-3f ? 2 : 1;
	}
	const mrpt::math::TPoint3Df ends[2] = {bbox.min, bbox.max};

	for (int ix = 0; (mustCheckWholeBbox || !anyVisible) && ix < numLayersX;
		 ix++)
	{
		const float x = ends[ix].x;
		for (int iy = 0; (mustCheckWholeBbox || !anyVisible) && iy < numLayersY;
			 iy++)
		{
			const float y = ends[iy].y;
			for (int iz = 0;
				 (mustCheckWholeBbox || !anyVisible) && iz < numLayersZ; iz++)
			{
				const float z = ends[iz].z;

				const auto [uv, bboxDepth] =
					projectToScreenCoordsAndDepth({x, y, z}, objState);

				lambdaProcessSample(uv, bboxDepth);
			}
		}
	}

	// if we already had *any* bbox corner inside the frustrum, it's visible.
	// if not, *but* the corners are in opposed sides of the frustrum, then the
	// (central part of the) object may be still visible:
	if (!anyVisible)
	{
		using Bools3x3_bits = uint32_t;
		constexpr Bools3x3_bits quadPairs[9] = {
			// clang-format off
			// 0:
			(0b110 << 6) |
			(0b110 << 3) |
			(0b000 << 0),   // 0b001 <= I'm here
			// 1:
			(0b111 << 6) |
			(0b111 << 3) |
			(0b000 << 0),   // 0b010 <= I'm here
			// 2:
			(0b011 << 6) |
			(0b011 << 3) |
			(0b000 << 0),   // 0b100 <= I'm here
			// 3:
			(0b110 << 6) |
			(0b110 << 3) |  // 0b001 <= I'm here
			(0b110 << 0),
			// 4:
			(0b111 << 6) |
			(0b101 << 3) |  // 0b010 <= I'm here
			(0b111 << 0),
			// 5:
			(0b011 << 6) |
			(0b011 << 3) |  // 0b100 <= I'm here
			(0b011 << 0),
			// 6:
			(0b000 << 6) |  // 0b001 <= I'm here
			(0b110 << 3) |
			(0b110 << 0),
			// 7:
			(0b000 << 6) |  // 0b010 <= I'm here
			(0b111 << 3) |
			(0b111 << 0),
			// 8:
			(0b000 << 6) |  // 0b100 <= I'm here
			(0b011 << 3) |
			(0b011 << 0),
			// clang-format on
		};

		uint32_t q = 0;
		for (int i = 0; i < 9; i++)
		{
			if (quadrants[i]) q = q | (1 << i);
		}

		for (int i = 0; i < 9 && !anyVisible; i++)
		{
			if (!quadrants[i]) continue;
			if ((q & quadPairs[i]) != 0)
			{
				// at least one match for a potential edge crossing the visible
				// area of the frustum:
				anyVisible = true;
			}
		}
	}
	if (!anyGoodDepth) { anyVisible = false; }

	return {lrpDepth, anyVisible, allVisible};
}

// Render a set of objects
void mrpt::opengl::enqueueForRendering(
	const mrpt::opengl::CListOpenGLObjects& objs,
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq,
	const bool skipCullChecks, const bool is1stShadowMapPass,
	RenderQueueStats* stats)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	using mrpt::math::CMatrixDouble44;

#ifdef MRPT_OPENGL_PROFILER
	mrpt::system::CTimeLoggerEntry tle(
		opengl_profiler(), "enqueueForRendering");
#endif

	const char* curClassName = nullptr;
	try
	{
		for (const auto& objPtr : objs)
		{
			if (!objPtr) continue;

#ifdef MRPT_OPENGL_PROFILER
			if (stats) stats->numObjTotal++;
#endif

			// Use plain pointers, faster than smart pointers:
			const CRenderizable* obj = objPtr.get();

			// Quick check: if we are in a shadow generation pass where this
			// object does not render, skip it:
			const auto lst_shaders = obj->requiredShaders();
			if (lst_shaders.size() == 1 &&
				lst_shaders.at(0) == DefaultShaderID::NONE)
				continue;

			// Save class name: just in case we have an exception, for error
			// reporting:
			curClassName = obj->GetRuntimeClass()->className;

			if (!obj->isVisible()) continue;

			// Skip objects that do not cast shadows, if we are in that first
			// shadow map pass.
			if (is1stShadowMapPass && !obj->castShadows()) continue;

			// Regenerate opengl vertex buffers?
			if (obj->hasToUpdateBuffers()) obj->updateBuffers();

			const CPose3D thisPose = obj->getCPose();
			CMatrixFloat44 HM =
				thisPose.getHomogeneousMatrixVal<CMatrixDouble44>()
					.cast_float();

			// Scaling:
			if (obj->getScaleX() != 1 || obj->getScaleY() != 1 ||
				obj->getScaleZ() != 1)
			{
				auto scale = CMatrixFloat44::Identity();
				scale(0, 0) = obj->getScaleX();
				scale(1, 1) = obj->getScaleY();
				scale(2, 2) = obj->getScaleZ();
				HM.asEigen() = HM.asEigen() * scale.asEigen();
			}

			// Make a copy of rendering state, so we always have the
			// original version of my parent intact.
			auto _ = state;

			// Compose relative to my parent pose:
			_.m_matrix.asEigen() = _.m_matrix.asEigen() * HM.asEigen();

			// Precompute matrices to be used in shaders:
			_.mv_matrix.asEigen() = _.v_matrix.asEigen() * _.m_matrix.asEigen();

			// PMV = P*V*M (observe the weird notation order)
			_.pmv_matrix.asEigen() =
				_.p_matrix.asEigen() * _.mv_matrix.asEigen();

			// PMV for the light = P*V*M (observe the weird notation order)
			_.light_pmv.asEigen() = _.light_pv.asEigen() * _.m_matrix.asEigen();

			// Let the culling algorithm know whether we are looking from the
			// camera or from the light:
			_.is1stShadowMapPass = is1stShadowMapPass;

			const auto [depth, withinView, wholeInView] =
				depthAndVisibleInView(obj, _, skipCullChecks);

			if (withinView)
			{
#ifdef MRPT_OPENGL_PROFILER
				if (stats) stats->numObjRendered++;
#endif

				// Enqueue this object...
				for (const auto shader_id : lst_shaders)
				{
					// eye-to-object depth:
					rq[shader_id].emplace(depth, RenderQueueElement(obj, _));
				}

				if (obj->isShowNameEnabled())
				{
					CText& label = obj->labelObject();

					// Update the label, only if it changed:
					if (label.getString() != obj->getName())
						label.setString(obj->getName());

					// Regenerate opengl vertex buffers, if first time or
					// label changed:
					if (label.hasToUpdateBuffers()) label.updateBuffers();

					rq[DefaultShaderID::TEXT].emplace(
						depth, RenderQueueElement(&label, _));
				}
			}

			// ...and its children:
			obj->enqueueForRenderRecursive(
				_, rq, wholeInView, is1stShadowMapPass);

		}  // end foreach object
	}
	catch (const exception& e)
	{
		THROW_EXCEPTION_FMT(
			"Exception while rendering class '%s':\n%s",
			curClassName ? curClassName : "(undefined)", e.what());
	}

#endif
}

void mrpt::opengl::processRenderQueue(
	const RenderQueue& rq,
	std::map<shader_id_t, mrpt::opengl::Program::Ptr>& shaders,
	const mrpt::opengl::TLightParameters& lights,
	const std::optional<unsigned int>& depthMapTextureId)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

#ifdef MRPT_OPENGL_PROFILER
	mrpt::system::CTimeLoggerEntry tle(opengl_profiler(), "processRenderQueue");
#endif

	for (const auto& rqSet : rq)
	{
		// bind the shader for this sequence of objects:
		mrpt::opengl::Program& shader = *shaders.at(rqSet.first);

		shader.use();

		if (depthMapTextureId)
		{
			// We are in the 2nd pass of shadow rendering:
			// bind depthmap texture
			glActiveTexture(GL_TEXTURE0 + SHADOW_MAP_TEXTURE_UNIT);
			glBindTexture(GL_TEXTURE_2D, *depthMapTextureId);
			CHECK_OPENGL_ERROR_IN_DEBUG();
		}

		CRenderizable::RenderContext rc;
		rc.shader = &shader;
		rc.shader_id = rqSet.first;
		rc.lights = &lights;

		// Process all objects using this shader:
		const auto& rqMap = rqSet.second;

		// Render in reverse depth order:
		for (auto it = rqMap.rbegin(); it != rqMap.rend(); ++it)
		{
			const RenderQueueElement& rqe = it->second;
			ASSERT_(rqe.object != nullptr);

			// Load matrices in shader:
			const auto IS_TRANSPOSED = GL_TRUE;
			if (shader.hasUniform("m_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("m_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.m_matrix.data());

			if (shader.hasUniform("p_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("p_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.p_matrix.data());

			if (shader.hasUniform("v_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("v_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.v_matrix.data());

			if (shader.hasUniform("v_matrix_no_translation"))
				glUniformMatrix4fv(
					shader.uniformId("v_matrix_no_translation"), 1,
					IS_TRANSPOSED,
					rqe.renderState.v_matrix_no_translation.data());

			if (shader.hasUniform("mv_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("mv_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.mv_matrix.data());

			if (shader.hasUniform("pmv_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("pmv_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.pmv_matrix.data());

			if (shader.hasUniform("cam_position"))
				glUniform3f(
					shader.uniformId("cam_position"), rqe.renderState.eye.x,
					rqe.renderState.eye.y, rqe.renderState.eye.z);

			if (shader.hasUniform("light_pv_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("light_pv_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.light_pv.data());

			if (shader.hasUniform("materialSpecular"))
				glUniform1f(
					shader.uniformId("materialSpecular"),
					rqe.object->materialShininess());

			rc.state = &rqe.renderState;

			// Render object:
			rqe.object->render(rc);
		}
	}

#endif
}

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
void mrpt::opengl::checkOpenGLErr_impl(
	unsigned int glErrorCode, const char* filename, int lineno)
{
	if (glErrorCode == GL_NO_ERROR) return;
#if MRPT_HAS_OPENGL_GLUT
	const std::string sErr = mrpt::format(
		"[%s:%i] OpenGL error: %s", filename, lineno,
		reinterpret_cast<const char*>(gluErrorString(glErrorCode)));
#else
	// w/o glu:
	const std::string sErr =
		mrpt::format("[%s:%i] OpenGL error: %u", filename, lineno, glErrorCode);
#endif
	std::cerr << "[gl_utils::checkOpenGLError] " << sErr << std::endl;
	THROW_EXCEPTION(sErr);
}
#endif
