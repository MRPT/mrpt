/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
static std::tuple<mrpt::math::TPoint2Df, float> projectToScreenCoordsAndDepth(
	const mrpt::math::TPoint3Df& localPt,
	const mrpt::opengl::TRenderMatrices& objState)
{
	const Eigen::Vector4f lrp_hm(localPt.x, localPt.y, localPt.z, 1.0f);
	const auto lrp_proj = (objState.pmv_matrix.asEigen() * lrp_hm).eval();

	const float depth = (lrp_proj(3) != 0) ? lrp_proj(2) / lrp_proj(3) : .001f;

	const auto uv = (lrp_proj(3) != 0)
		? mrpt::math::TPoint2Df(
			  lrp_proj(0) / lrp_proj(3), lrp_proj(1) / lrp_proj(3))
		: mrpt::math::TPoint2Df(.001f, .001f);

	return {uv, depth};
}

std::tuple<double, bool> mrpt::opengl::depthAndVisibleInView(
	const CRenderizable* obj, const mrpt::opengl::TRenderMatrices& objState)
{
	// This profiler has a too-high impact. Only for very low-level debugging.
	/*#ifdef MRPT_OPENGL_PROFILER
		mrpt::system::CTimeLoggerEntry tle(
			opengl_profiler(), "depthAndVisibleInView");
	#endif*/

	// Get a representative depth for this object (to sort objects from
	// eye-distance):
	const mrpt::math::TPoint3Df lrp = obj->getLocalRepresentativePoint();

	const auto [lrpUV, depth] = projectToScreenCoordsAndDepth(lrp, objState);

	// If the object is outside of the camera frustrum, do not even send
	// it for rendering:
	// bbox is in local object frame of reference.
	/*#ifdef MRPT_OPENGL_PROFILER
		mrpt::system::CTimeLoggerEntry tle2(
			opengl_profiler(), "depthAndVisibleInView.bbox");
	#endif*/
	const auto bbox = obj->getBoundingBoxLocalf();
	/*#ifdef MRPT_OPENGL_PROFILER
		tle2.stop();
	#endif*/

	const mrpt::math::TPoint3Df ends[2] = {bbox.min, bbox.max};

	// for each of the 8 corners:
	bool visible = false;
	bool quadrants[9] = {false, false, false, false, false,
						 false, false, false, false};

	// dont go thru the (min,max) dimensions of one axis if it's negligible:
	const auto diag = bbox.max - bbox.min;
	const float maxLen = mrpt::max3(diag.x, diag.y, diag.z);
	int numLayersX = 1, numLayersY = 1, numLayersZ = 1;
	bool anyGoodDepth = false;

	if (maxLen > 0)
	{
		numLayersX = (diag.x / maxLen) > 1e-3f ? 2 : 1;
		numLayersY = (diag.y / maxLen) > 1e-3f ? 2 : 1;
		numLayersZ = (diag.z / maxLen) > 1e-3f ? 2 : 1;
	}

	for (int ix = 0; !visible && ix < numLayersX; ix++)
	{
		const float x = ends[ix].x;
		for (int iy = 0; !visible && iy < numLayersY; iy++)
		{
			const float y = ends[iy].y;
			for (int iz = 0; !visible && iz < numLayersZ; iz++)
			{
				const float z = ends[iz].z;

				auto [uv, bboxDepth] =
					projectToScreenCoordsAndDepth({x, y, z}, objState);

				const bool goodDepth = bboxDepth > -1.0f && bboxDepth < 1.0f;
				anyGoodDepth = anyGoodDepth | goodDepth;

				if (!goodDepth)	 // continue;
					uv *= -1.0;

				const bool inside = goodDepth &&
					(uv.x >= -1 && uv.x <= 1 && uv.y >= -1 && uv.y < 1);
				if (inside)
				{
					visible = true;
					break;
				}
				// quadrants:
				int qx = uv.x < -1 ? 0 : (uv.x > 1 ? 2 : 1);
				int qy = uv.y < -1 ? 0 : (uv.y > 1 ? 2 : 1);
				quadrants[qx + 3 * qy] = true;
			}
		}
	}

	// if we already had *any* bbox corner inside the frustrum, it's visible.
	// if not, *but* the corners are in opposed sides of the frustrum, then the
	// (central part of the) object may be still visible:
	if (!visible)
	{
		using Bools3x3_bits = uint32_t;
		constexpr Bools3x3_bits quadPairs[9] = {
			// clang-format off
			// 0:
			(0b011 << 6) |
			(0b011 << 3) |
			(0b000 << 0),
			// 1:
			(0b111 << 6) |
			(0b111 << 3) |
			(0b000 << 0),
			// 2:
			(0b110 << 6) |
			(0b110 << 3) |
			(0b000 << 0),
			// 3:
			(0b011 << 6) |
			(0b011 << 3) |
			(0b011 << 0),
			// 4:
			(0b111 << 6) |
			(0b101 << 3) |
			(0b111 << 0),
			// 5:
			(0b110 << 6) |
			(0b110 << 3) |
			(0b110 << 0),
			// 6:
			(0b000 << 6) |
			(0b011 << 3) |
			(0b011 << 0),
			// 7:
			(0b000 << 6) |
			(0b111 << 3) |
			(0b111 << 0),
			// 8:
			(0b000 << 6) |
			(0b110 << 3) |
			(0b110 << 0),
			// clang-format on
		};

		uint32_t q = 0;
		for (int i = 0; i < 9; i++)
		{
			if (quadrants[i]) q = q | (1 << i);
		}

		for (int i = 0; i < 9 && !visible; i++)
		{
			if (!quadrants[i]) continue;
			if ((q & quadPairs[i]) != 0)
			{
				// at least one match for a potential edge crossing the visible
				// area of the frustum:
				visible = true;
			}
		}
	}
	if (!anyGoodDepth) { visible = false; }

	return {depth, visible};
}

// Render a set of objects
void mrpt::opengl::enqueueForRendering(
	const mrpt::opengl::CListOpenGLObjects& objs,
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq,
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
			// Save class name: just in case we have an exception, for error
			// reporting:
			curClassName = obj->GetRuntimeClass()->className;

			// Regenerate opengl vertex buffers?
			if (obj->hasToUpdateBuffers()) obj->updateBuffers();

			if (!obj->isVisible()) continue;

			const CPose3D& thisPose = obj->getPoseRef();
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
			_.mv_matrix.asEigen() = _.mv_matrix.asEigen() * HM.asEigen();

			// Precompute pmv_matrix to be used in shaders:
			_.pmv_matrix.asEigen() =
				_.p_matrix.asEigen() * _.mv_matrix.asEigen();

			const auto [depth, withinView] = depthAndVisibleInView(obj, _);

			if (withinView)
			{
#ifdef MRPT_OPENGL_PROFILER
				if (stats) stats->numObjRendered++;
#endif

				// Enqeue this object...
				const auto lst_shaders = obj->requiredShaders();
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
			obj->enqueueForRenderRecursive(_, rq);

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
	const mrpt::opengl::TLightParameters& lights)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

#ifdef MRPT_OPENGL_PROFILER
	mrpt::system::CTimeLoggerEntry tle(opengl_profiler(), "processRenderQueue");
#endif

	for (const auto& rqSet : rq)
	{
		// bind the shader for this sequence of objects:
		mrpt::opengl::Program& shader = *shaders.at(rqSet.first);

		glUseProgram(shader.programId());
		CHECK_OPENGL_ERROR();

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

			// Load matrices in shader:
			const auto IS_TRANSPOSED = GL_TRUE;
			if (shader.hasUniform("p_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("p_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.p_matrix.data());

			if (shader.hasUniform("mv_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("mv_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.mv_matrix.data());

			if (shader.hasUniform("pmv_matrix"))
				glUniformMatrix4fv(
					shader.uniformId("pmv_matrix"), 1, IS_TRANSPOSED,
					rqe.renderState.pmv_matrix.data());

			rc.state = &rqe.renderState;

			// Render object:
			ASSERT_(rqe.object != nullptr);
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
