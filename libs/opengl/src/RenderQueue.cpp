/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/RenderQueue.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/system/os.h>
#include <Eigen/Dense>
#include <map>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::opengl;

// Render a set of objects
void mrpt::opengl::enqueForRendering(
	const mrpt::opengl::CListOpenGLObjects& objs,
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq)
{
#if MRPT_HAS_OPENGL_GLUT
	using mrpt::math::CMatrixDouble44;

	const char* curClassName = nullptr;
	try
	{
		for (const auto& objPtr : objs)
		{
			if (!objPtr) continue;
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

			// Make a copy of rendering state, so we always have the original
			// version of my parent intact.
			auto _ = state;

			// Compose relative to my parent pose:
			_.mv_matrix.asEigen() = _.mv_matrix.asEigen() * HM.asEigen();

			// Precompute pmv_matrix to be used in shaders:
			_.pmv_matrix.asEigen() =
				_.p_matrix.asEigen() * _.mv_matrix.asEigen();

			// Get a representative depth for this object (to sort objects from
			// eye-distance):
			mrpt::math::TPoint3Df lrp = obj->getLocalRepresentativePoint();

			Eigen::Vector4f lrp_hm(lrp.x, lrp.y, lrp.z, 1.0f);
			const auto lrp_proj = (_.pmv_matrix.asEigen() * lrp_hm).eval();
			const float depth =
				(lrp_proj(3) != 0) ? lrp_proj(2) / lrp_proj(3) : .001f;

			// Enqeue this object...
			const auto lst_shaders = obj->requiredShaders();
			for (const auto shader_id : lst_shaders)
			{
				// eye-to-object depth:
				rq[shader_id].emplace(depth, RenderQueueElement(obj, _));
			}

			// ...and its children:
			obj->enqueForRenderRecursive(_, rq);

			if (obj->isShowNameEnabled())
			{
				CText& label = obj->labelObject();

				// Update the label, only if it changed:
				if (label.getString() != obj->getName())
					label.setString(obj->getName());

				// Regenerate opengl vertex buffers, if first time or label
				// changed:
				if (label.hasToUpdateBuffers()) label.updateBuffers();

				rq[DefaultShaderID::TEXT].emplace(
					depth, RenderQueueElement(&label, _));
			}

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
#if MRPT_HAS_OPENGL_GLUT
	MRPT_PROFILE_FUNC_START

	for (const auto& rqSet : rq)
	{
		// bind the shader for this sequence of objects:
		mrpt::opengl::Program& shader = *shaders.at(rqSet.first);

		glUseProgram(shader.programId());
		CHECK_OPENGL_ERROR();

		// Process all objects using this shader:
		const auto& rqMap = rqSet.second;

		// Render in reverse depth order:
		for (auto it = rqMap.rbegin(); it != rqMap.rend(); ++it)
		{
			const RenderQueueElement& rqe = it->second;

			// Load matrices in shader:
			const GLint u_pmat = shader.uniformId("p_matrix");
			const GLint u_mvmat = shader.uniformId("mv_matrix");

			const auto IS_TRANSPOSED = GL_TRUE;

			glUniformMatrix4fv(
				u_pmat, 1, IS_TRANSPOSED, rqe.renderState.p_matrix.data());

			glUniformMatrix4fv(
				u_mvmat, 1, IS_TRANSPOSED, rqe.renderState.mv_matrix.data());

			CRenderizable::RenderContext rc;
			rc.shader = &shader;
			rc.shader_id = rqSet.first;
			rc.state = &rqe.renderState;
			rc.lights = &lights;

			// Render object:
			ASSERT_(rqe.object != nullptr);
			{
				rqe.object->render(rc);
				CHECK_OPENGL_ERROR();
			}
		}
	}

#endif
}

void mrpt::opengl::checkOpenGLErr_impl(
	unsigned int glErrorCode, const char* filename, int lineno)
{
#if MRPT_HAS_OPENGL_GLUT
	if (glErrorCode == GL_NO_ERROR) return;
	const std::string sErr = mrpt::format(
		"[%s:%i] OpenGL error: %s", filename, lineno,
		reinterpret_cast<const char*>(gluErrorString(glErrorCode)));
	std::cerr << "[gl_utils::checkOpenGLError] " << sErr << std::endl;
	THROW_EXCEPTION(sErr);
#endif
}
