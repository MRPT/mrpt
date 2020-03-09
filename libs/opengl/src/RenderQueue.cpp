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
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/system/os.h>
#include <Eigen/Dense>
#include <map>
#include "opengl_internals.h"

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

			// Enqeue this object...
			const auto lst_shaders = obj->requiredShaders();
			const float depth = _.pmv_matrix(2, 3);
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

void gl_utils::checkOpenGLErr_impl(
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

void gl_utils::renderMessageBox(
	const float msg_x, const float msg_y, const float msg_w, const float msg_h,
	const std::string& text, float text_scale,
	const mrpt::img::TColor& back_col, const mrpt::img::TColor& border_col,
	const mrpt::img::TColor& text_col, const float border_width,
	const std::string& text_font, mrpt::opengl::TOpenGLFontStyle text_style,
	const double text_spacing, const double text_kerning)
{
	MRPT_TODO("Port or remove");
#if 0 && MRPT_HAS_OPENGL_GLUT
	const int nLines = 1 + std::count(text.begin(), text.end(), '\n');

	GLint win_dims[4];
	glGetIntegerv(GL_VIEWPORT, win_dims);
	const int w = win_dims[2];
	const int h = win_dims[3];

	const int min_wh = std::min(w, h);
	const float vw_w = w / float(min_wh);
	const float vw_h = h / float(min_wh);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glLoadIdentity();
	glOrtho(0, vw_w, 0, vw_h, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// The center of the message box:
	const float msg_x0 = vw_w * msg_x;
	const float msg_y0 = vw_h * msg_y;

	const float msg_x1 = vw_w * (msg_x + msg_w);
	const float msg_y1 = vw_h * (msg_y + msg_h);

	const float msg_real_w = msg_x1 - msg_x0;
	const float msg_real_h = msg_y1 - msg_y0;

	const float msg_cx = .5f * (msg_x0 + msg_x1);
	const float msg_cy = .5f * (msg_y0 + msg_y1);

	// Background:
	glColor4ub(back_col.R, back_col.G, back_col.B, back_col.A);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(msg_x0, msg_y0);
	glVertex2f(msg_x1, msg_y0);
	glVertex2f(msg_x1, msg_y1);
	glVertex2f(msg_x0, msg_y1);
	glEnd();

	// Border:
	glColor4ub(border_col.R, border_col.G, border_col.B, border_col.A);
	glLineWidth(border_width);
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINE_LOOP);
	glVertex2f(msg_x0, msg_y0);
	glVertex2f(msg_x1, msg_y0);
	glVertex2f(msg_x1, msg_y1);
	glVertex2f(msg_x0, msg_y1);
	glEnd();
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines

	// Draw text (centered):
	gl_utils::glSetFont(text_font);
	mrpt::img::TPixelCoordf txtSize =
		gl_utils::glGetExtends(text, text_scale, text_spacing, text_kerning);

	// Adjust text size if it doesn't fit into the box:
	if (txtSize.x > msg_real_w)
	{
		const float K = 0.99f * msg_real_w / txtSize.x;
		text_scale *= K;
		txtSize.x *= K;
		txtSize.y *= K;
	}
	if (txtSize.y > msg_real_h)
	{
		const float K = 0.99f * msg_real_h / txtSize.y;
		text_scale *= K;
		txtSize.x *= K;
		txtSize.y *= K;
	}

	const float text_w = txtSize.x;
	const float text_h =
		(nLines > 1 ? -(nLines - 1) * txtSize.y / float(nLines) : txtSize.y);
	const float text_x0 = msg_cx - .5f * text_w;
	const float text_y0 = msg_cy - .5f * text_h;

	glTranslatef(text_x0, text_y0, 0);
	glColor4ub(text_col.R, text_col.G, text_col.B, text_col.A);
	gl_utils::glDrawText(
		text, text_scale, text_style, text_spacing, text_kerning);

	// Restore gl flags:
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
#else
	MRPT_UNUSED_PARAM(msg_x);
	MRPT_UNUSED_PARAM(msg_y);
	MRPT_UNUSED_PARAM(msg_w);
	MRPT_UNUSED_PARAM(msg_h);
	MRPT_UNUSED_PARAM(text);
	MRPT_UNUSED_PARAM(text_scale);
	MRPT_UNUSED_PARAM(back_col);
	MRPT_UNUSED_PARAM(border_col);
	MRPT_UNUSED_PARAM(text_col);
	MRPT_UNUSED_PARAM(border_width);
	MRPT_UNUSED_PARAM(text_font);
	MRPT_UNUSED_PARAM(text_style);
	MRPT_UNUSED_PARAM(text_spacing);
	MRPT_UNUSED_PARAM(text_kerning);
#endif
}
