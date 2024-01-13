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
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderWireFrame, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderWireFrame::~CRenderizableShaderWireFrame() = default;

void CRenderizableShaderWireFrame::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// Generate vertices & colors:
	const_cast<CRenderizableShaderWireFrame&>(*this)
		.onUpdateBuffers_Wireframe();

	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	// Define OpenGL buffers:
	m_vertexBuffer.createOnce();
	m_vertexBuffer.bind();
	m_vertexBuffer.allocate(
		m_vertex_buffer_data.data(),
		sizeof(m_vertex_buffer_data[0]) * m_vertex_buffer_data.size());

	// color buffer:
	m_colorBuffer.createOnce();
	m_colorBuffer.bind();
	m_colorBuffer.allocate(
		m_color_buffer_data.data(),
		sizeof(m_color_buffer_data[0]) * m_color_buffer_data.size());

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();
#endif
}

void CRenderizableShaderWireFrame::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	// TODO: Port thick lines to opengl3?
	// glLineWidth(m_lineWidth);

	// Skip these geometric entities when in the 1st pass of shadow map:
	if (rc.state->is1stShadowMapPass) return;

	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	// Set up the vertex array:
	std::optional<GLuint> attr_position;
	if (rc.shader->hasAttribute("position"))
	{
		attr_position = rc.shader->attributeId("position");
		m_vao.bind();
		glEnableVertexAttribArray(*attr_position);
		m_vertexBuffer.bind();
		glVertexAttribPointer(
			*attr_position, /* attribute */
			3, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			0, /* stride */
			BUFFER_OFFSET(0) /* array buffer offset */
		);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Set up the color array:
	std::optional<GLuint> attr_color;
	if (rc.shader->hasAttribute("vertexColor"))
	{
		attr_color = rc.shader->attributeId("vertexColor");
		glEnableVertexAttribArray(*attr_color);
		m_colorBuffer.bind();
		glVertexAttribPointer(
			*attr_color, /* attribute */
			4, /* size */
			GL_UNSIGNED_BYTE, /* type */
			GL_TRUE, /* normalized? */
			0, /* stride */
			BUFFER_OFFSET(0) /* array buffer offset */
		);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	glDrawArrays(GL_LINES, 0, m_vertex_buffer_data.size());
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (attr_position) glDisableVertexAttribArray(*attr_position);
	if (attr_color) glDisableVertexAttribArray(*attr_color);
	CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}

const mrpt::math::TBoundingBox
	CRenderizableShaderWireFrame::wireframeVerticesBoundingBox() const
{
	mrpt::math::TBoundingBox bb;
	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	if (m_vertex_buffer_data.empty()) return bb;

	bb.min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb.max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& p : m_vertex_buffer_data)
	{
		keep_min(bb.min.x, p.x);
		keep_max(bb.max.x, p.x);
		keep_min(bb.min.y, p.y);
		keep_max(bb.max.y, p.y);
		keep_min(bb.min.z, p.z);
		keep_max(bb.max.z, p.z);
	}
	return bb;
}
