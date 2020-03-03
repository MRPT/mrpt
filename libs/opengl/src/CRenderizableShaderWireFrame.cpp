/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/Shader.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderWireFrame, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderWireFrame::~CRenderizableShaderWireFrame() = default;

void CRenderizableShaderWireFrame::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Generate vertices & colors:
	const_cast<CRenderizableShaderWireFrame&>(*this)
		.onUpdateBuffers_Wireframe();

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
#if MRPT_HAS_OPENGL_GLUT
	// TODO: Port thick lines to opengl3?
	// glLineWidth(m_lineWidth);

	glEnable(GL_LINE_SMOOTH);
	CHECK_OPENGL_ERROR();

	// Set up the vertex array:
	const GLuint attr_position = rc.shader->attributeId("position");
	m_vao.bind();
	glEnableVertexAttribArray(attr_position);
	m_vertexBuffer.bind();
	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		0, /* stride */
		BUFFER_OFFSET(0) /* array buffer offset */
	);
	CHECK_OPENGL_ERROR();

	// Set up the color array:
	const GLuint attr_color = rc.shader->attributeId("vertexColor");
	glEnableVertexAttribArray(attr_color);
	m_colorBuffer.bind();
	glVertexAttribPointer(
		attr_color, /* attribute */
		4, /* size */
		GL_UNSIGNED_BYTE, /* type */
		GL_TRUE, /* normalized? */
		0, /* stride */
		BUFFER_OFFSET(0) /* array buffer offset */
	);
	CHECK_OPENGL_ERROR();

	glDrawArrays(GL_LINES, 0, m_vertex_buffer_data.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);
	CHECK_OPENGL_ERROR();
#endif
}
