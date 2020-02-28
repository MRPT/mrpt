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
	m_vertexBuffer = make_buffer(
		GL_ARRAY_BUFFER, m_vertex_buffer_data.data(),
		sizeof(m_vertex_buffer_data[0]) * m_vertex_buffer_data.size());

	// Generate a name for a new array.
	glGenVertexArrays(1, &m_vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(m_vao);

	// color buffer:
	m_colorBuffer = make_buffer(
		GL_ARRAY_BUFFER, m_color_buffer_data.data(),
		sizeof(m_color_buffer_data[0]) * m_color_buffer_data.size());
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
	glEnableVertexAttribArray(attr_position);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
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
	glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
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
