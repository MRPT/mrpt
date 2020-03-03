/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CRenderizableShaderText.h>
#include <mrpt/opengl/Shader.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderText, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderText::~CRenderizableShaderText() = default;

void CRenderizableShaderText::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT
	// Generate vertices & colors:
	const_cast<CRenderizableShaderText&>(*this).onUpdateBuffers_Text();

	// ======== LINES ========
	// Define OpenGL buffers:
	m_linesVertexBuffer.createOnce();
	m_linesVertexBuffer.bind();
	m_linesVertexBuffer.allocate(
		m_vertex_buffer_data.data(),
		sizeof(m_vertex_buffer_data[0]) * m_vertex_buffer_data.size());

	// color buffer:
	m_linesColorBuffer.createOnce();
	m_linesColorBuffer.bind();
	m_linesColorBuffer.allocate(
		m_color_buffer_data.data(),
		sizeof(m_color_buffer_data[0]) * m_color_buffer_data.size());

	// ======== TRIANGLES ========
	const auto n = m_triangles.size();

	// Define OpenGL buffers:
	m_trianglesBuffer.createOnce();
	m_trianglesBuffer.bind();
	m_trianglesBuffer.allocate(m_triangles.data(), sizeof(m_triangles[0]) * n);

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();

#endif
}

void CRenderizableShaderText::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_LINE_SMOOTH);
	CHECK_OPENGL_ERROR();

	// === LINES ===
	// Set up the vertex array:
	const GLuint attr_position = rc.shader->attributeId("position");
	m_vao.bind();
	glEnableVertexAttribArray(attr_position);
	m_linesVertexBuffer.bind();
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
	m_linesColorBuffer.bind();
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

	// === TRIANGLES ===
	m_trianglesBuffer.bind();
	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(TTriangle::PointNormal), /* stride */
		BUFFER_OFFSET(offsetof(TTriangle::PointNormal, position.pt.x)));
	CHECK_OPENGL_ERROR();

	// Set up the color array:
	m_trianglesBuffer.bind();
	glVertexAttribPointer(
		attr_color, /* attribute */
		4, /* size */
		GL_UNSIGNED_BYTE, /* type */
		GL_TRUE, /* normalized? */
		sizeof(TTriangle::PointNormal), /* stride */
		BUFFER_OFFSET(offsetof(TTriangle::PointNormal, position.r)));
	CHECK_OPENGL_ERROR();

	// normals array: not used to render text

	// Draw:
	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);
	CHECK_OPENGL_ERROR();
#endif
}
