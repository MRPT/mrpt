/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/Shader.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderTriangles, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderTriangles::~CRenderizableShaderTriangles() = default;

void CRenderizableShaderTriangles::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT
	// Generate vertices & colors into m_triangles
	const_cast<CRenderizableShaderTriangles&>(*this)
		.onUpdateBuffers_Triangles();

	// Eval normals:
	const auto n = m_triangles.size();

	// Define OpenGL buffers:
	m_trianglesBuffer = mrpt::opengl::make_buffer(
		GL_ARRAY_BUFFER, m_triangles.data(), sizeof(m_triangles[0]) * n);

	// Generate a name for a new array.
	glGenVertexArrays(1, &m_vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(m_vao);
#endif
}

void CRenderizableShaderTriangles::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT

	// Set up the vertex array:
	const GLuint attr_position = rc.shader->attributeId("position");
	glEnableVertexAttribArray(attr_position);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(TTriangle::PointNormal), /* stride */
		BUFFER_OFFSET(offsetof(TTriangle::PointNormal, position.pt.x)));
	CHECK_OPENGL_ERROR();

	// Set up the color array:
	const GLuint attr_color = rc.shader->attributeId("vertexColor");
	glEnableVertexAttribArray(attr_color);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_color, /* attribute */
		4, /* size */
		GL_UNSIGNED_BYTE, /* type */
		GL_TRUE, /* normalized? */
		sizeof(TTriangle::PointNormal), /* stride */
		BUFFER_OFFSET(offsetof(TTriangle::PointNormal, position.r)));
	CHECK_OPENGL_ERROR();

	// Set up the normals array:
	const GLuint attr_normals = rc.shader->attributeId("vertexNormal");
	glEnableVertexAttribArray(attr_normals);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_normals, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(TTriangle::PointNormal), /* stride */
		BUFFER_OFFSET(offsetof(TTriangle::PointNormal, normal.x)));
	CHECK_OPENGL_ERROR();

	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);
	glDisableVertexAttribArray(attr_normals);

#endif
}
