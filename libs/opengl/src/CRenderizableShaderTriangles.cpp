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
	// Vertices and colors are already stored into m_triangles.

	// Eval normals:
	const auto n = m_triangles.size();
	m_trianglesNormals.resize(3 * n);  // normal per vertex
	for (size_t i = 0; i < n; i++)
	{
		const auto& t = m_triangles[i];
		const float ax = t.x(1) - t.x(0);
		const float ay = t.y(1) - t.y(0);
		const float az = t.z(1) - t.z(0);
		const float bx = t.x(2) - t.x(0);
		const float by = t.y(2) - t.y(0);
		const float bz = t.z(2) - t.z(0);

		mrpt::math::TVector3Df no;
		no.x = ay * bz - az * by;
		no.y = -ax * bz + az * bx;
		no.z = ax * by - ay * bx;

		for (unsigned k = 0; k < 3; k++) m_trianglesNormals[3 * i + k] = no;
	}

	// Define OpenGL buffers:
	m_trianglesBuffer = mrpt::opengl::make_buffer(
		GL_ARRAY_BUFFER, m_triangles.data(), sizeof(m_triangles[0]) * n);

	m_normalsBuffer = mrpt::opengl::make_buffer(
		GL_ARRAY_BUFFER, m_trianglesNormals.data(),
		sizeof(m_trianglesNormals[0]) * 3 * n);

	// Generate a name for a new array.
	glGenVertexArrays(1, &m_vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(m_vao);
#endif
}

void CRenderizableShaderTriangles::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT

	if (m_enableTransparency)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}
	CHECK_OPENGL_ERROR();

	// Set up the vertex array:
	const GLint attr_position = rc.shader->attributeId("position");
	glEnableVertexAttribArray(attr_position);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(mrpt::math::TPointXYZRGBAf), /* stride */
		BUFFER_OFFSET(offsetof(mrpt::math::TPointXYZRGBAf, pt.x)));
	CHECK_OPENGL_ERROR();

	// Set up the color array:
	const GLint attr_color = rc.shader->attributeId("vertexColor");
	glEnableVertexAttribArray(attr_color);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_color, /* attribute */
		4, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(mrpt::math::TPointXYZRGBAf), /* stride */
		BUFFER_OFFSET(offsetof(mrpt::math::TPointXYZRGBAf, R)));
	CHECK_OPENGL_ERROR();

	// Set up the normals array:
	const GLint attr_normals = rc.shader->attributeId("vertexNormal");
	glEnableVertexAttribArray(attr_normals);
	glBindBuffer(GL_ARRAY_BUFFER, m_normalsBuffer);
	glVertexAttribPointer(
		attr_normals, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(mrpt::math::TVector3Df), /* stride */
		BUFFER_OFFSET(0));
	CHECK_OPENGL_ERROR();

	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);
	glDisableVertexAttribArray(attr_normals);

	if (m_enableTransparency) glDisable(GL_BLEND);
#endif
}
