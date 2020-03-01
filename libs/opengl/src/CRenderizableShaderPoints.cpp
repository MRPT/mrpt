/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/serialization/CArchive.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderPoints, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderPoints::~CRenderizableShaderPoints() = default;

void CRenderizableShaderPoints::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Generate vertices & colors:
	const_cast<CRenderizableShaderPoints&>(*this).onUpdateBuffers_Points();

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

void CRenderizableShaderPoints::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT

	// Point size as uniform:
	glUniform1f(rc.shader->uniformId("vertexPointSize"), m_pointSize);

	// Variable point size code in the shader:
	glUniform1i(
		rc.shader->uniformId("enableVariablePointSize"),
		m_variablePointSize ? 1 : 0);

	glUniform1f(
		rc.shader->uniformId("variablePointSize_K"), m_variablePointSize_K);
	glUniform1f(
		rc.shader->uniformId("variablePointSize_DepthScale"),
		m_variablePointSize_DepthScale);

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

	glDrawArrays(GL_POINTS, 0, m_vertex_buffer_data.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);
	CHECK_OPENGL_ERROR();
#endif
}

void CRenderizableShaderPoints::params_serialize(
	mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint8_t>(0);  // serialization version
	out << m_pointSize << m_variablePointSize << m_variablePointSize_K
		<< m_variablePointSize_DepthScale;
}
void CRenderizableShaderPoints::params_deserialize(
	mrpt::serialization::CArchive& in)
{
	const auto version = in.ReadAs<uint8_t>();

	switch (version)
	{
		case 0:
			in >> m_pointSize >> m_variablePointSize >> m_variablePointSize_K >>
				m_variablePointSize_DepthScale;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
