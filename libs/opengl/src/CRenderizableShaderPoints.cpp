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
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderPoints, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderPoints::~CRenderizableShaderPoints() = default;

void CRenderizableShaderPoints::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// Generate vertices & colors:
	const_cast<CRenderizableShaderPoints&>(*this).onUpdateBuffers_Points();

	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

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

void CRenderizableShaderPoints::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

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

	glDrawArrays(GL_POINTS, 0, m_vertex_buffer_data.size());
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (attr_position) glDisableVertexAttribArray(*attr_position);
	if (attr_color) glDisableVertexAttribArray(*attr_color);
	CHECK_OPENGL_ERROR_IN_DEBUG();
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
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

const mrpt::math::TBoundingBoxf CRenderizableShaderPoints::verticesBoundingBox()
	const
{
	std::shared_lock<std::shared_mutex> wfReadLock(
		CRenderizableShaderPoints::m_pointsMtx.data);

	mrpt::math::TBoundingBoxf bb;

	if (m_vertex_buffer_data.empty()) return bb;

	bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

	for (const auto& p : m_vertex_buffer_data)
		bb.updateWithPoint(p);

	return bb;
}
