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
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderTriangles, CRenderizable, mrpt::opengl)

// Dtor:
CRenderizableShaderTriangles::~CRenderizableShaderTriangles() = default;

void CRenderizableShaderTriangles::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	// Generate vertices & colors into m_triangles
	const_cast<CRenderizableShaderTriangles&>(*this)
		.onUpdateBuffers_Triangles();

	const auto n = m_triangles.size();

	// Define OpenGL buffers:
	m_trianglesBuffer.createOnce();
	m_trianglesBuffer.bind();
	m_trianglesBuffer.allocate(m_triangles.data(), sizeof(m_triangles[0]) * n);

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();
#endif
}

void CRenderizableShaderTriangles::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// Enable/disable lights:
	if (rc.shader->hasUniform("enableLight"))
	{
		const Program& s = *rc.shader;
		GLint enabled = m_enableLight ? 1 : 0;
		glUniform1i(s.uniformId("enableLight"), enabled);
		CHECK_OPENGL_ERROR();
	}

	if (m_enableLight && rc.lights && rc.shader->hasUniform("light_diffuse") &&
		rc.shader->hasUniform("light_ambient") &&
		rc.shader->hasUniform("light_direction"))
	{
		const Program& s = *rc.shader;

		glUniform4f(
			s.uniformId("light_diffuse"), rc.lights->diffuse.R,
			rc.lights->diffuse.G, rc.lights->diffuse.B, rc.lights->diffuse.A);
		glUniform4f(
			s.uniformId("light_ambient"), rc.lights->ambient.R,
			rc.lights->ambient.G, rc.lights->ambient.B, rc.lights->ambient.A);
		// glUniform4fv(s.uniformId("light_specular"), 1,
		// &rc.lights->specular.R);
		glUniform3f(
			s.uniformId("light_direction"), rc.lights->direction.x,
			rc.lights->direction.y, rc.lights->direction.z);
		CHECK_OPENGL_ERROR();
	}

	// Set up the vertex array:
	std::optional<GLuint> attr_position;
	if (rc.shader->hasAttribute("position"))
	{
		attr_position = rc.shader->attributeId("position");
		m_vao.bind();
		glEnableVertexAttribArray(*attr_position);
		m_trianglesBuffer.bind();
		glVertexAttribPointer(
			*attr_position, /* attribute */
			3, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, xyzrgba.pt.x)));
		CHECK_OPENGL_ERROR();
	}

	// Set up the color array:
	std::optional<GLuint> attr_color;
	if (rc.shader->hasAttribute("vertexColor"))
	{
		attr_color = rc.shader->attributeId("vertexColor");
		glEnableVertexAttribArray(*attr_color);
		m_trianglesBuffer.bind();
		glVertexAttribPointer(
			*attr_color, /* attribute */
			4, /* size */
			GL_UNSIGNED_BYTE, /* type */
			GL_TRUE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, xyzrgba.r)));
		CHECK_OPENGL_ERROR();
	}

	// Set up the normals array:
	std::optional<GLuint> attr_normals;
	if (rc.shader->hasAttribute("vertexNormal"))
	{
		attr_normals = rc.shader->attributeId("vertexNormal");
		glEnableVertexAttribArray(*attr_normals);
		m_trianglesBuffer.bind();
		glVertexAttribPointer(
			*attr_normals, /* attribute */
			3, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, normal.x)));
		CHECK_OPENGL_ERROR();
	}

	if (m_cullface == TCullFace::NONE) { glDisable(GL_CULL_FACE); }
	else
	{
		glEnable(GL_CULL_FACE);
		glCullFace(m_cullface == TCullFace::FRONT ? GL_FRONT : GL_BACK);
		CHECK_OPENGL_ERROR();
	}

	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR();

	glDisable(GL_CULL_FACE);

	if (attr_position) glDisableVertexAttribArray(*attr_position);
	if (attr_color) glDisableVertexAttribArray(*attr_color);
	if (attr_normals) glDisableVertexAttribArray(*attr_normals);

#endif
}

const mrpt::math::TBoundingBox
	CRenderizableShaderTriangles::trianglesBoundingBox() const
{
	mrpt::math::TBoundingBox bb;

	if (m_triangles.empty()) return bb;

	bb.min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb.max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& t : m_triangles)
	{
		keep_min(bb.min.x, t.x(0));
		keep_max(bb.max.x, t.x(0));
		keep_min(bb.min.y, t.y(0));
		keep_max(bb.max.y, t.y(0));
		keep_min(bb.min.z, t.z(0));
		keep_max(bb.max.z, t.z(0));

		keep_min(bb.min.x, t.x(1));
		keep_max(bb.max.x, t.x(1));
		keep_min(bb.min.y, t.y(1));
		keep_max(bb.max.y, t.y(1));
		keep_min(bb.min.z, t.z(1));
		keep_max(bb.max.z, t.z(1));

		keep_min(bb.min.x, t.x(2));
		keep_max(bb.max.x, t.x(2));
		keep_min(bb.min.y, t.y(2));
		keep_max(bb.max.y, t.y(2));
		keep_min(bb.min.z, t.z(2));
		keep_max(bb.max.z, t.z(2));
	}
	return bb;
}

void CRenderizableShaderTriangles::params_serialize(
	mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint8_t>(0);  // serialization version
	out << m_enableLight << static_cast<uint8_t>(m_cullface);
}
void CRenderizableShaderTriangles::params_deserialize(
	mrpt::serialization::CArchive& in)
{
	const auto version = in.ReadAs<uint8_t>();

	switch (version)
	{
		case 0:
			in >> m_enableLight;
			m_cullface = static_cast<TCullFace>(in.ReadAs<uint8_t>());
			break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}
