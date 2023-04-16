/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
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

	std::shared_lock<std::shared_mutex> trisReadLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);

	const auto& tris = shaderTrianglesBuffer();

	const auto n = tris.size();

	// Define OpenGL buffers:
	m_trianglesBuffer.createOnce();
	m_trianglesBuffer.bind();
	m_trianglesBuffer.allocate(tris.data(), sizeof(tris[0]) * n);

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();
#endif
}

void CRenderizableShaderTriangles::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	std::shared_lock<std::shared_mutex> trisReadLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);

	const Program& s = *rc.shader;

	// Lights:
	if (m_enableLight && rc.lights && rc.shader->hasUniform("light_color") &&
		(!rc.activeLights || rc.activeLights.value() != rc.lights))
	{
		// buffered pointer, to prevent re-setting the opengl state with the
		// same values, a performance killer:
		rc.activeLights = rc.lights;

		const auto& l = rc.lights;

		s.setFloat3("light_color", l->color.R, l->color.G, l->color.B);
		s.setFloat3(
			"light_direction", l->direction.x, l->direction.y, l->direction.z);
		s.setFloat("light_ambient", l->ambient);
		s.setFloat("light_diffuse", l->diffuse);
		if (rc.shader->hasUniform("light_specular"))
			s.setFloat("light_specular", l->specular);

		if (rc.shader->hasUniform("light_zmax"))
			s.setFloat("light_zmax", rc.state->getLastLightClipZFar());
		if (rc.shader->hasUniform("camera_far_plane"))
			s.setFloat("camera_far_plane", rc.state->getLastClipZFar());

		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Set the texture uniform:
	if (rc.shader->hasUniform("shadowMap"))
	{
		// bound to GL_TEXTURE0 + "i":
		s.setInt("shadowMap", SHADOW_MAP_TEXTURE_UNIT);

		// Shadow params:
		if (const auto& l = rc.lights; l)
		{
			rc.shader->setFloat("shadow_bias", l->shadow_bias);
			rc.shader->setFloat(
				"shadow_bias_cam2frag", l->shadow_bias_cam2frag);
			rc.shader->setFloat("shadow_bias_normal", l->shadow_bias_normal);
		}
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
		CHECK_OPENGL_ERROR_IN_DEBUG();
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
		CHECK_OPENGL_ERROR_IN_DEBUG();
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
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	if (m_cullface == TCullFace::NONE && rc.activeCullFace != TCullFace::NONE)
	{
		rc.activeCullFace = TCullFace::NONE;
		glDisable(GL_CULL_FACE);
	}
	if (m_cullface != TCullFace::NONE && rc.activeCullFace != m_cullface)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(m_cullface == TCullFace::FRONT ? GL_FRONT : GL_BACK);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		rc.activeCullFace = m_cullface;
	}

	glDrawArrays(GL_TRIANGLES, 0, 3 * shaderTrianglesBuffer().size());
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (attr_position) glDisableVertexAttribArray(*attr_position);
	if (attr_color) glDisableVertexAttribArray(*attr_color);
	if (attr_normals) glDisableVertexAttribArray(*attr_normals);

#endif
}

const math::TBoundingBoxf CRenderizableShaderTriangles::trianglesBoundingBox()
	const
{
	mrpt::math::TBoundingBoxf bb;

	std::shared_lock<std::shared_mutex> trisReadLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);

	if (shaderTrianglesBuffer().empty()) return bb;

	bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

	for (const auto& t : shaderTrianglesBuffer())
		for (int i = 0; i < 3; i++)
			bb.updateWithPoint(t.vertices[i].xyzrgba.pt);

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
