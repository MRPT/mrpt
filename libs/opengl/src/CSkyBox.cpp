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
#include <mrpt/opengl/CSkyBox.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include <memory>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSkyBox, CRenderizable, mrpt::opengl)

void CSkyBox::renderUpdateBuffers() const
{
	// vertices:
	static constexpr float skyboxVertices[] = {
		// Positions
		-1.0f, 1.0f,  -1.0f,  //
		-1.0f, -1.0f, -1.0f,  //
		1.0f,  -1.0f, -1.0f,  //
		1.0f,  -1.0f, -1.0f,  //
		1.0f,  1.0f,  -1.0f,  //
		-1.0f, 1.0f,  -1.0f,  //

		-1.0f, -1.0f, 1.0f,	 //
		-1.0f, -1.0f, -1.0f,  //
		-1.0f, 1.0f,  -1.0f,  //
		-1.0f, 1.0f,  -1.0f,  //
		-1.0f, 1.0f,  1.0f,	 //
		-1.0f, -1.0f, 1.0f,	 //

		1.0f,  -1.0f, -1.0f,  //
		1.0f,  -1.0f, 1.0f,	 //
		1.0f,  1.0f,  1.0f,	 //
		1.0f,  1.0f,  1.0f,	 //
		1.0f,  1.0f,  -1.0f,  //
		1.0f,  -1.0f, -1.0f,  //

		-1.0f, -1.0f, 1.0f,	 //
		-1.0f, 1.0f,  1.0f,	 //
		1.0f,  1.0f,  1.0f,	 //
		1.0f,  1.0f,  1.0f,	 //
		1.0f,  -1.0f, 1.0f,	 //
		-1.0f, -1.0f, 1.0f,	 //

		-1.0f, 1.0f,  -1.0f,  //
		1.0f,  1.0f,  -1.0f,  //
		1.0f,  1.0f,  1.0f,	 //
		1.0f,  1.0f,  1.0f,	 //
		-1.0f, 1.0f,  1.0f,	 //
		-1.0f, 1.0f,  -1.0f,  //

		-1.0f, -1.0f, -1.0f,  //
		-1.0f, -1.0f, 1.0f,	 //
		1.0f,  -1.0f, -1.0f,  //
		1.0f,  -1.0f, -1.0f,  //
		-1.0f, -1.0f, 1.0f,	 //
		1.0f,  -1.0f, 1.0f	//
	};

	// Define OpenGL buffers:
	m_vbo.createOnce();
	m_vbo.bind();
	m_vbo.allocate(skyboxVertices, sizeof(skyboxVertices));

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();
}

void CSkyBox::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// This will load and/or select our texture, only once:
	initializeTextures();
	ASSERT_(m_glTexture.get().has_value());

	std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

	// Set the texture uniform:
	if (!rc.activeTextureUnit ||
		*rc.activeTextureUnit != m_glTexture.get()->unit)
	{
		rc.activeTextureUnit = m_glTexture.get()->unit;	 // buffer it
		const Program& s = *rc.shader;
		// bound to GL_TEXTURE0 + "i":
		glUniform1i(s.uniformId("textureSampler"), m_glTexture.get()->unit);
	}

	// Lights:
	if (m_enableLight && rc.lights && rc.shader->hasUniform("light_diffuse") &&
		rc.shader->hasUniform("light_ambient") &&
		rc.shader->hasUniform("light_direction") &&
		(!rc.activeLights || rc.activeLights.value() != rc.lights))
	{
		// buffered pointer, to prevent re-setting the opengl state with the
		// same values, a performance killer:
		rc.activeLights = rc.lights;

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
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Set up the vertex array:
	std::optional<GLuint> attr_position;
	if (rc.shader->hasAttribute("position"))
	{
		attr_position = rc.shader->attributeId("position");
		m_vao.bind();
		glEnableVertexAttribArray(*attr_position);
		m_vbo.bind();
		glVertexAttribPointer(
			*attr_position, /* attribute */
			3, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, xyzrgba.pt.x)));
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Set up the normals array:
	std::optional<GLuint> attr_normals;
	if (rc.shader->hasAttribute("vertexNormal"))
	{
		attr_normals = rc.shader->attributeId("vertexNormal");
		glEnableVertexAttribArray(*attr_normals);
		m_vbo.bind();
		glVertexAttribPointer(
			*attr_normals, /* attribute */
			3, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, normal.x)));
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Set up the UV array:
	std::optional<GLuint> attr_uv;
	if (rc.shader->hasAttribute("vertexUV"))
	{
		attr_uv = rc.shader->attributeId("vertexUV");
		glEnableVertexAttribArray(*attr_uv);
		m_vbo.bind();
		glVertexAttribPointer(
			*attr_uv, /* attribute */
			2, /* size */
			GL_FLOAT, /* type */
			GL_FALSE, /* normalized? */
			sizeof(TTriangle::Vertex), /* stride */
			BUFFER_OFFSET(offsetof(TTriangle::Vertex, uv.x)));
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	if (m_cullface == TCullFace::NONE &&
		(!rc.activeCullFace || *rc.activeCullFace != TCullFace::NONE))
	{
		rc.activeCullFace = TCullFace::NONE;
		glDisable(GL_CULL_FACE);
	}
	if (m_cullface != TCullFace::NONE &&
		(!rc.activeCullFace || *rc.activeCullFace != m_cullface))
	{
		glEnable(GL_CULL_FACE);
		glCullFace(m_cullface == TCullFace::FRONT ? GL_FRONT : GL_BACK);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		rc.activeCullFace = m_cullface;
	}

	// Draw:
	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR_IN_DEBUG();

	if (attr_position) glDisableVertexAttribArray(*attr_position);
	if (attr_uv) glDisableVertexAttribArray(*attr_uv);
	if (attr_normals) glDisableVertexAttribArray(*attr_normals);

#endif
}

uint8_t CSkyBox::serializeGetVersion() const { return 0; }
void CSkyBox::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_textureImage;
}

void CSkyBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			in >> m_textureImage;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
auto CSkyBox::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
	return {};
}

void CSkyBox::assignImage(
	const CSkyBox::TEXTURE_FACE face, const mrpt::img::CImage& img)
{
	// xx;
}
