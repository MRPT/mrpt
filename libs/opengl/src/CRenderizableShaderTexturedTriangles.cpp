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
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/TLightParameters.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>

#include <iostream>
#include <memory>  // std::align
#include <thread>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;
using mrpt::img::CImage;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CRenderizableShaderTexturedTriangles, CRenderizable, mrpt::opengl)

void CRenderizableShaderTexturedTriangles::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	// Generate vertices & colors into m_triangles
	const_cast<CRenderizableShaderTexturedTriangles&>(*this)
		.onUpdateBuffers_TexturedTriangles();

	std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

	const auto n = m_triangles.size();

	// Define OpenGL buffers:
	m_vbo.createOnce();
	m_vbo.bind();
	m_vbo.allocate(m_triangles.data(), sizeof(m_triangles[0]) * n);

	// VAO: required to use glEnableVertexAttribArray()
	m_vao.createOnce();

#endif
}

void CRenderizableShaderTexturedTriangles::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// This will load and/or select our texture, only once:
	initializeTextures();
	ASSERT_(m_glTexture.initialized());

	std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

	// Set the texture uniform:
	const auto texUnit = m_glTexture.textureUnit();
	if (!rc.activeTextureUnit || *rc.activeTextureUnit != texUnit)
	{
		rc.activeTextureUnit = texUnit;	 // buffer it
		const Program& s = *rc.shader;
		// bound to GL_TEXTURE0 + "i":
		glUniform1i(s.uniformId("textureSampler"), texUnit);
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
		const auto& diff = rc.lights->diffuse;
		const auto& amb = rc.lights->ambient;
		// const auto& spc = rc.lights->specular;
		const auto& dir = rc.lights->direction;

		glUniform4f(
			s.uniformId("light_diffuse"), diff.R, diff.G, diff.B, diff.A);
		glUniform4f(s.uniformId("light_ambient"), amb.R, amb.G, amb.B, amb.A);
		// glUniform4f(s.uniformId("light_specular"), spc.R, spc.G, spc.B,
		// spc.A);
		glUniform3f(s.uniformId("light_direction"), dir.x, dir.y, dir.z);
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

void CRenderizableShaderTexturedTriangles::assignImage(
	const CImage& img, const CImage& imgAlpha)
{
	MRPT_START

	CRenderizable::notifyChange();

	m_glTexture.unloadTexture();

	// Make a copy:
	m_textureImage = img;
	m_textureImageAlpha = imgAlpha;
	m_textureImageAssigned = true;

	m_enableTransparency = true;

	MRPT_END
}

void CRenderizableShaderTexturedTriangles::assignImage(const CImage& img)
{
	MRPT_START

	CRenderizable::notifyChange();

	m_glTexture.unloadTexture();

	// Make a copy:
	m_textureImage = img;
	m_textureImageAssigned = true;

	m_enableTransparency = false;

	MRPT_END
}

void CRenderizableShaderTexturedTriangles::assignImage(
	CImage&& img, CImage&& imgAlpha)
{
	MRPT_START

	CRenderizable::notifyChange();

	m_glTexture.unloadTexture();

	m_textureImage = std::move(img);
	m_textureImageAlpha = std::move(imgAlpha);
	m_textureImageAssigned = true;

	m_enableTransparency = true;

	MRPT_END
}

void CRenderizableShaderTexturedTriangles::assignImage(CImage&& img)
{
	MRPT_START

	CRenderizable::notifyChange();

	m_glTexture.unloadTexture();

	m_textureImage = std::move(img);
	m_textureImageAssigned = true;

	m_enableTransparency = false;

	MRPT_END
}

void CRenderizableShaderTexturedTriangles::initializeTextures() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	// Note: if we are rendering and the user assigned us no texture image,
	// let's create a dummy one with the uniform CRenderizable's color:
	if (!textureImageHasBeenAssigned() || m_textureImage.isEmpty())
	{
		mrpt::img::CImage im_rgb(4, 4, mrpt::img::CH_RGB),
			im_a(4, 4, mrpt::img::CH_GRAY);
		im_rgb.filledRectangle(0, 0, 3, 3, m_color);
		im_a.filledRectangle(
			0, 0, 3, 3,
			mrpt::img::TColor(m_color.A, m_color.A, m_color.A, m_color.A));
		const_cast<CRenderizableShaderTexturedTriangles*>(this)->assignImage(
			std::move(im_rgb), std::move(im_a));
	}

	if (m_glTexture.initialized())
	{
		m_glTexture.bind();	 // activate it:
		return;
	}

	// Reserve the new one --------------------------
	mrpt::opengl::COpenGLTexture::Options opts;
	opts.enableTransparency = m_enableTransparency;
	opts.magnifyLinearFilter = m_textureInterpolate;
	opts.generateMipMaps = m_textureUseMipMaps;

	m_glTexture.assignImage(m_textureImage, m_textureImageAlpha, opts);

#endif
}

CRenderizableShaderTexturedTriangles::~CRenderizableShaderTexturedTriangles()
{
	try
	{
		m_glTexture.unloadTexture();
	}
	catch (const std::exception& e)
	{
		std::cerr
			<< "[~CRenderizableShaderTexturedTriangles] Ignoring exception: "
			<< mrpt::exception_to_str(e);
	}
}

void CRenderizableShaderTexturedTriangles::writeToStreamTexturedObject(
	mrpt::serialization::CArchive& out) const
{
	uint8_t ver = 3;

	out << ver;
	out << m_enableTransparency << m_textureInterpolate << m_textureUseMipMaps;
	out << m_textureImage;
	if (m_enableTransparency) out << m_textureImageAlpha;
	out << m_textureImageAssigned;
	out << m_enableLight << static_cast<uint8_t>(m_cullface);  // v2
}

void CRenderizableShaderTexturedTriangles::readFromStreamTexturedObject(
	mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;

	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			in >> m_enableTransparency >> m_textureInterpolate;
			if (version >= 3) { in >> m_textureUseMipMaps; }
			else
			{
				m_textureUseMipMaps = true;
			}
			in >> m_textureImage;
			if (m_enableTransparency)
			{
				in >> m_textureImageAlpha;
				assignImage(m_textureImage, m_textureImageAlpha);
			}
			else
			{
				assignImage(m_textureImage);
			}
			if (version >= 1) in >> m_textureImageAssigned;
			else
				m_textureImageAssigned = true;

			if (version >= 2)
			{
				in >> m_enableLight;
				m_cullface = static_cast<TCullFace>(in.ReadAs<uint8_t>());
			}
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};

	CRenderizable::notifyChange();
}

const mrpt::math::TBoundingBoxf
	CRenderizableShaderTexturedTriangles::trianglesBoundingBox() const
{
	mrpt::math::TBoundingBoxf bb;

	std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

	if (m_triangles.empty()) return bb;

	bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

	for (const auto& t : m_triangles)
		for (int i = 0; i < 3; i++)
			bb.updateWithPoint(t.vertices[i].xyzrgba.pt);

	return bb;
}
