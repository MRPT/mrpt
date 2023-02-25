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
#include <mrpt/serialization/stl_serialization.h>

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
	ASSERT_(m_cubeTexture.initialized());

	// Needed to draw only "free" pixels:
	glDepthFunc(GL_LEQUAL);

	// Disable cull:
	rc.activeCullFace = TCullFace::NONE;
	glDisable(GL_CULL_FACE);

	{
		const Program& s = *rc.shader;
		// bound to GL_TEXTURE0 + "i":
		glUniform1i(s.uniformId("skybox"), m_cubeTexture.textureUnit());
	}

	// Set up the vertex array:
	GLuint attr_position;
	{
		attr_position = rc.shader->attributeId("position");
		m_vao.bind();
		glEnableVertexAttribArray(attr_position);
		m_vbo.bind();
		glVertexAttribPointer(
			attr_position,	// attribute
			3,	// size
			GL_FLOAT,  // type
			GL_FALSE,  // normalized?
			0,	// stride
			nullptr	 // offset
		);
		CHECK_OPENGL_ERROR_IN_DEBUG();
	}

	// Draw:
	glDrawArrays(GL_TRIANGLES, 0, 36);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glDisableVertexAttribArray(attr_position);
	glDepthFunc(GL_LESS);  // restore

#endif
}

uint8_t CSkyBox::serializeGetVersion() const { return 0; }
void CSkyBox::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_textureImages;	 // <mrpt/serialization/stl_serialization.h>
}

void CSkyBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			in >> m_textureImages;
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
	const CUBE_TEXTURE_FACE face, const mrpt::img::CImage& img)
{
	const int faceIdx = static_cast<int>(face);
	ASSERT_GE_(faceIdx, 0);
	ASSERT_LT_(faceIdx, 6);

	m_textureImages[faceIdx] = img;
	CRenderizable::notifyChange();
}

void CSkyBox::assignImage(const CUBE_TEXTURE_FACE face, mrpt::img::CImage&& img)
{
	const int faceIdx = static_cast<int>(face);
	ASSERT_GE_(faceIdx, 0);
	ASSERT_LT_(faceIdx, 6);

	m_textureImages[faceIdx] = std::move(img);
	CRenderizable::notifyChange();
}

void CSkyBox::initializeTextures() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

	if (m_cubeTexture.initialized())
	{
		m_cubeTexture.bindAsCubeTexture();
		return;
	}

	// Reserve the new one --------------------------
	m_cubeTexture.assignCubeImages(m_textureImages);

#endif
}

CSkyBox::~CSkyBox()
{
	try
	{
		m_cubeTexture.unloadTexture();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[~CSkyBox] Ignoring exception: "
				  << mrpt::exception_to_str(e);
	}
}
