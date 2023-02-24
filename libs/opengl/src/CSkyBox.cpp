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
	ASSERT_(m_cubeTexture.initialized());

	{
		const Program& s = *rc.shader;
		// bound to GL_TEXTURE0 + "i":
		glUniform1i(s.uniformId("textureSampler"), m_cubeTexture.textureUnit());
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

#endif
}

uint8_t CSkyBox::serializeGetVersion() const { return 0; }
void CSkyBox::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// out << m_textureImage;
}

void CSkyBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			// in >> m_textureImage;
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
