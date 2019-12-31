/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/serialization/CArchive.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGridPlaneXY, CRenderizable, mrpt::opengl)

/** Constructor  */
CGridPlaneXY::CGridPlaneXY(
	float xMin, float xMax, float yMin, float yMax, float z, float frequency,
	float lineWidth, bool antiAliasing)
	: m_xMin(xMin),
	  m_xMax(xMax),
	  m_yMin(yMin),
	  m_yMax(yMax),
	  m_plane_z(z),
	  m_frequency(frequency),
	  m_lineWidth(lineWidth),
	  m_antiAliasing(antiAliasing)
{
}

GLuint indexBuffer;
GLuint vao;

static GLuint make_buffer(
	GLenum target, const void* buffer_data, GLsizei buffer_size)
{
	GLuint buffer;
	glGenBuffers(1, &buffer);
	glBindBuffer(target, buffer);
	glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);
	return buffer;
}

static const GLfloat g_vertex_buffer_data[] = {
	// clang-format off
	-1.0f, -1.0f, 0.0f,
	1.0f, -1.0f,  0.0f,
	-1.0f, 1.0f, 0.0f,
	1.0f, 1.0f, 0.0f
	// clang-format on
};
static const GLushort g_element_buffer_data[] = {0, 1, 2, 3};

#if 0
// Vertex shader:
const char* DEFAULT_VERTEX_SHADER_CODE = R"XXX(
#version 110

uniform mat4 p_matrix, mv_matrix;

attribute vec3 in_pos;
attribute vec4 in_color;
varying vec4 frag_color;

void main()
{
    vec4 eye_position = mv_matrix * vec4(in_pos, 1.0);
    gl_Position = p_matrix * eye_position;
    frag_color = in_color;
}
)XXX";

	// Fragment shader:
	const char* DEFAULT_FRAGMENT_SHADER_CODE = R"XXX(
#version 110

varying vec4 frag_color;

void main()
{
    gl_FragColor = frag_color;
}
)XXX";
#endif

#define BUFFER_OFFSET(offset) (reinterpret_cast<GLvoid*>(offset))

void CGridPlaneXY::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT
	std::cerr << "renderUpdateBuffers\n";

	CHECK_OPENGL_ERROR();

	vao = make_buffer(
		GL_ARRAY_BUFFER, g_vertex_buffer_data, sizeof(g_vertex_buffer_data));

	indexBuffer = make_buffer(
		GL_ELEMENT_ARRAY_BUFFER, g_element_buffer_data,
		sizeof(g_element_buffer_data));

	MRPT_TODO("Implement me!");

#endif
}

void CGridPlaneXY::render(
	const mrpt::opengl::TRenderMatrices& state,
	mrpt::opengl::Program& shaders) const
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(m_frequency >= 0);

	// Enable antialiasing:
	CHECK_OPENGL_ERROR();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	CHECK_OPENGL_ERROR();
	glEnable(GL_BLEND);
	CHECK_OPENGL_ERROR();

	MRPT_TODO("Port thick lines to opengl3?");
	//	glLineWidth(m_lineWidth);

	// bind the shaders
	glUseProgram(shaders.programId());
	CHECK_OPENGL_ERROR();

	GLint attr_position = glGetAttribLocation(shaders.programId(), "position");
	CHECK_OPENGL_ERROR();
	ASSERT_(attr_position >= 0);

	// PMV matrices:
	GLint unif_p_matrix = glGetUniformLocation(shaders.programId(), "p_matrix");
	CHECK_OPENGL_ERROR();
	ASSERT_(unif_p_matrix >= 0);

	GLint unif_mv_matrix =
		glGetUniformLocation(shaders.programId(), "mv_matrix");
	CHECK_OPENGL_ERROR();
	ASSERT_(unif_mv_matrix >= 0);

	glUniformMatrix4fv(unif_p_matrix, 1, GL_FALSE, state.p_matrix.data());
	CHECK_OPENGL_ERROR();

	glUniformMatrix4fv(unif_mv_matrix, 1, GL_FALSE, state.mv_matrix.data());
	CHECK_OPENGL_ERROR();

	// Set up the vertex array:
	MRPT_TODO("Move this to the prepare method!");

	glEnableVertexAttribArray(attr_position);
	CHECK_OPENGL_ERROR();

	glBindBuffer(GL_ARRAY_BUFFER, vao);
	CHECK_OPENGL_ERROR();

	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		0, /* stride */
		BUFFER_OFFSET(0) /* array buffer offset */
	);
	CHECK_OPENGL_ERROR();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
	CHECK_OPENGL_ERROR();
	glDrawElements(
		GL_TRIANGLE_STRIP, /* mode */
		4, /* count */
		GL_UNSIGNED_SHORT, /* type */
		BUFFER_OFFSET(0) /* element array buffer offset */
	);
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	CHECK_OPENGL_ERROR();

#if 0
	glBegin(GL_LINES);

	for (float y = m_yMin; y <= m_yMax; y += m_frequency)
	{
		glVertex3f(m_xMin, y, m_plane_z);
		glVertex3f(m_xMax, y, m_plane_z);
	}

	for (float x = m_xMin; x <= m_xMax; x += m_frequency)
	{
		glVertex3f(x, m_yMin, m_plane_z);
		glVertex3f(x, m_yMax, m_plane_z);
	}
	glEnd();
#endif

#endif
}

uint8_t CGridPlaneXY::serializeGetVersion() const { return 1; }
void CGridPlaneXY::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_xMin << m_xMax;
	out << m_yMin << m_yMax << m_plane_z;
	out << m_frequency;
	out << m_lineWidth << m_antiAliasing;  // v1
}

void CGridPlaneXY::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax >> m_plane_z;
			in >> m_frequency;
			if (version >= 1)
				in >> m_lineWidth >> m_antiAliasing;
			else
			{
				m_lineWidth = 1.0f;
				m_antiAliasing = true;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CGridPlaneXY::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = m_xMin;
	bb_min.y = m_yMin;
	bb_min.z = 0;

	bb_max.x = m_xMax;
	bb_max.y = m_yMax;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
