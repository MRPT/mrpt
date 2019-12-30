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
std::shared_ptr<mrpt::opengl::Program> shaders;

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

#define BUFFER_OFFSET(offset) ((GLvoid*)(offset))
const int NumPoints = 5;

void CGridPlaneXY::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT
	std::cerr << "renderUpdateBuffers\n";

	CHECK_OPENGL_ERROR();

	// Create a buffer
	GLuint buffer;
	glGenBuffers(1, &buffer);
	CHECK_OPENGL_ERROR();

	mrpt::math::TPoint3Df points[NumPoints] = {
		{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, 1, 1}};

	// Bind it to GL_ARRAY_BUFFER and pass the data to the GPU
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	CHECK_OPENGL_ERROR();
	glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);
	CHECK_OPENGL_ERROR();

	// -----------------
	glGenBuffers(1, &indexBuffer);
	CHECK_OPENGL_ERROR();

	// Bind it to GL_ARRAY_BUFFER and pass the data to the GPU
	glBindBuffer(GL_ARRAY_BUFFER, indexBuffer);
	CHECK_OPENGL_ERROR();

	int indices[NumPoints] = {0, 1, 2, 3, 0};
	glBufferData(GL_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	CHECK_OPENGL_ERROR();
	// -----------------

	// Create a vertex array object (VAO)
	glGenVertexArrays(1, &vao);
	CHECK_OPENGL_ERROR();
	glBindVertexArray(vao);
	CHECK_OPENGL_ERROR();

	// Initialize the vertex in_pos attribute defined in the vertex shader

	// Get an index for the attribute from the shader
	GLuint loc_in_pos = glGetAttribLocation(shaders->programId(), "in_pos");
	CHECK_OPENGL_ERROR();
	glEnableVertexAttribArray(loc_in_pos);
	CHECK_OPENGL_ERROR();

	// Associate the attribute with the data in the buffer.
	// glVertexAttribPointer implicitly refers to the currently bound
	// GL_ARRAY_BUFFER
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	CHECK_OPENGL_ERROR();
	glVertexAttribPointer(
		loc_in_pos, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
	CHECK_OPENGL_ERROR();

	// Add indices for indexed rendering
	// Binding to GL_ELEMENT_ARRAY_BUFFER is saved with VAO state
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
	CHECK_OPENGL_ERROR();

	// Unbind the buffer
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	CHECK_OPENGL_ERROR();

	// Unbind the VAO
	glBindVertexArray(0);
	CHECK_OPENGL_ERROR();

	glPointSize(4.0f);  // make points show up better
	CHECK_OPENGL_ERROR();

	//
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

	// bind the VAO
	glBindVertexArray(vao);
	CHECK_OPENGL_ERROR();

	// draw primitives
	glDrawArrays(GL_LINES, 0, NumPoints);
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
