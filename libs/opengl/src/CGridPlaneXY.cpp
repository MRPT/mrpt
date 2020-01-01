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

static GLuint vertexBuffer, vao;

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
	1.0f, -1.0f, 0.0f,
	0.0f,  1.0f, 0.0f,
	// clang-format on
};
static const GLushort g_element_buffer_data[] = {0, 1, 2, 3};

#define BUFFER_OFFSET(offset) (reinterpret_cast<GLvoid*>(offset))

void CGridPlaneXY::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT
	std::cerr << "renderUpdateBuffers\n";

	CHECK_OPENGL_ERROR();

	vertexBuffer = make_buffer(
		GL_ARRAY_BUFFER, g_vertex_buffer_data, sizeof(g_vertex_buffer_data));

	/*	indexBuffer = make_buffer(
			GL_ELEMENT_ARRAY_BUFFER, g_element_buffer_data,
			sizeof(g_element_buffer_data));
	*/
	// Generate a name for a new array.
	glGenVertexArrays(1, &vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(vao);

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
	// glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	CHECK_OPENGL_ERROR();
	// glEnable(GL_BLEND);
	CHECK_OPENGL_ERROR();

	MRPT_TODO("Port thick lines to opengl3?");
	//	glLineWidth(m_lineWidth);

	// Set up the vertex array:
	MRPT_TODO("Move this to the prepare method!");

#if 0
	glBindVertexArray(vao);
	CHECK_OPENGL_ERROR();
#endif

	//	const GLint attr_position = shaders.attributeId("position");
	const GLint attr_position = 0;

	glEnableVertexAttribArray(attr_position);

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
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

	// glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
	// CHECK_OPENGL_ERROR();
	glDrawArrays(GL_TRIANGLES, 0, 3);
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
