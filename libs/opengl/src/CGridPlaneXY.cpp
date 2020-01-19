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

static const GLushort g_element_buffer_data[] = {0, 1, 2, 3};

void CGridPlaneXY::renderUpdateBuffers() const
{
#if MRPT_HAS_OPENGL_GLUT

	m_vertex_buffer_data.clear();
	for (float y = m_yMin; y <= m_yMax; y += m_frequency)
	{
		m_vertex_buffer_data.emplace_back(m_xMin, y, m_plane_z);
		m_vertex_buffer_data.emplace_back(m_xMax, y, m_plane_z);
	}

	for (float x = m_xMin; x <= m_xMax; x += m_frequency)
	{
		m_vertex_buffer_data.emplace_back(x, m_yMin, m_plane_z);
		m_vertex_buffer_data.emplace_back(x, m_yMax, m_plane_z);
	}

	m_vertexBuffer = make_buffer(
		GL_ARRAY_BUFFER, m_vertex_buffer_data.data(),
		sizeof(m_vertex_buffer_data[0]) * m_vertex_buffer_data.size());

	// Generate a name for a new array.
	glGenVertexArrays(1, &m_vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(m_vao);

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
	// glLineWidth(m_lineWidth);

	// Set up the vertex array:
	MRPT_TODO("Move this to the prepare method!");

	const GLint attr_position = shaders.attributeId("position");
	glEnableVertexAttribArray(attr_position);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
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
	glDrawArrays(GL_LINES, 0, m_vertex_buffer_data.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	CHECK_OPENGL_ERROR();

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
