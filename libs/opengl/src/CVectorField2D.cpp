/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CVectorField2D.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CVectorField2D, CRenderizableDisplayList, mrpt::opengl )

CVectorField2DPtr CVectorField2D::Create(const CMatrixFloat &Matrix_x, const CMatrixFloat &Matrix_y, float xmin, float xmax, float ymin, float ymax)	
{
	return CVectorField2DPtr(new CVectorField2D( Matrix_x,  Matrix_y, xmin, xmax, ymin, ymax));
}

/** Constructor */
CVectorField2D::CVectorField2D()
	: xcomp(0,0), ycomp(0,0), xMin(-1.0), xMax(1.0), yMin(-1.0), yMax(1.0), m_LineWidth(1.0),m_pointSize(1.0),m_antiAliasing(true)
{
	m_point_color = m_color;
	m_field_color = m_color;
}

/** Constructor with a initial set of lines. */
CVectorField2D::CVectorField2D( CMatrixFloat Matrix_x, CMatrixFloat Matrix_y, float xmin, float xmax, float ymin, float ymax)
	: m_LineWidth(1.0),m_pointSize(1.0),m_antiAliasing(true)
{
	MRPT_UNUSED_PARAM(Matrix_x); MRPT_UNUSED_PARAM(Matrix_y);
	MRPT_UNUSED_PARAM(xmin); MRPT_UNUSED_PARAM(xmax); MRPT_UNUSED_PARAM(ymin); MRPT_UNUSED_PARAM(ymax);
	m_point_color = m_color;
	m_field_color = m_color;
}


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CVectorField2D::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT

	// Enable antialiasing:
	glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
	if (m_antiAliasing || m_color.A != 255)
	{
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	if (m_antiAliasing)
	{
		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_POINT_SMOOTH);
	}

	glLineWidth(m_LineWidth);
	glPointSize(m_pointSize);

	checkOpenGLError();

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_POINTS);
	glColor4ub( m_point_color.R, m_point_color.G, m_point_color.B, m_point_color.A);

	const float x_cell_size = (xMax - xMin)/(xcomp.getColCount()-1);
	const float y_cell_size = (yMax - yMin)/(ycomp.getRowCount()-1);

	for (unsigned int i=0; i<xcomp.getColCount(); i++)
		for (unsigned int j=0; j<xcomp.getRowCount(); j++)
		{
			glVertex3f( xMin+i*x_cell_size, yMin+j*y_cell_size, 0);
		}

	glEnd();

	glBegin(GL_LINES);
	glColor4ub( m_field_color.R, m_field_color.G, m_field_color.B, m_field_color.A);
	for (unsigned int i=0; i<xcomp.getColCount(); i++)
		for (unsigned int j=0; j<xcomp.getRowCount(); j++)
		{
			glVertex3f( xMin+i*x_cell_size, yMin+j*y_cell_size, 0);
			glVertex3f( xMin+i*x_cell_size + xcomp(j,i), yMin+j*y_cell_size + ycomp(j,i), 0);
		}
	glEnd();

	glBegin(GL_TRIANGLES);
	glColor4ub( m_field_color.R, m_field_color.G, m_field_color.B, m_field_color.A);
	for (unsigned int i=0; i<xcomp.getColCount(); i++)
		for (unsigned int j=0; j<xcomp.getRowCount(); j++)
		{
			const float tri_side = 0.25*sqrt(xcomp(j,i)*xcomp(j,i) + ycomp(j,i)*ycomp(j,i));
			const float ang = ::atan2(ycomp(j,i), xcomp(j,i)) - 1.5708;
			glVertex3f( -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i), cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
			glVertex3f( cos(ang)*0.5*tri_side + xMin+i*x_cell_size + xcomp(j,i), sin(ang)*0.5*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
			glVertex3f( -cos(ang)*0.5*tri_side + xMin+i*x_cell_size + xcomp(j,i), -sin(ang)*0.5*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
		}
	glEnd();

	checkOpenGLError();
	glEnable(GL_LIGHTING);  // Disable lights when drawing lines

	// End of antialiasing:
    glPopAttrib();

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CVectorField2D::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);

		out << xcomp << ycomp;
		out << xMin << xMax << yMin << yMax;
		out << m_LineWidth;
		out << m_pointSize;
		out << m_antiAliasing;
		out << m_point_color;
		out << m_field_color;

	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CVectorField2D::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
		case 0:
			readFromStreamRender(in);

			in >> xcomp >> ycomp;
			in >> xMin >> xMax >> yMin >> yMax;
			in >> m_LineWidth;
			in >> m_pointSize;
			in >> m_antiAliasing;
			in >> m_point_color;
			in >> m_field_color;
			break;

		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
			break;
	};
	CRenderizableDisplayList::notifyChange();
}


void CVectorField2D::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = xMin;
	bb_min.y = yMin;
	bb_min.z = 0;

	bb_max.x = xMax;
	bb_max.y = yMax;
	bb_max.z = 0;

	const float x_cell_size = (xMax - xMin)/(xcomp.getColCount()-1);
	const float y_cell_size = (yMax - yMin)/(ycomp.getRowCount()-1);

	for (unsigned int i=0; i<xcomp.getColCount(); i++)
		for (unsigned int j=0; j<xcomp.getRowCount(); j++)
		{
			const float tri_side = 0.25*sqrt(xcomp(j,i)*xcomp(j,i) + ycomp(j,i)*ycomp(j,i));
			const float ang = ::atan2(ycomp(j,i), xcomp(j,i)) - 1.5708;

			if ( -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i) < bb_min.x)
				bb_min.x = -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i);

			if ( cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i) < bb_min.y)
				bb_min.y = cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i);

			if ( -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i) > bb_max.x)
				bb_max.x = -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i);

			if ( cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i) > bb_max.y)
				bb_max.y = cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i);
		}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CVectorField2D::adjustVectorFieldToGrid()
{
	ASSERT_(xcomp.size() > 0)

	const float ratio_xp = xcomp.maxCoeff()*(xcomp.getColCount()-1)/(xMax-xMin);
	const float ratio_xn = xcomp.minCoeff()*(xcomp.getColCount()-1)/(xMax-xMin);
	const float ratio_yp = ycomp.maxCoeff()*(ycomp.getRowCount()-1)/(yMax-yMin);
	const float ratio_yn = ycomp.minCoeff()*(ycomp.getRowCount()-1)/(yMax-yMin);
	const float norm_factor = 0.85/max(max(ratio_xp, abs(ratio_xn)), max(ratio_yp, abs(ratio_yn)));

	xcomp *= norm_factor;
	ycomp *= norm_factor;
	CRenderizableDisplayList::notifyChange();
}
