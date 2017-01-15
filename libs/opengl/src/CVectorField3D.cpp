/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CVectorField3D.h>
#include <mrpt/utils/CStream.h>
#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CVectorField3D, CRenderizableDisplayList, mrpt::opengl )

/** Constructor */
CVectorField3D::CVectorField3D()
    : x_vf(0,0), y_vf(0,0), z_vf(0,0), x_p(0,0), y_p(0,0), z_p(0,0), m_LineWidth(1.0),m_pointSize(1.0),m_antiAliasing(true),m_colorFromModule(false),m_showPoints(true)
{
	m_point_color = m_color;
	m_field_color = m_color;
	m_still_color = m_color;
	m_maxspeed_color = m_color;
	m_maxspeed = 1.f;
}

/** Constructor with a initial set of lines. */
CVectorField3D::CVectorField3D( CMatrixFloat x_vf_ini, CMatrixFloat y_vf_ini, CMatrixFloat z_vf_ini, CMatrixFloat x_p_ini, CMatrixFloat y_p_ini, CMatrixFloat z_p_ini)
    : m_LineWidth(1.0),m_pointSize(1.0),m_antiAliasing(true),m_colorFromModule(false),m_showPoints(true)
{
	x_vf = x_vf_ini;
	y_vf = y_vf_ini;
	z_vf = z_vf_ini;
	x_p = x_p_ini;
	y_p = y_p_ini;
	z_p = z_p_ini;
	m_point_color = m_color;
	m_field_color = m_color;
	m_still_color = m_color;
	m_maxspeed_color = m_color;
	m_maxspeed = 1.f;
}


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CVectorField3D::render_dl() const
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

	if (m_showPoints)
	{
		glBegin(GL_POINTS);
		glColor4ub( m_point_color.R, m_point_color.G, m_point_color.B, m_point_color.A);

		for (unsigned int i=0; i<x_p.getColCount(); i++)
			for (unsigned int j=0; j<x_p.getRowCount(); j++)
			{
				glVertex3f( x_p(j,i), y_p(j,i), z_p(j,i));
			}

		glEnd();
	}

	glBegin(GL_LINES);
	if (m_colorFromModule == false)
	{
		glColor4ub( m_field_color.R, m_field_color.G, m_field_color.B, m_field_color.A);
		for (unsigned int i=0; i<x_vf.getColCount(); i++)
			for (unsigned int j=0; j<x_vf.getRowCount(); j++)
			{
				glVertex3f( x_p(j,i), y_p(j,i), z_p(j,i));
				glVertex3f( x_p(j,i) + x_vf(j,i), y_p(j,i) + y_vf(j,i), z_p(j,i) + z_vf(j,i));
			}
	}
	else
	{
		for (unsigned int i=0; i<x_vf.getColCount(); i++)
			for (unsigned int j=0; j<x_vf.getRowCount(); j++)
			{
				//Compute color
				const float module = sqrt(square(x_vf(j,i)) + square(y_vf(j,i)) + square(z_vf(j,i)));
				if (module > m_maxspeed)
					glColor4ub( m_maxspeed_color.R, m_maxspeed_color.G, m_maxspeed_color.B, m_maxspeed_color.A);
				else
				{
					const float R = (m_maxspeed-module)*m_still_color.R/m_maxspeed + module*m_maxspeed_color.R/m_maxspeed;
					const float G = (m_maxspeed-module)*m_still_color.G/m_maxspeed + module*m_maxspeed_color.G/m_maxspeed;
					const float B = (m_maxspeed-module)*m_still_color.B/m_maxspeed + module*m_maxspeed_color.B/m_maxspeed;
					const float A = (m_maxspeed-module)*m_still_color.A/m_maxspeed + module*m_maxspeed_color.A/m_maxspeed;
					glColor4ub( R, G, B, A);
				}

				glVertex3f( x_p(j,i), y_p(j,i), z_p(j,i));
				glVertex3f( x_p(j,i) + x_vf(j,i), y_p(j,i) + y_vf(j,i), z_p(j,i) + z_vf(j,i));
			}
	}
	glEnd();

	//******** Future ************
//	glBegin(GL_TRIANGLES);
//	glColor4ub( m_field_color.R, m_field_color.G, m_field_color.B, m_field_color.A);
//	for (unsigned int i=0; i<xcomp.getColCount(); i++)
//		for (unsigned int j=0; j<xcomp.getRowCount(); j++)
//		{
//			const float tri_side = 0.25*sqrt(xcomp(j,i)*xcomp(j,i) + ycomp(j,i)*ycomp(j,i));
//			const float ang = ::atan2(ycomp(j,i), xcomp(j,i)) - 1.5708;
//			glVertex3f( -sin(ang)*0.866*tri_side + xMin+i*x_cell_size + xcomp(j,i), cos(ang)*0.866*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
//			glVertex3f( cos(ang)*0.5*tri_side + xMin+i*x_cell_size + xcomp(j,i), sin(ang)*0.5*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
//			glVertex3f( -cos(ang)*0.5*tri_side + xMin+i*x_cell_size + xcomp(j,i), -sin(ang)*0.5*tri_side + yMin+j*y_cell_size + ycomp(j,i), 0);
//		}
//	glEnd();

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
void CVectorField3D::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);

		out << x_vf << y_vf << z_vf;
		out << x_p << y_p << z_p;
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
void CVectorField3D::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
		case 0:
			readFromStreamRender(in);

            in >> x_vf >> y_vf >> z_vf;
            in >> x_p >> y_p >> z_p;
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


void CVectorField3D::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = 10e10; bb_min.y = 10e10; bb_min.z = 10e10;
	bb_max.x = -10e10; bb_max.y = -10e10; bb_max.z = -10e10;

	for (unsigned int i=0; i<x_p.getColCount(); i++)
		for (unsigned int j=0; j<x_p.getRowCount(); j++)
		{
			//Minimum values
			if (x_p(j,i) < bb_min.x)
				bb_min.x = x_p(j,i);

			if (x_p(j,i) + x_vf(j,i) < bb_min.x)
				bb_min.x = x_p(j,i) + x_vf(j,i);

			if (y_p(j,i) < bb_min.y)
				bb_min.y = y_p(j,i);

			if (y_p(j,i) + y_vf(j,i) < bb_min.y)
				bb_min.y = y_p(j,i) + y_vf(j,i);

			if (z_p(j,i) < bb_min.z)
				bb_min.z = z_p(j,i);

			if (z_p(j,i) + z_vf(j,i) < bb_min.z)
				bb_min.z = z_p(j,i) + z_vf(j,i);

			//Maximum values
			if (x_p(j,i) > bb_max.x)
				bb_max.x = x_p(j,i);

			if (x_p(j,i) + x_vf(j,i) > bb_max.x)
				bb_max.x = x_p(j,i) + x_vf(j,i);

			if (y_p(j,i) > bb_max.y)
				bb_max.y = y_p(j,i);

			if (y_p(j,i) + y_vf(j,i) > bb_max.y)
				bb_max.y = y_p(j,i) + y_vf(j,i);

			if (z_p(j,i) > bb_max.z)
				bb_max.z = z_p(j,i);

			if (z_p(j,i) + z_vf(j,i) > bb_max.z)
				bb_max.z = z_p(j,i) + z_vf(j,i);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}


