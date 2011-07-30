/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/CRenderizable.h>		// Include these before windows.h!!
#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;


/** For each object in the list:
*   - checks visibility of each object
*   - prepare the GL_MODELVIEW matrix according to its coordinates
*   - call its ::render()
*   - shows its name (if enabled).
*/
void CRenderizable::glutils::renderSetOfObjects(const CListOpenGLObjects &objectsToRender)
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_PROFILE_FUNC_START // Just the non-try/catch part of MRPT_START

	CListOpenGLObjects::const_iterator	itP;
	try
	{
		for (itP=objectsToRender.begin();itP!=objectsToRender.end();++itP)
		{
			if (!itP->present()) continue;
			const CRenderizable * it = itP->pointer(); // Use plain pointers, faster than smart pointers:
			if (!it->isVisible()) continue;

			// 3D coordinates transformation:
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			//glPushAttrib(GL_ALL_ATTRIB_BITS);
			//CRenderizable::checkOpenGLError();

			// It's more efficient to prepare the 4x4 matrix ourselves and load it directly into opengl stack:
			//  A homogeneous transformation matrix, in this order:
			//
			//     0  4  8  12
			//     1  5  9  13
			//     2  6  10 14
			//     3  7  11 15
			//
			const CPose3D & pos = it->getPoseRef();
			const CMatrixDouble33 &R = pos.m_ROT;
			const GLdouble m[16] = {
				R.coeff(0,0),R.coeff(1,0),R.coeff(2,0), 0,
				R.coeff(0,1),R.coeff(1,1),R.coeff(2,1), 0,
				R.coeff(0,2),R.coeff(1,2),R.coeff(2,2), 0,
				pos.m_coords[0],pos.m_coords[1],pos.m_coords[2], 1
				};
			glMultMatrixd( m );  // Multiply so it's composed with the previous, current MODELVIEW matrix

			// Do scaling after the other transformations!
			if (it->m_scale_x!=1 || it->m_scale_y!=1 || it->m_scale_z!=1)
				glScalef(it->m_scale_x,it->m_scale_y,it->m_scale_z);

			// Set color:
			glColor4f( it->m_color_R,it->m_color_G,it->m_color_B,it->m_color_A);

			it->render();

			if (it->m_show_name)
			{
				glDisable(GL_DEPTH_TEST);
				glColor3f(1.f,1.f,1.f);  // Must be called BEFORE glRasterPos3f
				glRasterPos3f(0.0f,0.0f,0.0f);

				GLfloat		raster_pos[4];
				glGetFloatv( GL_CURRENT_RASTER_POSITION, raster_pos);
				float eye_distance= raster_pos[3];

				void *font=NULL;
				if (eye_distance<2)
						font = GLUT_BITMAP_TIMES_ROMAN_24;
				else if(eye_distance<200)
					font = GLUT_BITMAP_TIMES_ROMAN_10;

				if (font)
					CRenderizable::renderTextBitmap( it->m_name.c_str(), font);

				glEnable(GL_DEPTH_TEST);
			}

			//glPopAttrib();
//			CRenderizable::checkOpenGLError();

			glPopMatrix();
			CRenderizable::checkOpenGLError();

		} // end foreach object
	}
	catch(exception &e)
	{
		char str[1000];
		os::sprintf(str,1000,"Exception while rendering a class '%s'\n%s",
			(*itP)->GetRuntimeClass()->className,
			e.what() );
		THROW_EXCEPTION(str);
	}
	catch(...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#endif
}


/*---------------------------------------------------------------
					renderTextBitmap
  ---------------------------------------------------------------*/
void	CRenderizable::glutils::renderTextBitmap( const char *str, void *fontStyle )
{
#if MRPT_HAS_OPENGL_GLUT
	while ( *str ) glutBitmapCharacter( fontStyle ,*(str++) );
#endif
}


void *aux_mrptfont2glutfont(const TOpenGLFont font)
{
#if MRPT_HAS_OPENGL_GLUT
	switch (font)
	{
	default:
	case MRPT_GLUT_BITMAP_TIMES_ROMAN_10: return GLUT_BITMAP_TIMES_ROMAN_10; break;
	case MRPT_GLUT_BITMAP_TIMES_ROMAN_24: return GLUT_BITMAP_TIMES_ROMAN_24; break;

	case MRPT_GLUT_BITMAP_HELVETICA_10: return GLUT_BITMAP_HELVETICA_10; break;
	case MRPT_GLUT_BITMAP_HELVETICA_12: return GLUT_BITMAP_HELVETICA_12; break;
	case MRPT_GLUT_BITMAP_HELVETICA_18: return GLUT_BITMAP_HELVETICA_18; break;
	}
#else
	return NULL;
#endif
}

/** Return the exact width in pixels for a given string, as will be rendered by renderTextBitmap().
  * \sa renderTextBitmap
  */
int CRenderizable::glutils::textBitmapWidth(
	const std::string &str,
	mrpt::opengl::TOpenGLFont    font)
{
#if MRPT_HAS_OPENGL_GLUT
	if (str.empty()) return 0;
	return glutBitmapLength(aux_mrptfont2glutfont(font), (const unsigned char*)str.c_str() );
#else
	return 10;
#endif
}

/*---------------------------------------------------------------
					renderTextBitmap
  ---------------------------------------------------------------*/
void CRenderizable::renderTextBitmap(
	int screen_x,
	int screen_y,
	const std::string &str,
	float  color_r,
	float  color_g,
	float  color_b,
	TOpenGLFont font
	)
{
#if MRPT_HAS_OPENGL_GLUT
    glDisable(GL_DEPTH_TEST);

	// If (x,y) are negative, wrap to the opposite side:
	if (screen_x<0 || screen_y<0)
	{
		// Size of the viewport:
		GLint	win_dims[4];  // [2]:width ,[3]:height
		glGetIntegerv( GL_VIEWPORT, win_dims );

		if (screen_x<0) screen_x += win_dims[2];
		if (screen_y<0) screen_y += win_dims[3];
	}

	// Draw text:
    glColor3f(color_r,color_g,color_b);

    // From: http://www.mesa3d.org/brianp/sig97/gotchas.htm
	GLfloat fx, fy;

	/* Push current matrix mode and viewport attributes */
	glPushAttrib( GL_TRANSFORM_BIT | GL_VIEWPORT_BIT );

	/* Setup projection parameters */
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	//glDepthRange( z, z );
	glViewport( (int) screen_x - 1, (int) screen_y - 1, 2, 2 );

	/* set the raster (window) position */
	fx = screen_x - (int) screen_x;
	fy = screen_y - (int) screen_y;
	//glRasterPos4f( fx, fy, 0.0, w );
	glRasterPos3f( fx, fy, 0.0 );

	/* restore matrices, viewport and matrix mode */
	glPopMatrix();
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glPopAttrib();

	// Select font:
	void *glut_font_sel = aux_mrptfont2glutfont(font);

	for (size_t i=0;i<str.size();i++)
		glutBitmapCharacter( glut_font_sel ,str[i] );

    glEnable(GL_DEPTH_TEST);
#endif
}


