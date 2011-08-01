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

#include <mrpt/opengl/CTextMessageCapable.h>

#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;


/** Renders the messages to the current opengl rendering context (to be called OUT of MRPT mrpt::opengl render() methods ).
  *  (w,h) are the dimensions of the rendering area in pixels.
  */
void CTextMessageCapable::render_text_messages(const int w, const int h) const
{
#if MRPT_HAS_OPENGL_GLUT
	// Render text labels as opengl primitives (much faster):
	GLint old_matMode = 0;
	glGetIntegerv(GL_MATRIX_MODE, &old_matMode);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glLoadIdentity();
	glOrtho(0,w,0,h,-1,1);

	glMatrixMode(GL_MODELVIEW);

	glDisable(GL_DEPTH_TEST);

	for (std::map<size_t,mrpt::opengl::T2DTextData>::const_iterator it=m_2D_texts.begin();it!=m_2D_texts.end();++it)
	{
		// If (x,y) \in [0,1[, it's interpreted as a ratio, otherwise, as an actual coordinate in pixels
		const int x = it->second.x>=1 ? int(it->second.x) : (it->second.x<0 ? int(w+it->second.x) : int(it->second.x * w));
		const int y = it->second.y>=1 ? int(it->second.y) : (it->second.y<0 ? int(h+it->second.y) : int(it->second.y * h));

		// Font size and family:
		double font_size=10;
		string font_name="sans";
		TOpenGLFontStyle font_style = mrpt::opengl::FILL;
		double font_spacing = 1.5;
		double font_kerning = 0.1;

		switch(it->second.font)
		{
			case MRPT_GLUT_BITMAP_TIMES_ROMAN_10: font_size=10; font_name="sans"; break;
			case MRPT_GLUT_BITMAP_TIMES_ROMAN_24: font_size=24; font_name="sans"; break;
			case MRPT_GLUT_BITMAP_HELVETICA_10:  font_size=10; font_name="mono"; break;
			case MRPT_GLUT_BITMAP_HELVETICA_12:  font_size=12; font_name="mono"; break;
			case MRPT_GLUT_BITMAP_HELVETICA_18:  font_size=18; font_name="mono"; break;

			// This means this is a vectorized font, so just copy the parameters set by the user:
			case MRPT_GLUT_BITMAP_NONE:
				font_size    = it->second.vfont_scale;
				font_name    = it->second.vfont_name;
				font_style   = it->second.vfont_style;
				font_spacing = it->second.vfont_spacing;
				font_kerning = it->second.vfont_kerning;
				break;

			default:
				std::cerr << "[CTextMessageCapable::render_text_messages] Invalid value for TOpenGLFont\n";
				break;
		};

		glPushMatrix();

		glTranslatef(x, y, 0.0);
		glColor3f(it->second.color.R,it->second.color.G,it->second.color.B);
		mrpt::opengl::gl_utils::glSetFont(font_name);
		mrpt::opengl::gl_utils::glDrawText(it->second.text, font_size, font_style, font_spacing, font_kerning );

		glPopMatrix();
	}

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	if (old_matMode!=GL_PROJECTION)
		glMatrixMode(old_matMode);

#endif
}

void CTextMessageCapable::clearTextMessages()
{
	m_2D_texts.clear();
}


void CTextMessageCapable::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const size_t unique_index ,
	const mrpt::opengl::TOpenGLFont font
	)
{
	mrpt::opengl::T2DTextData  d;
	d.text = text;
	d.color = color;
	d.x = x_frac;
	d.y = y_frac;
	d.font = font;

	m_2D_texts[unique_index] = d;
}

/// \overload with more font parameters - refer to mrpt::opengl::gl_utils::glDrawText()
void CTextMessageCapable::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const std::string  &font_name,
	const double  font_size,
	const mrpt::opengl::TOpenGLFontStyle font_style,
	const size_t  unique_index,
	const double  font_spacing,
	const double  font_kerning
	)
{
	mrpt::opengl::T2DTextData  d;
	d.text = text;
	d.color = color;
	d.x = x_frac;
	d.y = y_frac;
	d.font = MRPT_GLUT_BITMAP_NONE;  // It's not a bitmapped font
	d.vfont_name = font_name;
	d.vfont_scale = font_size;
	d.vfont_style = font_style;
	d.vfont_spacing = font_spacing;
	d.vfont_kerning = font_kerning;

	m_2D_texts[unique_index] = d;
}
