/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CTextMessageCapable.h>
#include <mrpt/opengl/gl_utils.h>

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

		if (it->second.draw_shadow)
		{
			// Draw shadow:
			glPushMatrix();

			glTranslatef(x+1, y-1, 0.0);
			glColor3f(it->second.shadow_color.R,it->second.shadow_color.G,it->second.shadow_color.B);
			mrpt::opengl::gl_utils::glSetFont(font_name);
			mrpt::opengl::gl_utils::glDrawText(it->second.text, font_size, font_style, font_spacing, font_kerning );

			glPopMatrix();
		}

		// Draw text:
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

#else
	MRPT_UNUSED_PARAM(w); MRPT_UNUSED_PARAM(h);
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

/** Just updates the text of a given text message, without touching the other parameters.
  * \return false if given ID doesn't exist.
  */
bool CTextMessageCapable::updateTextMessage(const size_t  unique_index, const std::string &text)
{
	std::map<size_t,mrpt::opengl::T2DTextData>::iterator it = m_2D_texts.find(unique_index);
	if (it == m_2D_texts.end())
		return false;
	else
	{
		it->second.text = text;
		return true;
	}
}


/// overload with more font parameters - refer to mrpt::opengl::gl_utils::glDrawText()
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
	const double  font_kerning,
	const bool has_shadow,
	const mrpt::utils::TColorf &shadow_color
	)
{
	mrpt::opengl::T2DTextData  d;
	d.text = text;
	d.color = color;
	d.draw_shadow = has_shadow;
	d.shadow_color = shadow_color;
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
