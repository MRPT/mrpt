/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/common.h>  // disable warnings
#include <mrpt/img/TColor.h>

#include <string>

namespace mrpt::opengl
{
/** Different style for vectorized font rendering \sa T2DTextData */
enum TOpenGLFontStyle
{
	FILL = 0,  ///< renders glyphs as filled polygons
	OUTLINE = 1,  ///< renders glyphs as outlines with GL_LINES

	/** This was "renders glyphs filled with antialiased outlines", but since
	   antialiased is not properly implemented in mrtp2 since the port to
	   OpenGL3, NICE is for now an alias for FILL. */
	NICE = 2
};

/** A description of a bitmapped or vectorized text font.
 *  (Vectorized fonts are recommended for new code).
 *
 * \sa mrpt::opengl::gl_utils::glSetFont(),
 * mrpt::opengl::gl_utils::glDrawText()
 */
struct TFontParams
{
	TFontParams() = default;

	/** Vectorized font name ("sans","mono","serif") */
	std::string vfont_name = "mono";

	/** Size of characters [pixels] */
	float vfont_scale = 10.0f;

	mrpt::img::TColorf color = {1.0f, 1.0f, 1.0f, 1.0f};

	bool draw_shadow = false;
	mrpt::img::TColorf shadow_color = {0.0f, 0.0f, 0.0f, 1.0f};

	/** (default: FILL) See TOpenGLFontStyle. */
	TOpenGLFontStyle vfont_style = opengl::FILL;

	/** (default: 1.5) Refer to mrpt::opengl::gl_utils::glDrawText */
	double vfont_spacing = 1.5;

	/** (default: 0.1) Refer to mrpt::opengl::gl_utils::glDrawText */
	double vfont_kerning = 0.1;
};

/** An auxiliary struct for holding a list of text messages in some mrpt::opengl
 * & mrpt::gui classes
 *  The font can be either a bitmapped or a vectorized font.
 *  \sa mrpt::opengl::CTextMessageCapable
 * \ingroup mrpt_opengl_grp
 */
struct T2DTextData : public TFontParams
{
	T2DTextData() = default;
	std::string text;
	double x{0}, y{0};
};
}  // namespace mrpt::opengl
