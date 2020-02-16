/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/common.h>  // disable warnings
#include <mrpt/img/TColor.h>
#include <string>

namespace mrpt::opengl
{
/** Existing fonts for 2D texts in mrpt::opengl methods.
 * \sa mrpt::opengl::CWxGLCanvasBase::renderTextBitmap
 * \ingroup mrpt_opengl_grp
 */
enum TOpenGLFont
{
	MRPT_GLUT_BITMAP_NONE = -1,
	MRPT_GLUT_BITMAP_TIMES_ROMAN_10 = 0,
	MRPT_GLUT_BITMAP_TIMES_ROMAN_24 = 1,
	MRPT_GLUT_BITMAP_HELVETICA_10 = 2,
	MRPT_GLUT_BITMAP_HELVETICA_12 = 3,
	MRPT_GLUT_BITMAP_HELVETICA_18 = 4
};

/** Different style for vectorized font rendering \sa T2DTextData */
enum TOpenGLFontStyle
{
	FILL = 0,  ///< renders glyphs as filled polygons
	OUTLINE = 1,  ///< renders glyphs as outlines with GL_LINES
	NICE = 2  ///< renders glyphs filled with antialiased outlines
};

/** A description of a bitmapped or vectorized text font.
 *  (Vectorized fonts are recommended for new code).
 *
 * \sa mrpt::opengl::gl_utils::glSetFont(),
 * mrpt::opengl::gl_utils::glDrawText()
 */
struct TFontParams
{
	TFontParams() : vfont_name("sans") {}
	mrpt::img::TColorf color;

	bool draw_shadow{false};
	mrpt::img::TColorf shadow_color;

	/** @name Bitmapped font params
		@{ */
	mrpt::opengl::TOpenGLFont font{MRPT_GLUT_BITMAP_NONE};
	/** @} */

	/** @name Vectorized font params - Applicable only if
	   font==MRPT_GLUT_BITMAP_NONE
		@{ */
	/** Vectorized font name ("sans","mono","serif") */
	std::string vfont_name;
	/** Size of characters */
	float vfont_scale{10.0f};
	/** (default: NICE) See TOpenGLFontStyle. */
	TOpenGLFontStyle vfont_style{};
	/** (default: 1.5) Refer to mrpt::opengl::gl_utils::glDrawText */
	double vfont_spacing{1.5};
	/** (default: 0.1) Refer to mrpt::opengl::gl_utils::glDrawText */
	double vfont_kerning{0.1};
	/** @} */
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
