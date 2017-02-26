/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_opengl_fonts_H
#define mrpt_opengl_fonts_H

#include <mrpt/utils/TColor.h>
#include <mrpt/utils/compiler_fixes.h> // disable warnings
#include <mrpt/opengl/link_pragmas.h>
#include <string>

namespace mrpt
{
	namespace opengl
	{
		/** Existing fonts for 2D texts in mrpt::opengl methods.
		  * \sa mrpt::opengl::CMyGLCanvasBase::renderTextBitmap
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
		enum TOpenGLFontStyle {
			FILL = 0,       ///< renders glyphs as filled polygons
			OUTLINE = 1,    ///< renders glyphs as outlines with GL_LINES
			NICE = 2        ///< renders glyphs filled with antialiased outlines
		};

		/** A description of a bitmapped or vectorized text font.
		  *  (Vectorized fonts are recommended for new code).
		  *
		  * \sa mrpt::opengl::gl_utils::glSetFont(), mrpt::opengl::gl_utils::glDrawText()
		  */
		struct OPENGL_IMPEXP TFontParams
		{
			TFontParams() :
				draw_shadow(false),
				font(MRPT_GLUT_BITMAP_NONE),
				vfont_name("sans"),
				vfont_scale(10),
				vfont_style(),
				vfont_spacing(1.5),
				vfont_kerning(0.1)
			{
			}

			mrpt::utils::TColorf	color;

			bool                    draw_shadow;
			mrpt::utils::TColorf	shadow_color;

			/** @name Bitmapped font params
			    @{ */
			mrpt::opengl::TOpenGLFont font;
			/** @} */

			/** @name Vectorized font params - Applicable only if font==MRPT_GLUT_BITMAP_NONE
			    @{ */
			std::string             vfont_name;  //!< Vectorized font name ("sans","mono","serif")
			double                  vfont_scale; //!< Size of characters
			TOpenGLFontStyle        vfont_style; //!< (default: NICE) See TOpenGLFontStyle.
			double                  vfont_spacing; //!< (default: 1.5) Refer to mrpt::opengl::gl_utils::glDrawText
			double                  vfont_kerning; //!< (default: 0.1) Refer to mrpt::opengl::gl_utils::glDrawText
			/** @} */
		};

		/** An auxiliary struct for holding a list of text messages in some mrpt::opengl & mrpt::gui classes
		  *  The font can be either a bitmapped or a vectorized font.
		  *  \sa mrpt::opengl::CTextMessageCapable
		  * \ingroup mrpt_opengl_grp
		  */
		struct OPENGL_IMPEXP T2DTextData : public TFontParams
		{
			T2DTextData() : x(0),y(0) { }

			std::string text;
			double      x,y;
		};

	}
}

#endif
