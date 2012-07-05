/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_opengl_fonts_H
#define mrpt_opengl_fonts_H

#include <mrpt/utils/TColor.h>
#include <mrpt/utils/types.h>
#include <mrpt/opengl/link_pragmas.h>

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
				font(MRPT_GLUT_BITMAP_NONE),
				vfont_name("sans"),
				vfont_scale(10),
				vfont_style(),
				vfont_spacing(1.5),
				vfont_kerning(0.1)
			{
			}

			mrpt::utils::TColorf	color;

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
			T2DTextData() { }

			std::string text;
			double      x,y;
		};

	}
}

#endif
