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

		/** An auxiliary struct for holding a list of text messages in some mrpt::opengl & mrpt::gui classes
		  *  The font can be either a bitmapped or a vectorized font.
		  *  \sa mrpt::opengl::CTextMessageCapable
		  * \ingroup mrpt_opengl_grp
		  */
		struct OPENGL_IMPEXP T2DTextData
		{
			T2DTextData() :
				font(MRPT_GLUT_BITMAP_NONE),
				vfont_name(),
				vfont_spacing(1.5),
				vfont_kerning(0.1)
			{
			}

			std::string 			text;
			mrpt::utils::TColorf	color;
			double					x,y;
			/** @name Bitmapped font params
			    @{ */
			mrpt::opengl::TOpenGLFont font;
			/** @} */

			/** @name Vectorized font params - Applicable only if font==MRPT_GLUT_BITMAP_NONE
			    @{ */
			std::string             vfont_name;  //!< Vectorized font name ("sans","mono","serif")
			double                  vfont_scale; //!< Size of characters
			TOpenGLFontStyle        vfont_style;
			double                  vfont_spacing; //!< Refer to mrpt::opengl::gl_utils::glDrawText
			double                  vfont_kerning; //!< Refer to mrpt::opengl::gl_utils::glDrawText
			/** @} */
		};

	}
}

#endif
