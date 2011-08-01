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
#ifndef opengl_glutils_H
#define opengl_glutils_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/opengl/opengl_fonts.h>

#ifndef opengl_CRenderizable_H
#	include <mrpt/opengl/CRenderizable.h>
#endif

#include <mrpt/opengl/link_pragmas.h>

namespace mrpt
{
	namespace opengl
	{
		/** A set of auxiliary functions that can be called to render OpenGL primitives from MRPT or user code
		  */
		namespace gl_utils
		{
			/** For each object in the list:
			  *   - checks visibility of each object
			  *   - prepare the GL_MODELVIEW matrix according to its coordinates
			  *   - call its ::render()
			  *   - shows its name (if enabled).
			  *
			  *  \note Used by  COpenGLViewport, CSetOfObjects
			  */
			void OPENGL_IMPEXP renderSetOfObjects(const mrpt::opengl::CListOpenGLObjects &objs);

			/** Checks glGetError and throws an exception if an error situation is found */
			void OPENGL_IMPEXP checkOpenGLError();



			/** @name OpenGL bitmapped 2D fonts
			    @{ */

			/** This method is safe for calling from within ::render() methods \sa renderTextBitmap */
			void OPENGL_IMPEXP renderTextBitmap( const char *str, void *fontStyle );

			/** Return the exact width in pixels for a given string, as will be rendered by renderTextBitmap().
			  * \sa renderTextBitmap
			  */
			int OPENGL_IMPEXP textBitmapWidth(
				const std::string &str,
				mrpt::opengl::TOpenGLFont    font = mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24 );

			/** @} */   // --------------------------------------------------


			/** @name OpenGL vector 3D fonts
			    @{ */

			/// sets the font to use for future font rendering commands. currently sans, serif and mono are available.
			/// @param fontname string containing font name
			void OPENGL_IMPEXP  glSetFont( const std::string & fontname );

			/// returns the name of the currently active font
			const OPENGL_IMPEXP std::string & glGetFont();

			/// different style for font rendering
			enum TEXT_STYLE {
				FILL = 0,       ///< renders glyphs as filled polygons
				OUTLINE = 1,    ///< renders glyphs as outlines with GL_LINES
				NICE = 2        ///< renders glyphs filled with antialiased outlines
			};

			/// renders a string in GL using the current settings.
			/// Font coordinates are +X along the line and +Y along the up direction of glyphs.
			/// The origin is at the top baseline at the left of the first character. Characters have a maximum size of 1.
			/// linefeed is interpreted as a new line and the start is offset in -Y direction by @ref spacing . Individual characters
			/// are separated by @ref kerning + plus their individual with.
			/// @param text string to be rendered, unknown characters are replaced with '?'
			/// @param textScale The size of the characters (default=1.0)
			/// @param style rendering style
			/// @param spacing distance between individual text lines
			/// @param kerning distance between characters
			/// \note This functions comes from libcvd (LGPL, http://www.edwardrosten.com/cvd/ )
			std::pair<double, double> OPENGL_IMPEXP glDrawText(const std::string & text, const double textScale, enum TEXT_STYLE style = NICE, double spacing = 1.5, double kerning = 0.1);

			/// returns the size of the bounding box of a text to be rendered, similar to @ref glDrawText but without any visual output
			/// \note This functions comes from libcvd (LGPL, http://www.edwardrosten.com/cvd/ )
			std::pair<double, double> OPENGL_IMPEXP glGetExtends(const std::string & text, const double textScale, double spacing = 1.5, double kerning = 0.1);

			/** @} */   // --------------------------------------------------

		}
	}
}

#endif
