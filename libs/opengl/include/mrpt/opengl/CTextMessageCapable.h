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
#ifndef opengl_CTextMessageCapable_H
#define opengl_CTextMessageCapable_H


#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/opengl_fonts.h>

namespace mrpt
{
	namespace opengl
	{
		/** Keeps a list of text messages which can be rendered to OpenGL contexts by graphic classes.
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CTextMessageCapable
		{
		protected:
			std::map<size_t,mrpt::opengl::T2DTextData>  m_2D_texts;

			/** Renders the messages to the current opengl rendering context (to be called OUT of MRPT mrpt::opengl render() methods ).
			  *  (w,h) are the dimensions of the rendering area in pixels.
			  */
			void render_text_messages(const int w, const int h) const;

		public:
			void clearTextMessages();


			/** Add 2D text messages overlapped to the 3D rendered scene. The string will remain displayed in the 3D window
			  *   until it's changed with subsequent calls to this same method, or all the texts are cleared with clearTextMessages().
			  *
			  *  \param x The X position, interpreted as absolute pixels from the left if X>=1, absolute pixels from the left if X<0 or as a width factor if in the range [0,1[.
			  *  \param y The Y position, interpreted as absolute pixels from the bottom if Y>=1, absolute pixels from the top if Y<0 or as a height factor if in the range [0,1[.
			  *  \param text The text string to display.
			  *  \param color The text color. For example: TColorf(1.0,1.0,1.0)
			  *  \param unique_index An "index" for this text message, so that subsequent calls with the same index will overwrite this text message instead of creating new ones.
			  *
			  *  You'll need to refresh the display manually with forceRepaint().
			  *
			  * \sa clearTextMessages
			  */
			void addTextMessage(
				const double x_frac,
				const double y_frac,
				const std::string &text,
				const mrpt::utils::TColorf &color = mrpt::utils::TColorf(1.0,1.0,1.0),
				const size_t unique_index = 0,
				const mrpt::opengl::TOpenGLFont font = mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24
				);

			/// \overload with more font parameters - refer to mrpt::opengl::gl_utils::glDrawText()
			void addTextMessage(
				const double x_frac,
				const double y_frac,
				const std::string &text,
				const mrpt::utils::TColorf &color,
				const std::string  &font_name,
				const double  font_size,
				const mrpt::opengl::TOpenGLFontStyle font_style = mrpt::opengl::NICE,
				const size_t  unique_index = 0,
				const double  font_spacing = 1.5,
				const double  font_kerning = 0.1
				);

		}; // end of CTextMessageCapable

	} // end namespace
} // End of namespace

#endif
