/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
				const double  font_kerning = 0.1,
				const bool has_shadow = false,
				const mrpt::utils::TColorf &shadow_color = mrpt::utils::TColorf(0,0,0)
				);

			/** Just updates the text of a given text message, without touching the other parameters.
			  * \return false if given ID doesn't exist.
			  */
			bool updateTextMessage(const size_t  unique_index, const std::string &text);

		}; // end of CTextMessageCapable

	} // end namespace
} // End of namespace

#endif
