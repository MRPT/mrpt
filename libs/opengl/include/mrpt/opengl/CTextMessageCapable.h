/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CTextMessageCapable_H
#define opengl_CTextMessageCapable_H


#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <map>

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

			/// overload with more font parameters - refer to mrpt::opengl::gl_utils::glDrawText()
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
