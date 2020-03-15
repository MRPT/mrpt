/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <map>

namespace mrpt::opengl
{
/** Keeps a list of text messages which can be rendered to OpenGL contexts by
 * graphic classes.
 * \ingroup mrpt_opengl_grp
 */
class CTextMessageCapable
{
   public:
	void clearTextMessages();

	/** Add 2D text messages overlapped to the 3D rendered scene. The string
	 * will remain displayed in the 3D window
	 *   until it's changed with subsequent calls to this same method, or all
	 * the texts are cleared with clearTextMessages().
	 *
	 *  \param x The X position, interpreted as absolute pixels from the left
	 * if X>=1, absolute pixels from the left if X<0 or as a width factor if in
	 * the range [0,1[.
	 *  \param y The Y position, interpreted as absolute pixels from the bottom
	 * if Y>=1, absolute pixels from the top if Y<0 or as a height factor if in
	 * the range [0,1[.
	 *  \param text The text string to display.
	 *  \param color The text color. For example: TColorf(1.0,1.0,1.0)
	 *  \param unique_index An "index" for this text message, so that
	 * subsequent calls with the same index will overwrite this text message
	 * instead of creating new ones.
	 *
	 *  You'll need to refresh the display manually with forceRepaint().
	 *
	 * \sa clearTextMessages, updateTextMessage
	 */
	void addTextMessage(
		const double x_frac, const double y_frac, const std::string& text,
		const size_t unique_index = 0,
		const TFontParams& fontParams = TFontParams());

	/** Just updates the text of a given text message, without touching the
	 * other parameters.
	 * \return false if given ID doesn't exist.
	 */
	bool updateTextMessage(const size_t unique_index, const std::string& text);

	struct DataPerText : mrpt::opengl::T2DTextData
	{
		mutable mrpt::opengl::CText::Ptr gl_text, gl_text_shadow;
		mutable bool gl_text_outdated = true;
	};

	struct TListTextMessages
	{
		std::map<size_t, DataPerText> messages;

		/** (re)generate all CText objects in the gl_text fields */
		void regenerateGLobjects() const;
	};

	const TListTextMessages& getTextMessages() const { return m_2D_texts; }

   protected:
	TListTextMessages m_2D_texts;

};  // end of CTextMessageCapable

}  // namespace mrpt::opengl
