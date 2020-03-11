/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CTextMessageCapable.h>

#include <mrpt/opengl/opengl_api.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

void CTextMessageCapable::TListTextMessages::regenerateGLobjects() const
{
	// (re)generate the opengl CText objects for each label:
	for (auto& kv : messages)
	{
		const DataPerText& labelData = kv.second;
		if (labelData.gl_text && labelData.gl_text_outdated) continue;

		if (!labelData.gl_text)
		{
			labelData.gl_text = mrpt::opengl::CText::Create();
		}
		if (labelData.draw_shadow && !labelData.gl_text_shadow)
			labelData.gl_text_shadow = mrpt::opengl::CText::Create();

		if (!labelData.draw_shadow && labelData.gl_text_shadow)
			labelData.gl_text_shadow.reset();

		kv.second.gl_text_outdated = false;
	}
}

void CTextMessageCapable::clearTextMessages() { m_2D_texts.messages.clear(); }

/** Just updates the text of a given text message, without touching the other
 * parameters.
 * \return false if given ID doesn't exist.
 */
bool CTextMessageCapable::updateTextMessage(
	const size_t unique_index, const std::string& text)
{
	auto it = m_2D_texts.messages.find(unique_index);
	if (it == m_2D_texts.messages.end())
		return false;
	else
	{
		it->second.text = text;
		it->second.gl_text_outdated = true;
		return true;
	}
}

/// overload with more font parameters - refer to
/// mrpt::opengl::gl_utils::glDrawText()
void CTextMessageCapable::addTextMessage(
	const double x_frac, const double y_frac, const std::string& text,
	const size_t unique_index, const TFontParams& fontParams)
{
	DataPerText d;
	static_cast<TFontParams&>(d) = fontParams;
	d.text = text;
	d.x = x_frac;
	d.y = y_frac;

	m_2D_texts.messages[unique_index] = std::move(d);
}
