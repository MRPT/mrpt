/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizableShaderText.h>

namespace mrpt::opengl
{
/** A 2D text that always "faces the observer" despite it having a real 3D
 * position, used to compute its position on the screen, and depth (so it can be
 * occluded).
 *
 * Use setString() and setFont() to change the text and its appareance.
 *
 * ![mrpt::opengl::CText](preview_CText.png)
 *
 * \sa CText3D, opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CText : public CRenderizableShaderText
{
	DEFINE_SERIALIZABLE(CText, mrpt::opengl)
   protected:
	std::string m_str;
	std::string m_fontName = "sans";
	int m_fontHeight = 20, m_fontWidth = 0;

	void onUpdateBuffers_Text() override;

   public:
	/** Sets the text to display */
	void setString(const std::string& s)
	{
		if (m_str == s) return;
		m_str = s;
		CRenderizable::notifyChange();
	}
	/** Return the current text associated to this label */
	std::string getString() const { return m_str; }

	/** Sets the font among "sans", "serif", "mono". */
	void setFont(const std::string& s, int height)
	{
		if (m_fontName == s && m_fontHeight == height) return;
		m_fontName = s;
		m_fontHeight = height;
		CRenderizable::notifyChange();
	}
	std::string getFont() const { return m_fontName; }

	shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TEXT};
	}
	void render(const RenderContext& rc) const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	mrpt::math::TBoundingBox getBoundingBox() const override;

	/** Constructor */
	CText(const std::string& str = std::string("")) : m_str(str) {}

	virtual ~CText() override;

	std::pair<double, double> computeTextExtension() const;

	void toYAMLMap(mrpt::containers::yaml& propertiesMap) const override;
};

}  // namespace mrpt::opengl
