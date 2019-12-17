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

namespace mrpt::opengl
{
/** A 3D text (rendered with OpenGL primitives), with selectable font face and
 * drawing style.
 *  Use \a setString and \a setFont to change the text displayed by this object
 * (can be multi-lined).
 *
 *  Text is drawn along the (+X,+Y) axes.
 *
 *  Default size of characters is "1.0 units". Change it with the standard
 * method \a CRenderizable::setScale() as with any other 3D object.
 *  The color can be also changed with standard methods in the base class \a
 * CRenderizable.
 *
 *  \sa opengl::COpenGLScene, CText
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CText3D </td> <td> \image html preview_CText3D.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \note This class is based on code from libcvd (BSD,
 * http://www.edwardrosten.com/cvd/ )
 * \ingroup mrpt_opengl_grp
 */
class CText3D : public CRenderizable
{
	DEFINE_SERIALIZABLE(CText3D, mrpt::opengl)
   protected:
	std::string m_str;
	std::string m_fontName;
	TOpenGLFontStyle m_text_style;
	double m_text_spacing;
	double m_text_kerning;

   public:
	/** Sets the displayed string */
	inline void setString(const std::string& s)
	{
		m_str = s;
		CRenderizable::notifyChange();
	}
	/** Returns the currently text associated to this object */
	inline const std::string& getString() const { return m_str; }
	/** Changes the font name, among accepted values: "sans", "mono", "serif" */
	inline void setFont(const std::string& font)
	{
		m_fontName = font;
		CRenderizable::notifyChange();
	}
	/** Returns the text font  */
	inline const std::string& getFont() const { return m_fontName; }
	/** Change drawing style: FILL, OUTLINE, NICE */
	void setTextStyle(const mrpt::opengl::TOpenGLFontStyle text_style)
	{
		m_text_style = text_style;
		CRenderizable::notifyChange();
	}
	/** Gets the current drawing style */
	mrpt::opengl::TOpenGLFontStyle getTextStyle() const { return m_text_style; }
	void setTextSpacing(const double text_spacing)
	{
		m_text_spacing = text_spacing;
		CRenderizable::notifyChange();
	}
	double setTextSpacing() const { return m_text_spacing; }
	void setTextKerning(const double text_kerning)
	{
		m_text_kerning = text_kerning;
		CRenderizable::notifyChange();
	}
	double setTextKerning() const { return m_text_kerning; }

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor */
	CText3D(
		const std::string& str = std::string(""),
		const std::string& fontName = std::string("sans"),
		const double scale = 1.0,
		const mrpt::opengl::TOpenGLFontStyle text_style = mrpt::opengl::NICE,
		const double text_spacing = 1.5, const double text_kerning = 0.1);

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CText3D() override;
};

}  // namespace mrpt::opengl
