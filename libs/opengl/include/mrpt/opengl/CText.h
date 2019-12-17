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
/** A 2D text (bitmap rendering): it always "faces the observer" despite it's at
 * some 3D location.
 *  Use setString and setFont to change the text displayed by this object.
 *
 *  \note All texts appear with the font GLUT_BITMAP_TIMES_ROMAN_10 for now
 * (i.e. setFont is ignored)
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CText </td> <td> \image html preview_CText.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 *  \sa CText3D
 * \ingroup mrpt_opengl_grp
 */
class CText : public CRenderizable
{
	DEFINE_SERIALIZABLE(CText, mrpt::opengl)
   protected:
	std::string m_str;
	std::string m_fontName;
	int m_fontHeight, m_fontWidth;

   public:
	/** Sets the text to display */
	void setString(const std::string& s) { m_str = s; }
	/** Return the current text associated to this label */
	std::string getString() const { return m_str; }
	/** Sets the font (It has no effect yet!) */
	void setFont(const std::string& s, int height)
	{
		m_fontName = s;
		m_fontHeight = height;
	}
	std::string getFont() const { return m_fontName; }

	void render() const override;
	void renderUpdateBuffers() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor */
	CText(const std::string& str = std::string(""));

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CText() override;
};

}  // namespace mrpt::opengl
