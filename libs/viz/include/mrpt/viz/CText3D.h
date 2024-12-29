/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/opengl_fonts.h>

namespace mrpt::viz
{
/** A 3D text (rendered with OpenGL primitives), with selectable font face and
 * drawing style.
 *  Use \a setString and \a setFont to change the text displayed by this object
 * (can be multi-lined).
 *
 *  Text is drawn along the (+X,+Y) axes.
 *
 * Default size of characters is "1.0 units". Change it with the standard
 * method \a CVisualObject::setScale() as with any other 3D object.
 * The color can be also changed with standard methods in the base class \a
 * CRenderizable.
 *
 * ![mrpt::viz::CText3D](preview_CText3D.png)
 *
 * \sa opengl::Scene, CText
 * \note This class is based on code from libcvd (BSD,
 * http://www.edwardrosten.com/cvd/ ) \ingroup mrpt_viz_grp
 */
class CText3D : public CVisualObject
{
  DEFINE_SERIALIZABLE(CText3D, mrpt::viz)
 protected:
  std::string m_str;
  std::string m_fontName = "sans";
  TOpenGLFontStyle m_text_style;
  double m_text_spacing = 1.5;
  double m_text_kerning = 0.1;

 public:
  CText3D(
      const std::string& str = std::string(""),
      const std::string& fontName = std::string("sans"),
      const float scale = 1.0,
      const mrpt::viz::TOpenGLFontStyle text_style = mrpt::viz::NICE,
      const double text_spacing = 1.5,
      const double text_kerning = 0.1);

  ~CText3D() override;

  /** Sets the displayed string */
  void setString(const std::string& s)
  {
    m_str = s;
    CVisualObject::notifyChange();
  }

  /** Returns the currently text associated to this object */
  [[nodiscard]] const std::string& getString() const { return m_str; }

  /** Changes the font name, among accepted values: "sans", "mono", "serif" */
  void setFont(const std::string& font)
  {
    m_fontName = font;
    CVisualObject::notifyChange();
  }

  /** Returns the text font  */
  [[nodiscard]] const std::string& getFont() const { return m_fontName; }

  /** Change drawing style: FILL, OUTLINE, NICE */
  void setTextStyle(const mrpt::viz::TOpenGLFontStyle text_style)
  {
    m_text_style = text_style;
    CVisualObject::notifyChange();
  }

  /** Gets the current drawing style */
  [[nodiscard]] mrpt::viz::TOpenGLFontStyle getTextStyle() const { return m_text_style; }

  void setTextSpacing(const double text_spacing)
  {
    m_text_spacing = text_spacing;
    CVisualObject::notifyChange();
  }

  [[nodiscard]] double setTextSpacing() const { return m_text_spacing; }

  void setTextKerning(const double text_kerning)
  {
    m_text_kerning = text_kerning;
    CVisualObject::notifyChange();
  }

  [[nodiscard]] double setTextKerning() const { return m_text_kerning; }

  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  void toYAMLMap(mrpt::containers::yaml& propertiesMap) const override;
};

}  // namespace mrpt::viz
