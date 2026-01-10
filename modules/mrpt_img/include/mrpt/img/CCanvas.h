/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/math_frwds.h>

#include <cmath>  // sin() cos()

namespace mrpt::img
{
class CImage;

/** This virtual class defines the interface of any object accepting drawing
 * primitives on it.
 *
 *  A number of text fonts can be selected with CCanvas::selectTextFont(). These
 * are the
 *   implemented font names:
 *
 *  - "6x13"
 *  - "6x13B" (bold)
 *  - "6x13O" (italic)
 *  - "9x15"
 *  - "9x15B" (bold)
 *  - "10x20"
 *  - "18x18ja" (Japanese, UNICODE character values)
 *
 *  For an example of each font check the <a
 * href="http://www.mrpt.org/Implemented_2D_Fonts">corresponding wiki page</a>.
 *
 * \sa CImage
 * \ingroup mrpt_img_grp
 */
class CCanvas
{
 protected:
  /** The selected font name. */
  std::string m_selectedFont{"9x15"};

  /** Direct access to character bitmaps. */
  const uint32_t* m_selectedFontBitmaps{nullptr};

 public:
  CCanvas() = default;
  virtual ~CCanvas() = default;

  CCanvas(const CCanvas& other) = default;
  CCanvas& operator=(const CCanvas& other) = default;
  CCanvas(CCanvas&& other) noexcept = default;
  CCanvas& operator=(CCanvas&& other) noexcept = default;

  /** Definition of pen styles */
  enum TPenStyle : uint8_t
  {
    psSolid = 0,
    psDash,      /* -------  */
    psDot,       /* .......  */
    psDashDot,   /* _._._._  */
    psDashDotDot /* _.._.._  */
  };

  /** Changes the value of the pixel (x,y).
   *  Pixel coordinates starts at the left-top corner of the image, and start
   * in (0,0).
   *  The meaning of the parameter "color" depends on the implementation: it
   * will usually
   *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray
   * level.
   *
   *  You can also use a TColor() type as input and it will be automatically
   * converted to size_t.
   *
   *  This method must support (x,y) values OUT of the actual image size
   * without neither
   *   raising exceptions, nor leading to memory access errors.
   *
   */
  virtual void setPixel(const TPixelCoord& pt, const mrpt::img::TColor& color) = 0;

  /** Returns the width of the image in pixels
   */
  [[nodiscard]] virtual int32_t getWidth() const = 0;

  /** Returns the height of the image in pixels
   */
  [[nodiscard]] virtual int32_t getHeight() const = 0;

  /** Draws a line.
   * \param pt0 The starting point
   * \param pt1 The end point
   * \param color The color of the line
   * \param width The desired width of the line (this is IGNORED in this
   * virtual class)
   *  This method may be redefined in some classes implementing this
   * interface in a more appropriate manner.
   */
  virtual void line(
      const TPixelCoord& pt0,
      const TPixelCoord& pt1,
      const mrpt::img::TColor& color,
      int32_t width = 1,
      TPenStyle penStyle = psSolid);

  /** Draws a rectangle (an empty rectangle, without filling)
   * \param pt0 The top-left point
   * \param pt1 The right-bottom coordinate
   * \param color The color of the line
   * \param width The desired width of the line.
   * \sa filledRectangle
   */
  void rectangle(
      const TPixelCoord& pt0,
      const TPixelCoord& pt1,
      const mrpt::img::TColor& color,
      int32_t width = 1);

  /** Draws a triangle
   * \param pt The triangle center
   * \param size The size of the triangle
   * \param color The color of the line
   *	\param inferior The position of the triangle
   * \param width The desired width of the line.
   * \sa triangle
   */
  void triangle(
      const TPixelCoord& pt,
      int32_t size,
      mrpt::img::TColor color,
      bool inferior = true,
      int32_t width = 1);

  /** Draws a filled rectangle.
   * \param pt0 The top-left point
   * \param pt1 The right-bottom point
   * \param color The color of the rectangle fill
   *  This method may be redefined in some classes implementing this
   * interface in a more appropriate manner.
   * \sa rectangle
   */
  virtual void filledRectangle(
      const TPixelCoord& pt0, const TPixelCoord& pt1, mrpt::img::TColor color);

  /** Renders 2D text using bitmap fonts.
   * \param pt The point where the text is to be drawn
   * \param str The string to put. If using UNICODE characters, use UTF-8
   * encoding.
   * \param color The text color
   *
   * \sa selectTextFont
   */
  virtual void textOut(const TPixelCoord& pt, const std::string& str, mrpt::img::TColor color);

  /** Select the current font used when drawing text.
   * \param fontName The name of the font
   *
   *  Valid font names:
   *  - 5x7
   *  - 6x13
   *  - 6x13B
   *  - 6x13O
   *  - 9x15   (Default at start-up)
   *  - 9x15B
   *  - 10x20
   *  - 18x18ja (Asian characters for UTF-8 strings - Only available if MRPT
   * is built with MRPT_HAS_ASIAN_FONTS = true)
   *
   *   <img src="sample_textFonts.png" >
   *
   * \sa textOut, The example in <a
   * href="http://www.mrpt.org/Implemented_2D_Fonts">this page</a>.
   */
  virtual void selectTextFont(const std::string& fontName);

  /** Draws an image as a bitmap at a given position.
   * \param pt The top-left corner on this canvas where the image is to be drawn
   * \param img The image to be drawn in this canvas
   *  This method may be redefined in some classes implementing this
   * interface in a more appropriate manner.
   */
  virtual void drawImage(const TPixelCoord& pt, const mrpt::img::CImage& img);

  /** Draw a mark.
   * \param pt The point where the mark is to be drawn
   * \param color The color of the cross
   * \param size The size of the cross
   * \param type The cross type. It could be: 'x', '+', ':'(like '+' but
   * clear at the center dot), or 's' (square)
   * \param width The desired width of the cross (this is IGNORED yet)
   */
  void drawMark(
      const TPixelCoord& pt,
      const mrpt::img::TColor color,
      char type,
      int32_t size = 5,
      int32_t width = 1);

  /** Draws a circle of a given radius.
   * \param pt The center coordinate
   * \param radius The radius - in pixels.
   * \param color The color of the circle.
   * \param width The desired width of the line (this is IGNORED in this virtual class)
   */
  virtual void drawCircle(
      const TPixelCoord& center,
      int32_t radius,
      const mrpt::img::TColor& color = mrpt::img::TColor(255, 255, 255),
      int32_t width = 1);

  /** Draws an ellipse representing a given confidence interval of a 2D
   * Gaussian distribution.
   * \param mean_x The x coordinate of the center point of the ellipse.
   * \param mean_y The y coordinate of the center point of the ellipse.
   * \param cov2D A 2x2 covariance matrix.
   * \param confIntervalStds How many "sigmas" for the confidence level (i.e.
   * 2->95%, 3=99.97%,...)
   * \param color The color of the ellipse
   * \param width The desired width of the line (this is IGNORED in this
   * virtual class)
   * \param nEllipsePoints The number of points to generate to approximate
   * the ellipse shape.
   * \exception std::exception On an invalid matrix.
   */
  void ellipseGaussian(
      const mrpt::math::CMatrixFixed<double, 2, 2>& cov2D,
      const double mean_x,
      const double mean_y,
      double confIntervalStds = 2,
      const mrpt::img::TColor& color = mrpt::img::TColor(255, 255, 255),
      int32_t width = 1,
      int nEllipsePoints = 20);

  /** Draws a set of marks onto the image, given a generic container of
   * entities having just "x" and "y" fields.
   *  The class of FEATURELIST can be, for example,
   * std::vector<mrpt::math::TPoint2D>, std::vector<TPixelCoordsf> or
   * mrpt::vision::CFeatureList
   * \sa drawFeatures
   */
  template <class FEATURELIST>
  void drawFeaturesSimple(
      const FEATURELIST& list, const TColor& color = TColor::red(), const int cross_size = 5)
  {
    for (size_t i = 0; i < list.size(); ++i)
    {
      const int x = round(list.getFeatureX(i));
      const int y = round(list.getFeatureY(i));
      drawMark({x, y}, color, '+', cross_size);
    }
  }

  /** Draws a set of marks (or scaled circles for features with scale) onto
   * the image, given a generic container of features.
   *  The class of FEATURELIST can be:
   *    - mrpt::vision::CFeatureList
   *    - mrpt::vision::TKeyPointList
   *
   * \sa drawFeaturesSimple
   */
  template <class FEATURELIST>
  void drawFeatures(
      const FEATURELIST& list,
      const TColor& color = TColor::red(),
      const bool showIDs = false,
      const bool showResponse = false,
      const bool showScale = false,
      const char marker = '+')
  {
    for (size_t i = 0; i < list.size(); ++i)
    {
      const int x = round(list.getFeatureX(i));
      const int y = round(list.getFeatureY(i));
      drawMark({x, y}, color, marker);
      if (showIDs)
      {
        this->textOut(
            {x, y}, format("%u", static_cast<unsigned int>(list.getFeatureID(i))), TColor::red());
      }
      if (showResponse)
      {
        this->textOut(
            {x, y + 10}, format("R:%u", static_cast<unsigned int>(list.getFeatureResponse(i))),
            TColor::red());
      }
      if (!list.isPointFeature(i))
      {
        this->drawCircle({x, y}, list.getScale(i), TColor::red());
      }
      else if (showScale)
      {
        this->textOut({x, y + 20}, format("S:%.01f", list.getScale(i)), TColor::red());
      }
    }
  }
};  // End of class

}  // namespace mrpt::img
