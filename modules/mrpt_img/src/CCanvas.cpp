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

#include <mrpt/core/reverse_bytes.h>
#include <mrpt/core/round.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/zip.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <Eigen/Dense>
#include <cstring>  // memcpy
#include <map>

// Include the MRPT bitmap fonts:
#include "mrpt_font_10x20.h"
#include "mrpt_font_5x7.h"
#include "mrpt_font_6x13.h"
#include "mrpt_font_6x13B.h"
#include "mrpt_font_6x13O.h"
#include "mrpt_font_9x15.h"
#include "mrpt_font_9x15B.h"

// Japanese fonts?
#if MRPT_HAS_ASIAN_FONTS
#include "mrpt_font_18x18ja.h"
#endif

// Each font has a block a data with this header (It's actually zip-compressed
// since mrpt >0.6.5)
//	const uint32_t mrpt_font_9x15B [] = {
//	9,15, /* width, height */
//	0x0000,0x00FF, /* UNICODE characters range: */

using namespace mrpt;
using namespace mrpt::img;
using namespace std;

struct FontData
{
  std::vector<uint8_t> data;
  bool prepared_to_big_endian = false;
};

// Each vector is the target place where to uncompress each font.
map<string, FontData> list_registered_fonts;
bool list_fonts_init = false;

void init_fonts_list()
{
  if (!list_fonts_init)
  {
    list_registered_fonts.clear();

// This was used only once
#if 0
#define SAVE_COMPRESSED(ARR)                                                           \
  {                                                                                    \
    list_registered_fonts[#ARR].resize(sizeof(mrpt_font_##ARR));                       \
    memcpy(&list_registered_fonts[#ARR][0], mrpt_font_##ARR, sizeof(mrpt_font_##ARR)); \
    cout << #ARR << " -> " << sizeof(mrpt_font_##ARR) << "\n";                         \
    CCompressedOutputStream f(string("mrpt_font_") + string(#ARR) + string(".gz"));    \
    f.WriteBuffer(mrpt_font_##ARR, sizeof(mrpt_font_##ARR));                           \
    /*mrpt::compress::zip::compress( list_registered_fonts[#ARR], f ); */              \
  }

    	SAVE_COMPRESSED(5x7)
//    	SAVE_COMPRESSED(6x13)
//    	SAVE_COMPRESSED(6x13B)
//    	SAVE_COMPRESSED(6x13O)
//    	SAVE_COMPRESSED(9x15)
//    	SAVE_COMPRESSED(9x15B)
//    	SAVE_COMPRESSED(10x20)

#if MRPT_HAS_ASIAN_FONTS
//    	SAVE_COMPRESSED(18x18ja)
#endif

#endif

#if 1  // Normal operation: Load fonts and uncompress them:

#define LOAD_FONT(FONTNAME)                                                                 \
  {                                                                                         \
    std::vector<uint8_t> tmpBuf(sizeof(mrpt_font_gz_##FONTNAME));                           \
    memcpy(&tmpBuf[0], mrpt_font_gz_##FONTNAME, sizeof(mrpt_font_gz_##FONTNAME));           \
    mrpt::io::zip::decompress_gz_data_block(tmpBuf, list_registered_fonts[#FONTNAME].data); \
  }

    LOAD_FONT(5x7)
    LOAD_FONT(6x13)
    LOAD_FONT(6x13B)
    LOAD_FONT(6x13O)
    LOAD_FONT(9x15)
    LOAD_FONT(9x15B)
    LOAD_FONT(10x20)
#if MRPT_HAS_ASIAN_FONTS
    LOAD_FONT(18x18ja)
#endif

#endif

    list_fonts_init = true;
  }
}

/*---------------------------------------------------------------
            line
---------------------------------------------------------------*/
void CCanvas::line(
    const TPixelCoord& pt0,
    const TPixelCoord& pt1,
    const mrpt::img::TColor& color,
    [[maybe_unused]] int32_t width,
    [[maybe_unused]] TPenStyle penStyle)
{
  const auto delta = TPixelCoordf(pt1) - TPixelCoordf(pt0);

  const auto img_width = getWidth();
  const auto img_height = getHeight();

  // In these cases, there is nothing to do!
  if (delta.x == 0 && delta.y == 0)
  {
    return;
  }
  if (pt0.x < 0 && pt1.x < 0)
  {
    return;
  }
  if (pt0.y < 0 && pt1.y < 0)
  {
    return;
  }
  if (pt0.x >= img_width && pt1.x >= img_width)
  {
    return;
  }
  if (pt0.y >= img_height && pt1.y >= img_height)
  {
    return;
  }

  const float dist = std::sqrt(square(delta.x) + square(delta.y));
  const auto Nf = std::ceil(dist);
  const auto N = static_cast<int>(Nf);

  // The N steps to perform next:
  const auto step = TPixelCoordf(delta.x / Nf, delta.y / Nf);

  auto pt = TPixelCoordf(pt0);

  for (int i = 0; i < N; i++)
  {
    setPixel(TPixelCoord{pt}, color);
    pt.x += step.x;
    pt.y += step.y;
  }
}

/*---------------------------------------------------------------
            rectangle
---------------------------------------------------------------*/
void CCanvas::rectangle(
    const TPixelCoord& pt0, const TPixelCoord& pt1, const mrpt::img::TColor& color, int32_t width)
{
  const auto img_width = getWidth();
  const auto img_height = getHeight();

  // Clip to the image:
  int x_min = max(pt0.x, 0);
  int x_max = min(pt1.x, img_width - 1);
  int y_min = max(pt0.y, 0);
  int y_max = min(pt1.y, img_height - 1);

  // Draw the four lines:
  line({x_min, y_min}, {x_max, y_min}, color, width);
  line({x_max, y_min}, {x_max, y_max}, color, width);
  line({x_max, y_max}, {x_min, y_max}, color, width);
  line({x_min, y_max}, {x_min, y_min}, color, width);
}

void CCanvas::triangle(
    const TPixelCoord& pt,
    int32_t size,
    const mrpt::img::TColor color,
    bool inferior,
    int32_t width)
{
  const int ts = round(0.866 * size);
  const int tc = round(0.5 * size);
  if (inferior)
  {
    line({pt.x, pt.y + size}, {pt.x + ts, pt.y - tc}, color, width);
    line({pt.x, pt.y + size}, {pt.x - ts, pt.y - tc}, color, width);
    line({pt.x + ts, pt.y - tc}, {pt.x - ts, pt.y - tc}, color, width);
  }
  else
  {
    line({pt.x, pt.y - size}, {pt.x + ts, pt.y + tc}, color, width);
    line({pt.x, pt.y - size}, {pt.x - ts, pt.y + tc}, color, width);
    line({pt.x + ts, pt.y + tc}, {pt.x - ts, pt.y + tc}, color, width);
  }
}

/*---------------------------------------------------------------
            filledRectangle
---------------------------------------------------------------*/
void CCanvas::filledRectangle(
    const TPixelCoord& pt0, const TPixelCoord& pt1, const mrpt::img::TColor color)
{
  const int x0 = std::max(0, std::min(pt0.x, pt1.x));
  const int y0 = std::max(0, std::min(pt0.y, pt1.y));
  const int x1 = std::min(getWidth() - 1, std::max(pt0.x, pt1.x));
  const int y1 = std::min(getHeight() - 1, std::max(pt0.y, pt1.y));

  for (int y = y0; y <= y1; y++)
  {
    for (int x = x0; x <= x1; x++)
    {
      setPixel({x, y}, color);
    }
  }
}

/*---------------------------------------------------------------
          selectTextFont
---------------------------------------------------------------*/
void CCanvas::selectTextFont(const std::string& fontName)
{
  init_fonts_list();

  // Assure list name is in the list:
  auto it = list_registered_fonts.find(fontName);
  if (it == list_registered_fonts.end())
  {
    // Error
    cerr << "[CCanvas::selectTextFont] Warning: Unknown font: " << fontName << "\n";
    return;
  }

  FontData& fd = it->second;
  m_selectedFontBitmaps = reinterpret_cast<const uint32_t*>(&fd.data[0]);
  m_selectedFont = fontName;

#if MRPT_IS_BIG_ENDIAN
  // Fix endianness of char tables:
  if (!fd.prepared_to_big_endian)
  {
    fd.prepared_to_big_endian = true;  // Only do once
    uint32_t* ptr = reinterpret_cast<uint32_t*>(&fd.data[0]);
    for (size_t i = 0; i < fd.data.size() / sizeof(uint32_t); i++)
      mrpt::reverseBytesInPlace(ptr[i]);
  }
#endif
}

/*---------------------------------------------------------------
            drawImage
---------------------------------------------------------------*/
void CCanvas::drawImage(const TPixelCoord& pt, const mrpt::img::CImage& img)
{
  MRPT_START
  ASSERT_(img.getPixelDepth() == mrpt::img::PixelDepth::D8U);

  const auto img_lx = img.getWidth();
  const auto img_ly = img.getHeight();

  if (img.isColor())
  {
    for (int32_t xx = 0; xx < img_lx; xx++)
    {
      for (int32_t yy = 0; yy < img_ly; yy++)
      {
        auto ptr = img.ptr<uint8_t>(xx, yy);
        setPixel({pt.x + xx, pt.y + yy}, {ptr[0], ptr[1], ptr[2]});
      }
    }
  }
  else
  {
    for (int32_t xx = 0; xx < img_lx; xx++)
    {
      for (int32_t yy = 0; yy < img_ly; yy++)
      {
        const auto c = img.at<uint8_t>(xx, yy);
        setPixel({pt.x + xx, pt.y + yy}, {c, c, c});
      }
    }
  }

  MRPT_END
}

void CCanvas::drawMark(
    const TPixelCoord& pt,
    const mrpt::img::TColor color,
    char type,  // NOLINT
    int32_t size,
    int32_t width)
{
  switch (type)
  {
    case '+':
      line({pt.x - size, pt.y}, {pt.x + size, pt.y}, color, width);
      line({pt.x, pt.y - size}, {pt.x, pt.y + size}, color, width);
      break;
    case 's':
      line({pt.x - size, pt.y - size}, {pt.x + size, pt.y - size}, color, width);
      line({pt.x + size, pt.y - size}, {pt.x + size, pt.y + size}, color, width);
      line({pt.x - size, pt.y + size}, {pt.x + size, pt.y + size}, color, width);
      line({pt.x - size, pt.y - size}, {pt.x - size, pt.y + size}, color, width);
      break;
    case 'x':
      line({pt.x - size, pt.y - size}, {pt.x + size, pt.y + size}, color, width);
      line({pt.x + size, pt.y - size}, {pt.x - size, pt.y + size}, color, width);
      break;
    case ':':
      line({pt.x - size, pt.y}, {pt.x - 2, pt.y}, color, width);
      line({pt.x + 2, pt.y}, {pt.x + size, pt.y}, color, width);
      line({pt.x, pt.y - size}, {pt.x, pt.y - 2}, color, width);
      line({pt.x, pt.y + 2}, {pt.x, pt.y + size}, color, width);
      break;
    default:
      THROW_EXCEPTION("Unexpected 'type' of cross to be drawn");
  }
}

/*---------------------------------------------------------------
            drawCircle
---------------------------------------------------------------*/
void CCanvas::drawCircle(
    const TPixelCoord& center, int32_t radius, const mrpt::img::TColor& color, int32_t width)
{
  if (radius < 0)
  {
    radius = -radius;
  }

  int nSegments;

  if (radius == 0)
  {
    nSegments = 2;
  }
  else
  {
    nSegments = int(M_2PI * radius);
  }

  int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  double ang, Aa = M_2PI / (nSegments - 1);
  int i;

  for (i = 0, ang = 0; i < nSegments; i++, ang += Aa)
  {
    x2 = round(center.x + radius * cos(ang));
    y2 = round(center.y + radius * sin(ang));

    if (i > 0)
    {
      line({x1, y1}, {x2, y2}, color, width);
    }

    x1 = x2;
    y1 = y2;
  }  // end for points on ellipse
}

/*---------------------------------------------------------------
            textOut
---------------------------------------------------------------*/
void CCanvas::textOut(const TPixelCoord& pt, const std::string& str, const mrpt::img::TColor color)
{
  MRPT_START

  if (m_selectedFontBitmaps == nullptr)
  {  // First call: load fonts
    this->selectTextFont("9x15");
  }

  // Decode UNICODE string:
  std::vector<uint16_t> uniStr;
  mrpt::system::decodeUTF8(str, uniStr);

  int px = pt.x;
  int py = pt.y;

  // Char size:
  const auto char_w = m_selectedFontBitmaps[0];
  const auto char_h = m_selectedFontBitmaps[1];

  for (unsigned short unichar : uniStr)
  {
    // look for the character in the table:
    const uint32_t* table_ptr = m_selectedFontBitmaps + 2;
    uint32_t charset_ini = table_ptr[0];
    uint32_t charset_end = table_ptr[1];

    while (charset_end != 0)
    {
      // Is in this range?
      if (unichar <= charset_end && unichar >= charset_ini)
      {
        // Draw this character:
        int pyy = py;

        const uint32_t* char_bitmap =
            table_ptr + 2 + (static_cast<size_t>(char_h) * (unichar - charset_ini));

        for (int y = 0; y < static_cast<int>(char_h); y++, pyy += 1)
        {
          // Use memcpy() here since directly dereferencing is an
          // invalid operation in architectures (S390X) where
          // unaligned accesses are forbiden:
          uint32_t row;
          memcpy(&row, char_bitmap, sizeof(row));
          char_bitmap++;
          for (int x = 0, pxx = px; x < static_cast<int>(char_w); x++, pxx++)
          {
            if (!!(row & (1 << x)))
            {
              setPixel({pxx, pyy}, color);
            }
          }
        }

        // Advance the raster cursor:
        px += static_cast<int>(char_w);

        // Next char!
        break;
      }

      // No: Move to the next block and keep searching:
      uint32_t n_chars = charset_end - charset_ini + 1;
      table_ptr += 2 /* Header */ + n_chars * char_h;

      // get new block header:
      charset_ini = table_ptr[0];
      charset_end = table_ptr[1];
    }
    // Char not in the font!
  }

  MRPT_END
}

void CCanvas::ellipseGaussian(
    const mrpt::math::CMatrixFixed<double, 2, 2>& cov2D,
    const double mean_x,
    const double mean_y,
    double confIntervalStds,
    const mrpt::img::TColor& color,
    int32_t width,  // NOLINT
    int nEllipsePoints)
{
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  double ang;
  mrpt::math::CMatrixFixed<double, 2, 2> eigVec, eigVals;
  std::vector<double> eVals;
  int i;

  // Compute the eigen-vectors & values:
  cov2D.eig(eigVec, eVals);
  eigVals.setDiagonal(eVals);

  eigVals.asEigen() = eigVals.array().sqrt().matrix();

  mrpt::math::CMatrixFixed<double, 2, 2> M;
  M.asEigen() = eigVals.asEigen() * eigVec.transpose();

  // Compute the points of the 2D ellipse:
  for (i = 0, ang = 0; i < nEllipsePoints; i++, ang += (M_2PI / (nEllipsePoints - 1)))
  {
    double ccos = cos(ang);
    double ssin = sin(ang);

    x2 = round(mean_x + confIntervalStds * (ccos * M(0, 0) + ssin * M(1, 0)));
    y2 = round(mean_y + confIntervalStds * (ccos * M(0, 1) + ssin * M(1, 1)));

    if (i > 0)
    {
      line({x1, y1}, {x2, y2}, color, width);
    }

    x1 = x2;
    y1 = y2;
  }  // end for points on ellipse
}
