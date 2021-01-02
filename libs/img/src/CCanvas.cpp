/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

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
#define SAVE_COMPRESSED(ARR)                                                  \
	{                                                                         \
		list_registered_fonts[#ARR].resize(sizeof(mrpt_font_##ARR));          \
		memcpy(                                                               \
			&list_registered_fonts[#ARR][0], mrpt_font_##ARR,                 \
			sizeof(mrpt_font_##ARR));                                         \
		cout << #ARR << " -> " << sizeof(mrpt_font_##ARR) << endl;            \
		CFileGZOutputStream f(                                                \
			string("mrpt_font_") + string(#ARR) + string(".gz"));             \
		f.WriteBuffer(mrpt_font_##ARR, sizeof(mrpt_font_##ARR));              \
		/*mrpt::compress::zip::compress( list_registered_fonts[#ARR], f ); */ \
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

#define LOAD_FONT(FONTNAME)                                           \
	{                                                                 \
		std::vector<uint8_t> tmpBuf(sizeof(mrpt_font_gz_##FONTNAME)); \
		memcpy(                                                       \
			&tmpBuf[0], mrpt_font_gz_##FONTNAME,                      \
			sizeof(mrpt_font_gz_##FONTNAME));                         \
		mrpt::io::zip::decompress_gz_data_block(                      \
			tmpBuf, list_registered_fonts[#FONTNAME].data);           \
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
	int x0, int y0, int x1, int y1, const mrpt::img::TColor color,
	[[maybe_unused]] unsigned int width, [[maybe_unused]] TPenStyle penStyle)
{
	float x, y;

	auto Ax = (float)(x1 - x0);
	auto Ay = (float)(y1 - y0);

	// In this cases, there is nothing to do!
	if (Ax == 0 && Ay == 0) return;
	if (x0 < 0 && x1 < 0) return;
	if (y0 < 0 && y1 < 0) return;
	if (x0 >= (int)getWidth() && x1 >= (int)getWidth()) return;
	if (y0 >= (int)getHeight() && y1 >= (int)getHeight()) return;

	float dist = sqrt(square(Ax) + square(Ay));
	int i, N = (int)ceil(dist);

	// The N steps to perform next:
	Ax /= N;
	Ay /= N;
	x = (float)x0;
	y = (float)y0;

	for (i = 0; i < N; i++)
	{
		x += Ax;
		y += Ay;
		setPixel((int)x, (int)y, color);
	}  // end for i
}

/*---------------------------------------------------------------
						rectangle
---------------------------------------------------------------*/
void CCanvas::rectangle(
	int x0, int y0, int x1, int y1, const mrpt::img::TColor color,
	unsigned int width)
{
	int w_min = (int)-ceil(((float)width) / 2);
	int w_max = (int)floor(((float)width) / 2);
	// Draw "width" rectangles one into another:
	for (int w = w_min; w <= w_max; w++)
	{
		line(x0 - w, y0 - w, x1 + w, y0 - w, color);
		line(x1 + w, y0 - w, x1 + w, y1 + w, color);
		line(x1 + w, y1 + w, x0 - w, y1 + w, color);
		line(x0 - w, y1 + w, x0 - w, y0 - w, color);
	}  // end for "w"
}

/*****************************************************AJOGD***************************************************/
/*---------------------------------------------------------------
						triangle
---------------------------------------------------------------*/
void CCanvas::triangle(
	int x0, int y0, int size, const mrpt::img::TColor color, bool inferior,
	unsigned int width)
{
	int ts = round(0.866 * size);
	int tc = round(0.5 * size);
	if (inferior)
	{
		line(x0, y0 + size, x0 + ts, y0 - tc, color, width);
		line(x0, y0 + size, x0 - ts, y0 - tc, color, width);
		line(x0 + ts, y0 - tc, x0 - ts, y0 - tc, color, width);
	}
	else
	{
		line(x0, y0 - size, x0 + ts, y0 + tc, color, width);
		line(x0, y0 - size, x0 - ts, y0 + tc, color, width);
		line(x0 + ts, y0 + tc, x0 - ts, y0 + tc, color, width);
	}
}
/************************************************************************************************************/

/*---------------------------------------------------------------
						filledRectangle
---------------------------------------------------------------*/
void CCanvas::filledRectangle(
	int x0, int y0, int x1, int y1, const mrpt::img::TColor color)
{
	int x_min = max(x0, 0);
	int x_max = min(x1, (int)getWidth() - 1);
	int y_min = max(y0, 0);
	int y_max = min(y1, (int)getHeight() - 1);

	for (int y = y_min; y <= y_max; y++)
		for (int x = x_min; x <= x_max; x++) setPixel(x, y, color);
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
		cerr << "[CCanvas::selectTextFont] Warning: Unknown font: " << fontName
			 << endl;
		return;
	}
	else
	{
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
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void CCanvas::drawImage(int x, int y, const mrpt::img::CImage& img)
{
	MRPT_START
	ASSERT_(img.getPixelDepth() == mrpt::img::PixelDepth::D8U);

	int img_lx = img.getWidth();
	int img_ly = img.getHeight();

	if (img.isColor())
	{
		for (int xx = 0; xx < img_lx; xx++)
			for (int yy = 0; yy < img_ly; yy++)
			{
				auto ptr = img(xx, yy);
				const int p = ptr[0] | (ptr[1] << 8) | (ptr[2] << 16);
				setPixel(x + xx, y + yy, p);
			}
	}
	else
	{
		unsigned char c;
		int col;
		for (int xx = 0; xx < img_lx; xx++)
			for (int yy = 0; yy < img_ly; yy++)
			{
				c = *((unsigned char*)img(xx, yy));
				col = c | (c << 8) | (c << 16);
				setPixel(x + xx, y + yy, col);
			}
	}

	MRPT_END
}

/*---------------------------------------------------------------
						drawImage
---------------------------------------------------------------*/
void CCanvas::drawImage(
	[[maybe_unused]] int x, [[maybe_unused]] int y,
	[[maybe_unused]] const mrpt::img::CImage& img,
	[[maybe_unused]] float rotation, [[maybe_unused]] float scale)
{
	MRPT_START

	THROW_EXCEPTION("Not implemented yet!! Try yourself! ;-)");

	MRPT_END
}

void CCanvas::drawMark(
	int x0, int y0, const mrpt::img::TColor color, char type, int size,
	unsigned int width)
{
	switch (type)
	{
		case '+':
			line(x0 - size, y0, x0 + size, y0, color, width);
			line(x0, y0 - size, x0, y0 + size, color, width);
			break;
		case 's':
			line(x0 - size, y0 - size, x0 + size, y0 - size, color, width);
			line(x0 + size, y0 - size, x0 + size, y0 + size, color, width);
			line(x0 - size, y0 + size, x0 + size, y0 + size, color, width);
			line(x0 - size, y0 - size, x0 - size, y0 + size, color, width);
			break;
		case 'x':
			line(x0 - size, y0 - size, x0 + size, y0 + size, color, width);
			line(x0 + size, y0 - size, x0 - size, y0 + size, color, width);
			break;
		case ':':
			line(x0 - size, y0, x0 - 2, y0, color, width);
			line(x0 + 2, y0, x0 + size, y0, color, width);
			line(x0, y0 - size, x0, y0 - 2, color, width);
			line(x0, y0 + 2, x0, y0 + size, color, width);
			break;
		default:
			THROW_EXCEPTION("Unexpected 'type' of cross to be drawn");
	}
}

/*---------------------------------------------------------------
						drawCircle
---------------------------------------------------------------*/
void CCanvas::drawCircle(
	int x, int y, int radius, const mrpt::img::TColor& color,
	unsigned int width)
{
	if (radius < 0) radius = -radius;

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
		x2 = round(x + radius * cos(ang));
		y2 = round(y + radius * sin(ang));

		if (i > 0) line(x1, y1, x2, y2, color, width);

		x1 = x2;
		y1 = y2;
	}  // end for points on ellipse
}

/*---------------------------------------------------------------
						textOut
---------------------------------------------------------------*/
void CCanvas::textOut(
	int x0, int y0, const std::string& str, const mrpt::img::TColor color)
{
	MRPT_START

	if (!m_selectedFontBitmaps)  // First call: load fonts
		this->selectTextFont("9x15");

	// Am I an image?
	bool y_axis_reversed = false;
	auto* im_image = dynamic_cast<CImage*>(this);
	if (im_image) y_axis_reversed = !im_image->isOriginTopLeft();

	// Decode UNICODE string:
	std::vector<uint16_t> uniStr;
	mrpt::system::decodeUTF8(str, uniStr);

	int px = x0;
	int py = y0;

	// Char size:
	int char_w = m_selectedFontBitmaps[0];
	int char_h = m_selectedFontBitmaps[1];

	for (unsigned short unichar : uniStr)
	{
		// look for the character in the table:
		const uint32_t* table_ptr = m_selectedFontBitmaps + 2;
		uint32_t charset_ini = table_ptr[0];
		uint32_t charset_end = table_ptr[1];

		while (charset_end)
		{
			// Is in this range?
			if (unichar <= charset_end && unichar >= charset_ini)
			{
				// Draw this character:
				int pyy = y_axis_reversed ? (py + char_h - 1) : py;

				const uint32_t* char_bitmap =
					table_ptr + 2 + char_h * (unichar - charset_ini);

				for (int y = 0; y < char_h;
					 y++, pyy += y_axis_reversed ? -1 : 1)
				{
					// Use memcpy() here since directly dereferencing is an
					// invalid operation in architectures (S390X) where
					// unaligned accesses are forbiden:
					uint32_t row;
					memcpy(&row, char_bitmap, sizeof(row));
					char_bitmap++;
					for (int x = 0, pxx = px; x < char_w; x++, pxx++)
						if (!!(row & (1 << x))) setPixel(pxx, pyy, color);
				}

				// Advance the raster cursor:
				px += char_w;

				// Next char!
				break;
			}
			else
			{
				// No: Move to the next block and keep searching:
				uint32_t n_chars = charset_end - charset_ini + 1;
				table_ptr += 2 /* Header */ + n_chars * char_h;

				// get new block header:
				charset_ini = table_ptr[0];
				charset_end = table_ptr[1];
			}
		}
		// Char not in the font!
	}

	MRPT_END
}

void CCanvas::ellipseGaussian(
	const mrpt::math::CMatrixFixed<double, 2, 2>& cov2D, const double mean_x,
	const double mean_y, double confIntervalStds,
	const mrpt::img::TColor& color, unsigned int width, int nEllipsePoints)
{
	MRPT_START
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
	for (i = 0, ang = 0; i < nEllipsePoints;
		 i++, ang += (M_2PI / (nEllipsePoints - 1)))
	{
		double ccos = cos(ang);
		double ssin = sin(ang);

		x2 = round(
			mean_x + confIntervalStds * (ccos * M(0, 0) + ssin * M(1, 0)));
		y2 = round(
			mean_y + confIntervalStds * (ccos * M(0, 1) + ssin * M(1, 1)));

		if (i > 0) line(x1, y1, x2, y2, color, width);

		x1 = x2;
		y1 = y2;
	}  // end for points on ellipse

	MRPT_END_WITH_CLEAN_UP(std::cout << "Covariance matrix leading to error is:"
									 << std::endl
									 << cov2D << std::endl;);
}
