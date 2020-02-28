/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

// Stripped out version of libCVD gltext.cpp file, ported to OpenGL>=3
// 2-BSD License.

#include "gltext.h"

#include <mrpt/math/TPoint2D.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>

using namespace std;

namespace mrpt::opengl::internal
{
struct Point
{
	float x, y;
};

struct Font
{
	typedef unsigned short Index;

	struct Char
	{
		Index vertexOffset;
		Index triangleOffset;
		Index outlineOffset;
		int numTriangles;  // was: GLsizei
		int numOutlines;
		float advance;
	};

	Point* vertices;
	Index* triangles;
	Index* outlines;
	Char* characters;
	string glyphs;

	const Char* findChar(const char c) const
	{
		size_t ind = glyphs.find(c);
		if (ind == string::npos) return nullptr;
		return characters + ind;
	}

	float getAdvance(const char c) const
	{
		const Char* ch = findChar(c);
		if (!ch) return 0;
		return ch->advance;
	}

	void fill(
		const char c, std::vector<mrpt::opengl::TTriangle>& tris,
		std::vector<mrpt::math::TPoint3Df>& lines,
		const mrpt::math::TPoint2Df& raster_pos) const
	{
		const Char* ch = findChar(c);
		if (!ch || !ch->numTriangles) return;
		/* Was:
		 * glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
		 * glDrawElements(
		 * GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT,
		 * triangles + ch->triangleOffset);
		 */
	}

	void outline(
		const char c, std::vector<mrpt::opengl::TTriangle>& tris,
		std::vector<mrpt::math::TPoint3Df>& lines,
		const mrpt::math::TPoint2Df& raster_pos) const
	{
		const Char* ch = findChar(c);
		if (!ch || !ch->numOutlines) return;
		/* Was:
		 * glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
		 * glDrawElements(
		 * GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT,
		 * outlines + ch->outlineOffset);
		 */
		MRPT_TODO("Continue here");
		// lines
	}

	void draw(
		const char c, std::vector<mrpt::opengl::TTriangle>& tris,
		std::vector<mrpt::math::TPoint3Df>& lines,
		const mrpt::math::TPoint2Df& raster_pos) const
	{
		outline(c, tris, lines, raster_pos);
		fill(c, tris, lines, raster_pos);
	}
};

// the fonts defined in these headers are derived from Bitstream Vera fonts. See
// http://www.gnome.org/fonts/ for license and details
#include "glfont_mono.h"
#include "glfont_sans.h"
#include "glfont_serif.h"

struct FontData
{
	typedef map<string, Font*> FontMap;

	FontData()
	{
		fonts["sans"] = &sans_font;
		fonts["mono"] = &mono_font;
		fonts["serif"] = &serif_font;
		glSetFont("sans");
	}
	inline Font* currentFont() { return fonts[currentFontName]; }

	string currentFontName;
	FontMap fonts;
};

static struct FontData data;

void glSetFont(const std::string& fontname)
{
	if (data.fonts.count(fontname) > 0) data.currentFontName = fontname;
}

const std::string& glGetFont() { return data.currentFontName; }

std::pair<double, double> glDrawText(
	const std::string& text, std::vector<mrpt::opengl::TTriangle>& tris,
	std::vector<mrpt::math::TPoint3Df>& render_lines, enum TEXT_STYLE style,
	double spacing, double kerning)
{
	// Was: glPushMatrix();
	mrpt::math::TPoint2Df raster_pos = {0, 0};

	// figure out which operation to do on the Char (yes, this is a pointer to
	// member function :)
	void (Font::*operation)(
		const char c, std::vector<mrpt::opengl::TTriangle>& tris,
		std::vector<mrpt::math::TPoint3Df>& lines,
		const mrpt::math::TPoint2Df& raster_pos) const;
	switch (style)
	{
		case FILL:
			operation = &Font::fill;
			break;
		case OUTLINE:
			operation = &Font::outline;
			break;
		case NICE:
			operation = &Font::draw;
			break;
	};

	int lines = 0;
	double max_total = 0;
	double total = 0;
	const Font* font = data.currentFont();
	const Font::Char* space = font->findChar(' ');
	const double tab_width = 8 * ((space) ? (space->advance) : 1);
	for (size_t i = 0; i < text.length(); ++i)
	{
		char c = text[i];
		if (c == '\n')
		{
			raster_pos.x -= total;
			raster_pos.y -= spacing;

			max_total = std::max(max_total, total);
			total = 0;
			++lines;
			continue;
		}
		if (c == '\t')
		{
			const float advance = tab_width - std::fmod(total, tab_width);
			total += advance;
			raster_pos.x += advance;
			continue;
		}
		const Font::Char* ch = font->findChar(c);
		if (!ch)
		{
			c = toupper(c);
			ch = font->findChar(c);
			if (!ch)
			{
				c = '?';
				ch = font->findChar(c);
			}
		}
		if (!ch) continue;
		(font->*operation)(c, tris, render_lines, raster_pos);

		double w = ch->advance + kerning;
		raster_pos.x += w;
		total += w;
	}

	max_total = std::max(total, max_total);
	return std::make_pair(max_total, (lines + 1) * spacing);
}

std::pair<double, double> glGetExtends(
	const std::string& text, double spacing, double kerning)
{
	int lines = 0;
	double max_total = 0;
	double total = 0;
	const Font* font = data.currentFont();
	for (size_t i = 0; i < text.length(); ++i)
	{
		char c = text[i];
		if (c == '\n')
		{
			max_total = std::max(max_total, total);
			total = 0;
			++lines;
			continue;
		}
		const Font::Char* ch = font->findChar(c);
		if (!ch)
		{
			c = toupper(c);
			ch = font->findChar(c);
			if (!ch)
			{
				c = '?';
				ch = font->findChar(c);
			}
		}
		if (!ch) continue;
		total += ch->advance + kerning;
	}
	max_total = std::max(total, max_total);
	return std::make_pair(max_total, (lines + 1) * spacing);
}

}  // namespace mrpt::opengl::internal
