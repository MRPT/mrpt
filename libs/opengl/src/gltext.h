/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Stripped out version of libCVD gl-helpers.h file, ported to OpenGL>=3
// 2-BSD License.

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/opengl_fonts.h>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace mrpt::opengl::internal
{
/** @name OpenGL vector 3D fonts
	@{ */

/// sets the font to use for future font rendering commands.
/// Options are:  "sans", "serif", "mono".
/// @param fontname string containing font name
void glSetFont(const std::string& fontname);

/// returns the name of the currently active font
const std::string& glGetFont();

/// different style for font rendering
using TEXT_STYLE = TOpenGLFontStyle;

/// renders a string in GL using the current settings.
/// Font coordinates are +X along the line and +Y along the up direction of
/// glyphs. The origin is at the top baseline at the left of the first
/// character. Characters have a maximum size of 1. linefeed is interpreted as a
/// new line and the start is offset in -Y direction by @ref spacing .
/// Individual characters are separated by @ref kerning + plus their individual
/// with.
/// @param text string to be rendered, unknown characters are replaced with '?'
/// @param style rendering style
/// @param spacing distance between individual text lines
/// @param kerning distance between characters
std::pair<double, double> glDrawText(
	const std::string& text, std::vector<mrpt::opengl::TTriangle>& tris,
	std::vector<mrpt::math::TPoint3Df>& lines, TEXT_STYLE style = NICE,
	double spacing = 1.5, double kerning = 0.1);

/// returns the size of the bounding box of a text to be rendered, similar to
/// @ref glDrawText but without any visual output
std::pair<double, double> glGetExtends(
	const std::string& text, double spacing = 1.5, double kerning = 0.1);

/** @} */

}  // namespace mrpt::opengl::internal
