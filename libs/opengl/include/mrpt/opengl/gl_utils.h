/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/opengl_fonts.h>

namespace mrpt
{
namespace opengl
{
/** A set of auxiliary functions that can be called to render OpenGL primitives
 * from MRPT or user code
 * \ingroup mrpt_opengl_grp
 */
namespace gl_utils
{
/** @name Miscellaneous rendering methods
	@{ */

/** For each object in the list:
 *   - checks visibility of each object
 *   - update the MODELVIEW matrix according to its coordinates
 *   - call its ::render()
 *   - shows its name (if enabled).
 *
 *  \note Used by  COpenGLViewport, CSetOfObjects
 */
void renderSetOfObjects(
	const mrpt::opengl::CListOpenGLObjects& objs,
	const mrpt::opengl::TRenderMatrices& state, mrpt::opengl::Program& shaders);

/** Checks glGetError and throws an exception if an error situation is found */
void checkOpenGLErr_impl(int glErrorCode, const char* filename, int lineno);

/** Can be used by derived classes to draw a triangle with a normal vector
 * computed automatically - to be called within a glBegin()-glEnd() block.
 */
void renderTriangleWithNormal(
	const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2,
	const mrpt::math::TPoint3D& p3);
void renderTriangleWithNormal(
	const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
	const mrpt::math::TPoint3Df& p3);

/** Can be used by derived classes to draw a quad with a normal vector computed
 * automatically - to be called within a glBegin()-glEnd() block. */
void renderQuadWithNormal(
	const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
	const mrpt::math::TPoint3Df& p3, const mrpt::math::TPoint3Df& p4);

/** Draws a message box with a centered (possibly multi-lined) text.
 *  It consists of a filled rectangle with a frame around and the centered text
 * in the middle.
 *
 *  The appearance of the box is highly configurable via parameters.
 *
 *   \param[in] msg_x, msg_y The left-lower corner coordinates, in ratio [0,1]
 * of the viewport size. (0,0) if the left-bottom corner of the viewport.
 *   \param[in] msg_w, msg_h The width & height, in ratio [0,1] of the viewport
 * size.
 *   \param[in] text  The text to display. Multiple lines can be drawn with the
 * '\n' character.
 *   \param[in] text_scale Size of characters, in ration [0,1] of the viewport
 * size. Note that this size may be scaled automatically reduced to fit the text
 * withtin the rectangle of the message box.
 *   \param[in] back_col Color of the rectangle background. Alpha can be <255
 * to enable transparency.
 *   \param[in] border_col Color of the rectangle frame. Alpha can be <255 to
 * enable transparency.
 *   \param[in] text_col Color of the text background. Alpha can be <255 to
 * enable transparency.
 *   \param[in] border_width Width of the border, in pixels
 *   \param[in] text_font, text_style, text_spacing, text_kerning See
 * mrpt::opengl::gl_utils::glDrawText()
 *
 *  Example (see directory: 'samples/display3D_custom_render'):
 *
 *  <img src="gl_utils_message_box.jpg" >
 */
void renderMessageBox(
	const float msg_x, const float msg_y, const float msg_w, const float msg_h,
	const std::string& text, float text_scale,
	const mrpt::img::TColor& back_col = mrpt::img::TColor(0, 0, 50, 150),
	const mrpt::img::TColor& border_col = mrpt::img::TColor(0, 0, 0, 140),
	const mrpt::img::TColor& text_col = mrpt::img::TColor(255, 255, 255, 220),
	const float border_width = 4.0f,
	const std::string& text_font = std::string("sans"),
	mrpt::opengl::TOpenGLFontStyle text_style = mrpt::opengl::FILL,
	const double text_spacing = 1.5, const double text_kerning = 0.1);

/** @} */  // -----------------------------------------------------

/** @name OpenGL bitmapped 2D fonts
	@{ */

/** This method is safe for calling from within ::render() methods \sa
 * renderTextBitmap */
void renderTextBitmap(const char* str, void* fontStyle);

/** Return the exact width in pixels for a given string, as will be rendered by
 * renderTextBitmap().
 * \sa renderTextBitmap
 */
int textBitmapWidth(
	const std::string& str, mrpt::opengl::TOpenGLFont font =
								mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24);

/** @} */  // --------------------------------------------------

/** @name OpenGL vector 3D fonts
	@{ */

/// sets the font to use for future font rendering commands. currently "sans",
/// "serif" and "mono" are available.
/// @param fontname string containing font name
void glSetFont(const std::string& fontname);

/// returns the name of the currently active font
const std::string& glGetFont();

/// renders a string in GL using the current settings.
/// Font coordinates are +X along the line and +Y along the up direction of
/// glyphs.
/// The origin is at the top baseline at the left of the first character.
/// Characters have a maximum size of 1.
/// linefeed is interpreted as a new line and the start is offset in -Y
/// direction by @ref spacing . Individual characters
/// are separated by @ref kerning + plus their individual with.
/// @param text string to be rendered, unknown characters are replaced with '?'
/// @param textScale The size of the characters (default=1.0)
/// @param style rendering style
/// @param spacing distance between individual text lines
/// @param kerning distance between characters
/// \note This functions comes from libcvd (BSD,
/// http://www.edwardrosten.com/cvd/ )
mrpt::img::TPixelCoordf glDrawText(
	const std::string& text, const double textScale,
	enum TOpenGLFontStyle style = NICE, double spacing = 1.5,
	double kerning = 0.1);

/// returns the size of the bounding box of a text to be rendered, similar to
/// @ref glDrawText but without any visual output
/// \note This functions comes from libcvd (BSD,
/// http://www.edwardrosten.com/cvd/ )
mrpt::img::TPixelCoordf glGetExtends(
	const std::string& text, const double textScale, double spacing = 1.5,
	double kerning = 0.1);

/** @} */  // --------------------------------------------------
}  // namespace gl_utils
}  // namespace opengl
}  // namespace mrpt
