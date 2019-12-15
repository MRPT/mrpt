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
/** @name Data types for mrpt::opengl::gl_utils
	@{ */

/** Information about the rendering process being issued. \sa See
 * getCurrentRenderingInfo for more details */
struct TRenderInfo
{
	/** Rendering viewport geometry (in pixels) */
	int vp_x, vp_y, vp_width, vp_height;
	/** The 4x4 projection matrix */
	mrpt::math::CMatrixFixed<float, 4, 4> proj_matrix;
	/** The 4x4 model transformation matrix */
	mrpt::math::CMatrixFixed<float, 4, 4> model_matrix;
	/** PROJ * MODEL */
	mrpt::math::CMatrixFixed<float, 4, 4> full_matrix;
	/** The 3D location of the camera */
	mrpt::math::TPoint3Df camera_position;

	/** Computes the normalized coordinates (range=[0,1]) on the current
	 * rendering viewport of a
	 * point with local coordinates (wrt to the current model matrix) of
	 * (x,y,z).
	 *  The output proj_z_depth is the real distance from the eye to the point.
	 */
	void projectPoint(
		float x, float y, float z, float& proj_x, float& proj_y,
		float& proj_z_depth) const;

	/** Exactly like projectPoint but the (x,y) projected coordinates are given
	 * in pixels instead of normalized coordinates. */
	void projectPointPixels(
		float x, float y, float z, float& proj_x_px, float& proj_y_px,
		float& proj_z_depth) const
	{
		projectPoint(x, y, z, proj_x_px, proj_y_px, proj_z_depth);
		proj_x_px = (proj_x_px + 1.0f) * (vp_width / 2.0f);
		proj_y_px = (proj_y_px + 1.0f) * (vp_height / 2.0f);
	}
};

/** @} */  // -----------------------------------------------------

/** @name Miscellaneous rendering methods
	@{ */

/** For each object in the list:
 *   - checks visibility of each object
 *   - prepare the GL_MODELVIEW matrix according to its coordinates
 *   - call its ::render()
 *   - shows its name (if enabled).
 *
 *  \note Used by  COpenGLViewport, CSetOfObjects
 */
void renderSetOfObjects(const mrpt::opengl::CListOpenGLObjects& objs);

/** Checks glGetError and throws an exception if an error situation is found */
void checkOpenGLErr_impl(const char* filename, int lineno);

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

/** Gather useful information on the render parameters.
 *  It can be called from within the render() method of CRenderizable-derived
 * classes, and
 *   the returned matrices can be used to determine whether a given point
 * (lx,ly,lz)
 *   in local coordinates wrt the object being rendered falls within the screen
 * or not:
 * \code
 *  TRenderInfo ri;
 *  getCurrentRenderingInfo(ri);
 *  Eigen::Matrix<float,4,4> M= ri.proj_matrix * ri.model_matrix *
 * HomogeneousMatrix(lx,ly,lz);
 *  const float rend_x = M(0,3)/M(3,3);
 *  const float rend_y = M(1,3)/M(3,3);
 * \endcode
 *  where (rend_x,rend_y) are both in the range [-1,1].
 */
void getCurrentRenderingInfo(TRenderInfo& ri);

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
