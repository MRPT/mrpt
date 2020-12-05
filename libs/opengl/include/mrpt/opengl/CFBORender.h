/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/COpenGLScene.h>

namespace mrpt::opengl
{
/** A class for rendering 3D scenes off-screen directly into an image using
 * OpenGL extensions (glext).
 *  To define a background color, set it in the scene's "main" viewport.
 *
 *  You can add overlaid text messages, see base class CTextMessageCapable
 *
 *  \sa Example "fbo_render_test"
 * \ingroup mrpt_opengl_grp
 */
class CFBORender
{
   public:
	/** Constructor.
	 * \param[in] skip_glut_window Should be set to true only if another GUI
	 * windows already exist with an associated OpenGL context. If left to
	 * false, a hidden GLUT window will be created.
	 */
	CFBORender(
		unsigned int width = 800, unsigned int height = 600,
		const bool skip_glut_window = false);

	/** Destructor */
	virtual ~CFBORender();

	/** Change the scene camera.
	 */
	void setCamera(const COpenGLScene& scene, const CCamera& camera);

	/** Get a reference to the scene camera.
	 */
	CCamera& getCamera(const COpenGLScene& scene);

	/** Render the scene and get the rendered rgb image. Resizes the image
	   buffer if it is necessary.
	  */
	void getFrame(const COpenGLScene& scene, mrpt::img::CImage& image);

	/** Render the scene and get the rendered rgb image. Does not resize the
	   image buffer.
	  */
	void getFrame2(const COpenGLScene& scene, mrpt::img::CImage& image);

	/** Resize the rendering canvas size. */
	void resize(unsigned int width, unsigned int height);

	/** Get the default background color (unles an COpenGLViewport defines a
	 * custom color) */
	const mrpt::img::TColorf& getBackgroundColor() const
	{
		return m_default_bk_color;
	}

	/** Set the default background color (unles an COpenGLViewport defines a
	 * custom color) */
	void setBackgroundColor(const mrpt::img::TColorf& col)
	{
		m_default_bk_color = col;
	}

   protected:
	int m_win, m_width, m_height;
	unsigned int m_fbo{0}, m_tex{0};
	bool m_win_used;
	mrpt::img::TColorf m_default_bk_color;

	/** Provide information on Framebuffer object extension.
	 */
	bool isExtensionSupported(const std::string& extension);
};
}  // namespace mrpt::opengl
