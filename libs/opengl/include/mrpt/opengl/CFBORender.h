/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/format.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/COpenGLFramebuffer.h>
#include <mrpt/opengl/COpenGLScene.h>

namespace mrpt::opengl
{
/** Render 3D scenes off-screen directly to RGB and/or RGB+D images.
 *
 * Main methods:
 * - render_RGB(): Renders a scene into an RGB image.
 * - render_RGBD(): Renders a scene into an RGB and depth images.
 *
 * To define a background color, define it in your
 * `scene.getViewport()->setCustomBackgroundColor()`. You can add overlaid text
 * messages, see base class CTextMessageCapable
 *
 * The SE(3) pose from which the scene is rendered is defined by the scene
 * `"main"` viewport camera pose.
 * See \ref gui_fbo_render_example for code examples.
 *
 * \sa \ref opengl_offscreen_render_example , \ref gui_fbo_render_example
 * \ingroup mrpt_opengl_grp
 */
class CFBORender
{
   public:
	/** Parameters for CFBORender constructor */
	struct Parameters
	{
		Parameters() = default;

		Parameters(unsigned int Width, unsigned int Height)
			: width(Width), height(Height)
		{
		}

		unsigned int width = 800;  //!< Width of images to render.
		unsigned int height = 600;	//!< Height of images to render.

		/** By default, each CFBORender constructor will create its own EGL
		 * context, which enables using them in different threads, use in
		 * head-less applications, etc.
		 *
		 *  Set this to false to save that effort, only if it is ensured that
		 * render calls will always happen from a thread where OpenGL has been
		 * already initialized and a context created.
		 */
		bool create_EGL_context = true;

		/** Can be used to select a particular GPU (or software-emulated)
		 * device.
		 *
		 * Create a CFBORender object with the environment variable
		 * MRPT_FBORENDER_SHOW_DEVICES=true to see a list of available and
		 * detected GPU devices. */
		int deviceIndexToUse = 0;

		int blueSize = 8, redSize = 8, greenSize = 8, depthSize = 24;
		bool conformantOpenGLES2 = false;  //!< Default: EGL_OPENGL_ES_BIT
		bool renderableOpenGLES2 = false;  //!< Default: EGL_OPENGL_ES_BIT
		bool bindOpenGLES_API = false;	//!< Default: EGL_OPENGL_API
		int contextMajorVersion = 0;  //!< 0=default
		int contextMinorVersion = 0;  //!< 0=default
		/// See https://www.khronos.org/opengl/wiki/Debug_Output
		bool contextDebug = false;
	};

	/** Main constructor */
	CFBORender(const Parameters& p);

	/** Constructor */
	CFBORender(unsigned int width = 800, unsigned int height = 600)
		: CFBORender(Parameters(width, height))
	{
	}

	/** Destructor */
	~CFBORender();

	/** Change the scene camera to be used when rendering the scene through this
	 * particular instance of CFBORender. */
	void setCamera(const COpenGLScene& scene, const CCamera& camera)
	{
		m_renderFromCamera = camera;
	}

	/** Get a reference to the scene camera to be used when rendering the scene
	 * through this particular instance of CFBORender. */
	CCamera& getCamera(const COpenGLScene& scene) { return m_renderFromCamera; }

	/** Render the scene and get the rendered RGB image. Resizes the image
	 *  buffer if necessary to the configured render resolution.
	 *
	 *  \sa render_RGBD()
	 */
	void render_RGB(const COpenGLScene& scene, mrpt::img::CImage& outRGB);

	/** Render the scene and get the rendered RGB and depth images.
	 * Resizes the provided buffers if necessary to the configured render
	 * resolution.
	 * The output depth image is in linear depth distance units (e.g. "meters").
	 * Note that values is depth, not range, that is, it's the "+z" coordinate
	 * of a point as seen from the camera, with +Z pointing forward in the view
	 * direction (the common convention in computer vision).
	 * Pixels without any observed object in the valid viewport {clipMin,
	 * clipMax} range will be returned with a range of `0.0`.
	 *
	 *  \sa render_RGB()
	 */
	void render_RGBD(
		const COpenGLScene& scene, mrpt::img::CImage& outRGB,
		mrpt::math::CMatrixFloat& outDepth);

	/** Like render_RGBD(), but only renders the depth image.
	 *  \sa render_RGBD()
	 */
	void render_depth(
		const COpenGLScene& scene, mrpt::math::CMatrixFloat& outDepth);

   protected:
	void* m_eglDpy = nullptr;
	void* m_eglCfg = nullptr;  // EGLConfig
	void* m_eglContext = nullptr;
	void* m_eglSurf = nullptr;	// EGLSurface

	unsigned int m_texRGB = 0;

	const Parameters m_params;	//!< Parameters used in the ctor

	COpenGLFramebuffer m_fb;
	mrpt::opengl::CCamera m_renderFromCamera;

	void internal_render_RGBD(
		const COpenGLScene& scene,
		const mrpt::optional_ref<mrpt::img::CImage>& outRGB,
		const mrpt::optional_ref<mrpt::math::CMatrixFloat>& outDepth);
};
}  // namespace mrpt::opengl
