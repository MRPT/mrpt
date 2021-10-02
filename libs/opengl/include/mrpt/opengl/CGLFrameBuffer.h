/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CGLFrameBuffer.h>
#include <mrpt/opengl/COpenGLScene.h>

namespace mrpt::opengl
{
/** Wrapper for an OpenGL FrameBuffer resource, with RGBA + depth buffers.
 *
 * \ingroup mrpt_opengl_grp
 */
class CGLFramebuffer
{
   public:
	CGLFramebuffer() = default;
	~CGLFramebuffer() { free(); }

	/** Creates a new FB object */
	void init(unsigned int width, unsigned int height, int nSamples = 1);

	/** Release resources */
	void free();

	/** Bind the framebuffer object to the current context */
	void bind();

	/** Unbind the framebuffer object from the context */
	void release();

	/// Blit the framebuffer object onto the screen
	void blit();

	bool initialized() { return m_Framebuffer != 0; }

   protected:
	unsigned int m_Framebuffer = 0, m_Depth = 0, m_Color = 0;
	unsigned int m_width = 0, m_height = 0;	 /// In pixels
	int m_Samples = 0;
};

}  // namespace mrpt::opengl
