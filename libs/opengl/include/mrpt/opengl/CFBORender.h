/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CFBORender_H
#define opengl_CFBORender_H

#include <mrpt/utils/CImage.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CTextMessageCapable.h>

namespace mrpt
{
	namespace opengl
	{
		/** A class for rendering 3D scenes off-screen directly into an image using OpenGL extensions (glext).
		  *  To define a background color, set it in the scene's "main" viewport.
		  *
		  *  You can add overlaid text messages, see base class CTextMessageCapable
		  *
		  *  \sa Example "fbo_render_test"
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CFBORender : public mrpt::opengl::CTextMessageCapable
		{
		public:
			/** Constructor. 
			  * \param[in] skip_glut_window Should be set to true only if another GUI windows already exist with an associated OpenGL context. If left to false, a hidden GLUT window will be created.
			  */
			CFBORender( unsigned int width = 800, unsigned int height = 600, const bool skip_glut_window = false );

			/** Destructor */
			virtual ~CFBORender();

			/** Change the scene camera.
			  */
			void  setCamera( const COpenGLScene& scene, const CCamera& camera );

			/** Get a reference to the scene camera.
			  */
			CCamera  &getCamera( const COpenGLScene& scene );

			/** Render the scene and get the rendered rgb image. Resizes the image buffer if it
				is necessary.
			  */
			void  getFrame( const COpenGLScene& scene, mrpt::utils::CImage& image );

			/** Render the scene and get the rendered rgb image. Does not resize the image buffer.
				MANDATORY: The image origin must be bottom left.
			  */
			void  getFrame2( const COpenGLScene& scene, mrpt::utils::CImage& image );

			/** Resize the rendering canvas size. */
			void  resize( unsigned int width, unsigned int height );

			/** Get the default background color (unles an COpenGLViewport defines a custom color) */
			const mrpt::utils::TColorf & getBackgroundColor() const { return m_default_bk_color; }

			/** Set the default background color (unles an COpenGLViewport defines a custom color) */
			void setBackgroundColor(const mrpt::utils::TColorf &col){ m_default_bk_color=col; }

		protected:
			int                  m_win, m_width, m_height;
			unsigned int         m_fbo, m_tex;
			bool                 m_win_used;
			mrpt::utils::TColorf m_default_bk_color;

			/** Provide information on Framebuffer object extension.
			  */
			int isExtensionSupported( const char* extension );
		};
	} // end namespace

} // End of namespace

#endif
