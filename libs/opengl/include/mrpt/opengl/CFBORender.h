/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

			/** Constructor
			  */
			CFBORender( unsigned int width = 800, unsigned int height = 600 );

			/** Destructor
			 */
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

			/** Resize the image size.
			  */
			void  resize( unsigned int width, unsigned int height );

		protected:
			int m_win, m_width, m_height;
			unsigned int m_fbo, m_tex;

			/** Provide information on Framebuffer object extension.
			  */
			int isExtensionSupported( const char* extension );
		};
	} // end namespace

} // End of namespace

#endif
