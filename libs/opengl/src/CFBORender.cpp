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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/CFBORender.h>
#include "opengl_internals.h"

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#	else
#		include <cv.h>
#	endif
#endif


using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::opengl;

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CFBORender::CFBORender( unsigned int width, unsigned int height ) : m_width(width), m_height(height)
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// check a previous initialization of the GLUT
	if(!glutGet(GLUT_INIT_STATE))
	{
		// create the context (a little trick)
		int argc = 1;
		char *argv[1] = { NULL };
		glutInit(&argc, argv);
	}

	// create a hidden window
	m_win = glutCreateWindow("CFBORender");
	glutHideWindow();

	// call after creating the hidden window
	if(!isExtensionSupported("GL_EXT_framebuffer_object"))
		THROW_EXCEPTION("Framebuffer Object extension unsupported");

	// In win32 we have to load the pointers to the functions:
#ifdef MRPT_OS_WINDOWS
	glGenFramebuffersEXT = (PFNGLGENFRAMEBUFFERSEXTPROC)wglGetProcAddress("glGenFramebuffersEXT");
	glDeleteFramebuffersEXT = (PFNGLDELETEFRAMEBUFFERSEXTPROC)wglGetProcAddress("glDeleteFramebuffersEXT");
	glBindFramebufferEXT = (PFNGLBINDFRAMEBUFFEREXTPROC)wglGetProcAddress("glBindFramebufferEXT");
	glFramebufferTexture2DEXT = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC) wglGetProcAddress("glFramebufferTexture2DEXT");

	ASSERT_(glGenFramebuffersEXT!=NULL)
	ASSERT_(glDeleteFramebuffersEXT!=NULL)
	ASSERT_(glBindFramebufferEXT!=NULL)
	ASSERT_(glFramebufferTexture2DEXT!=NULL)
#endif

	// gen the frambuffer object (FBO), similar manner as a texture
	glGenFramebuffersEXT(1, &m_fbo);

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo);

	// change viewport size (in pixels)
	glViewport(0, 0, m_width, m_height);

	// make a texture
	glGenTextures(1, &m_tex);

	// initialize texture that will store the framebuffer image
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, m_tex);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	// bind this texture to the current framebuffer obj. as color_attachement_0
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_NV, m_tex, 0);

	//'unbind' the frambuffer object, so subsequent drawing ops are not drawn into the FBO.
	// '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	MRPT_END

//#else
//	THROW_EXCEPTION("MRPT compiled without OpenCV and/or OpenGL support!!")
#endif
}

/*---------------------------------------------------------------
						Destructor:
 ---------------------------------------------------------------*/
CFBORender::~CFBORender()
{
#if MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// delete the current texture, the framebuffer object and the GLUT window
	glDeleteTextures(1, &m_tex);
	glDeleteFramebuffersEXT(1, &m_fbo);
	glutDestroyWindow(m_win);

	MRPT_END

//#else
//	THROW_EXCEPTION("MRPT compiled without OpenGL support!!")
#endif
}

/*---------------------------------------------------------------
					Set the scene camera
 ---------------------------------------------------------------*/
void  CFBORender::setCamera( const COpenGLScene& scene, const CCamera& camera )
{
	MRPT_START

	scene.getViewport("main")->getCamera() = camera;

	MRPT_END
}

/*---------------------------------------------------------------
					Get the scene camera
 ---------------------------------------------------------------*/
CCamera&  CFBORender::getCamera( const COpenGLScene& scene )
{
	MRPT_START

	return scene.getViewport("main")->getCamera();

	MRPT_END
}

/*---------------------------------------------------------------
		Render the scene and get the rendered rgb image. This
		function resizes the image buffer if it is necessary
 ---------------------------------------------------------------*/
void  CFBORender::getFrame( const COpenGLScene& scene, CImage& buffer )
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// resize the buffer if it is necessary
	if(	buffer.getWidth()        != static_cast<size_t>(m_width) ||
		buffer.getHeight()       != static_cast<size_t>(m_height) ||
		buffer.getChannelCount() != 3 ||
		buffer.isOriginTopLeft() != false )
	{
		buffer.resize(m_width, m_height, 3, false);
	}

	// Go on.
	getFrame2(scene,buffer);;

	MRPT_END
#endif
}

/*---------------------------------------------------------------
		Render the scene and get the rendered rgb image. This
		function does not resize the image buffer.
 ---------------------------------------------------------------*/
void  CFBORender::getFrame2( const COpenGLScene& scene, CImage& buffer )
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// check the buffer size
	ASSERT_EQUAL_( buffer.getWidth(), static_cast<size_t>(m_width) )
	ASSERT_EQUAL_( buffer.getHeight(),static_cast<size_t>(m_height) )
	ASSERT_EQUAL_( buffer.getChannelCount(), 3 )
	ASSERT_EQUAL_( buffer.isOriginTopLeft(), false )

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo);

	// Render opengl objects:
	// ---------------------------
	scene.render();

	// If any, draw the 2D text messages:
	// ----------------------------------
	render_text_messages(m_width,m_height);


	// get the current read buffer setting and save it
	IplImage* image = (IplImage*)buffer.getAsIplImage();
	glReadPixels(0, 0, m_width, m_height, GL_BGR_EXT, GL_UNSIGNED_BYTE, image->imageData);
	image = NULL;

	//'unbind' the frambuffer object, so subsequent drawing ops are not drawn into the FBO.
	// '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	MRPT_END

#endif
}

/*---------------------------------------------------------------
					Resize the image size
 ---------------------------------------------------------------*/
void CFBORender::resize( unsigned int width, unsigned int height )
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// update members
	m_width = width;
	m_height = height;

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo);

	// change viewport size (in pixels)
	glViewport(0, 0, m_width, m_height);

	// change texture size
	glBindTexture(GL_TEXTURE_RECTANGLE_NV, m_tex);
	glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	//'unbind' the frambuffer object, so subsequent drawing ops are not drawn into the FBO.
	// '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	MRPT_END

//#else
//	THROW_EXCEPTION("MRPT compiled without OpenCV and/or OpenGL support!!")
#endif
}

/*---------------------------------------------------------------
		Provide information on Framebuffer object extension
 ---------------------------------------------------------------*/
int CFBORender::isExtensionSupported( const char* extension )
{
#if MRPT_HAS_OPENGL_GLUT

	MRPT_START

	const GLubyte *extensions = NULL;
	const GLubyte *start;
	GLubyte *where, *terminator;

	/* Extension names should not have spaces. */
	where = (GLubyte *) strchr(extension, ' ');
	if (where || *extension == '\0')
		return 0;
	extensions = glGetString(GL_EXTENSIONS);

	/* It takes a bit of care to be fool-proof about parsing the
	OpenGL extensions string. Don't be fooled by sub-strings,
	etc. */
	start = extensions;
	for (;;) {
		where = (GLubyte *) strstr((const char *) start, extension);
		if (!where)
			break;
		terminator = where + strlen(extension);
		if (where == start || *(where - 1) == ' ')
			if (*terminator == ' ' || *terminator == '\0')
				return 1;
		start = terminator;
	}

	MRPT_END

//#else
//	THROW_EXCEPTION("MRPT compiled without OpenGL support!!")
#endif

	return 0;
}
