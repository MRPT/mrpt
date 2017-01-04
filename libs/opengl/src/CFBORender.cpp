/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CFBORender.h>
#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::opengl;

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CFBORender::CFBORender( unsigned int width, unsigned int height, const bool skip_glut_window ) :
	m_width(width),
	m_height(height),
	m_win_used(!skip_glut_window),
	m_default_bk_color(.6f,.6f,.6f,1)
{
#if MRPT_HAS_OPENCV && MRPT_HAS_OPENGL_GLUT

	MRPT_START

	if (m_win_used)
	{
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
	}

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
	const GLenum texTarget =
#	if defined(GL_TEXTURE_RECTANGLE_NV)
		GL_TEXTURE_RECTANGLE_NV;
#	elif defined(GL_TEXTURE_RECTANGLE_ARB)
		GL_TEXTURE_RECTANGLE_ARB;
#	else
		GL_TEXTURE_RECTANGLE_EXT;
#	endif

	glBindTexture(texTarget, m_tex);
	glTexImage2D(texTarget, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	// bind this texture to the current framebuffer obj. as color_attachement_0
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, texTarget, m_tex, 0);

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
	// delete the current texture, the framebuffer object and the GLUT window
	glDeleteTextures(1, &m_tex);
	glDeleteFramebuffersEXT(1, &m_fbo);
	if (m_win_used) glutDestroyWindow(m_win);
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
#else
	MRPT_UNUSED_PARAM(scene); MRPT_UNUSED_PARAM(buffer);
#endif
}

/*---------------------------------------------------------------
		Render the scene and get the rendered rgb image. This
		function does not resize the image buffer.
 ---------------------------------------------------------------*/
void  CFBORender::getFrame2( const COpenGLScene& scene, CImage& buffer )
{
#if MRPT_HAS_OPENGL_GLUT

	MRPT_START

	// check the buffer size
	ASSERT_EQUAL_( buffer.getWidth(), static_cast<size_t>(m_width) )
	ASSERT_EQUAL_( buffer.getHeight(),static_cast<size_t>(m_height) )
	ASSERT_EQUAL_( buffer.getChannelCount(), 3 )
	ASSERT_EQUAL_( buffer.isOriginTopLeft(), false )

	// bind the framebuffer, fbo, so operations will now occur on it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo);

	glClearColor(m_default_bk_color.R,m_default_bk_color.G,m_default_bk_color.B,m_default_bk_color.A);

	// Render opengl objects:
	// ---------------------------
	scene.render();

	// If any, draw the 2D text messages:
	// ----------------------------------
	render_text_messages(m_width,m_height);


	// TODO NOTE: This should fail if the image has padding bytes. See glPixelStore() etc.
	glReadPixels(0, 0, m_width, m_height, GL_BGR_EXT, GL_UNSIGNED_BYTE, buffer(0,0) );

	//'unbind' the frambuffer object, so subsequent drawing ops are not drawn into the FBO.
	// '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	MRPT_END
#else
	MRPT_UNUSED_PARAM(scene); MRPT_UNUSED_PARAM(buffer);
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
	const GLenum texTarget =
#	if defined(GL_TEXTURE_RECTANGLE_NV)
		GL_TEXTURE_RECTANGLE_NV;
#	elif defined(GL_TEXTURE_RECTANGLE_ARB)
		GL_TEXTURE_RECTANGLE_ARB;
#	else
		GL_TEXTURE_RECTANGLE_EXT;
#	endif

	glBindTexture(texTarget, m_tex);
	glTexImage2D(texTarget, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	//'unbind' the frambuffer object, so subsequent drawing ops are not drawn into the FBO.
	// '0' means "windowing system provided framebuffer
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	MRPT_END

//#else
//	THROW_EXCEPTION("MRPT compiled without OpenCV and/or OpenGL support!!")
#else
	MRPT_UNUSED_PARAM(width); MRPT_UNUSED_PARAM(height);
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
#else
	MRPT_UNUSED_PARAM(extension);
#endif

	return 0;
}
