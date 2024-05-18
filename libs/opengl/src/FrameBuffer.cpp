/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/core/backtrace.h>
#include <mrpt/core/get_env.h>
#include <mrpt/opengl/FrameBuffer.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt::opengl;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
namespace
{
bool isExtensionSupported(const std::string& extension)
{
  MRPT_START
  for (int index = 0;; index++)
  {
    const auto extName = glGetStringi(GL_EXTENSIONS, index);
    if (!extName) break;
    const auto sExt = std::string(reinterpret_cast<const char*>(extName));
    // std::cout << sExt << "\n";
    if (sExt == extension) return true;
  }
  MRPT_END
  return false;
}
}  // namespace
#endif

void FrameBuffer::RAII_Impl::create(unsigned int width, unsigned int height, int nSamples)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  if (!isExtensionSupported("GL_EXT_framebuffer_object"))
    THROW_EXCEPTION(
        "Framebuffer Object extension unsupported "
        "(GL_EXT_framebuffer_object)");

  auto& _ = m_state.get();

  _.m_width = width;
  _.m_height = height;
  _.m_Samples = nSamples;
  _.m_isDepthMap = false;

  const auto oldFBs = CurrentBinding();

  // Render buffer: RGB
  glGenRenderbuffers(1, &_.m_Color);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  glBindRenderbuffer(GL_RENDERBUFFER, _.m_Color);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  if (nSamples <= 1)
  {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, _.m_width, _.m_height);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }
  else
  {
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, nSamples, GL_RGBA8, _.m_width, _.m_height);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Render buffer: DEPTH
  glGenRenderbuffers(1, &_.m_Depth);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  glBindRenderbuffer(GL_RENDERBUFFER, _.m_Depth);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  if (nSamples <= 1)
  {
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, _.m_width, _.m_height);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }
  else
  {
    glRenderbufferStorageMultisample(
        GL_RENDERBUFFER, nSamples, GL_DEPTH_COMPONENT24, _.m_width, _.m_height);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Frame buffer:
  glGenFramebuffers(1, &_.m_Framebuffer);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // bind the framebuffer, fbo, so operations will now occur on it
  glBindFramebuffer(GL_FRAMEBUFFER, _.m_Framebuffer);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _.m_Color);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _.m_Depth);
  CHECK_OPENGL_ERROR_IN_DEBUG();

#if 0
	glFramebufferRenderbuffer(
		GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, _.m_Depth);
	CHECK_OPENGL_ERROR_IN_DEBUG();
#endif

  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  CHECK_OPENGL_ERROR();

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) THROW_EXCEPTION("Could not create framebuffer object.");

  _.m_created = true;

  // Restore:
  Bind(oldFBs);
#else
  THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void FrameBuffer::RAII_Impl::createDepthMap(unsigned int width, unsigned int height)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!isExtensionSupported("GL_EXT_framebuffer_object"))
    THROW_EXCEPTION(
        "Framebuffer Object extension unsupported "
        "(GL_EXT_framebuffer_object)");

  auto& _ = m_state.get();

  _.m_isDepthMap = true;
  _.m_width = width;
  _.m_height = height;

  const auto oldFBs = CurrentBinding();

  // Depth FBO:
  glGenFramebuffers(1, &_.m_Framebuffer);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Create depth texture:
  glGenTextures(1, &_.m_DepthMapTexture);

  glBindTexture(GL_TEXTURE_2D, _.m_DepthMapTexture);

  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
      nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

  const float borderColor[4] = {1.0, 1.0, 1.0, 1.0};
  glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

  // attach depth texture as FBO's depth buffer
  glBindFramebuffer(GL_FRAMEBUFFER, _.m_Framebuffer);
  glFramebufferTexture2D(
      GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _.m_DepthMapTexture, 0);
  glDrawBuffer(GL_NONE);  // dont draw color
  glReadBuffer(GL_NONE);

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) THROW_EXCEPTION("Could not create depth map FBO.");

  _.m_created = true;

  // Restore:
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  // Restore:
  Bind(oldFBs);
#else
  THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void FrameBuffer::RAII_Impl::destroy()
{
  auto& _ = m_state.get();

  if (!_.m_created) return;

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  unbind();

  if (_.m_isDepthMap)
  {
    glDeleteFramebuffers(1, &_.m_Framebuffer);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }
  else
  {
    glDeleteRenderbuffers(1, &_.m_Color);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glDeleteRenderbuffers(1, &_.m_Depth);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glDeleteFramebuffers(1, &_.m_Framebuffer);
    CHECK_OPENGL_ERROR();
  }
#endif
  _.m_Color = _.m_Depth = 0;
  _.m_Framebuffer = _.m_DepthMapTexture = 0;
  _.m_created = false;
}

FrameBufferBinding FrameBuffer::RAII_Impl::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  const FrameBufferBinding ids = CurrentBinding();

  auto& _ = m_state.get();

  glBindFramebuffer(GL_FRAMEBUFFER, _.m_Framebuffer);
  CHECK_OPENGL_ERROR_IN_DEBUG();
  if (_.m_Samples > 1)
  {
    glEnable(GL_MULTISAMPLE);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  return ids;
#else
  THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}

void FrameBuffer::RAII_Impl::unbind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  auto& _ = m_state.get();
  if (_.m_Samples > 1) glDisable(GL_MULTISAMPLE);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  CHECK_OPENGL_ERROR();
#endif
}

void FrameBuffer::blit()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  auto& _ = m_impl.m_state.get();

  glBindFramebuffer(GL_READ_FRAMEBUFFER, _.m_Framebuffer);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glDrawBuffer(GL_BACK);

  glBlitFramebuffer(
      0, 0, _.m_width, _.m_height, 0, 0, _.m_width, _.m_height,
      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif
}

void FrameBuffer::Bind(const FrameBufferBinding& ids)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  glBindFramebuffer(GL_READ_FRAMEBUFFER, ids.readFbId);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, ids.drawFbId);
#endif
}

void FrameBuffer::Unbind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif
}

FrameBufferBinding FrameBuffer::CurrentBinding()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  GLint drawFboId = 0, readFboId = 0;
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFboId);
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFboId);

  FrameBufferBinding ids;
  ids.drawFbId = drawFboId;
  ids.readFbId = readFboId;
  return ids;
#else
  THROW_EXCEPTION("MRPT built without OpenGL support");
#endif
}
