/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>

using namespace mrpt::opengl;

namespace
{
const char* getGLErrorString(GLenum err)
{
  switch (err)
  {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
    case GL_STACK_OVERFLOW:
      return "GL_STACK_OVERFLOW";
    case GL_STACK_UNDERFLOW:
      return "GL_STACK_UNDERFLOW";
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
    default:
      return "Unknown OpenGL error";
  }
}
}  // namespace

void mrpt::opengl::checkOpenGLErr_impl(unsigned int glErrorCode, const char* filename, int lineno)
{
  if (glErrorCode == 0 /*GL_NO_ERROR*/)
  {
    return;
  }
  const std::string sErr =
      mrpt::format("[%s:%i] OpenGL error: %s", filename, lineno, getGLErrorString(glErrorCode));
  std::cerr << "[gl_utils::checkOpenGLError] " << sErr << "\n";
  THROW_EXCEPTION(sErr);
}
