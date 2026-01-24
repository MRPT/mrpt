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

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
void mrpt::opengl::checkOpenGLErr_impl(unsigned int glErrorCode, const char* filename, int lineno)
{
  if (glErrorCode == GL_NO_ERROR)
  {
    return;
  }
#if MRPT_HAS_OPENGL_GLUT
  const std::string sErr = mrpt::format(
      "[%s:%i] OpenGL error: %s", filename, lineno,
      reinterpret_cast<const char*>(gluErrorString(glErrorCode)));
#else
  // w/o glu:
  const std::string sErr = mrpt::format("[%s:%i] OpenGL error: %u", filename, lineno, glErrorCode);
#endif
  std::cerr << "[gl_utils::checkOpenGLError] " << sErr << "\n";
  THROW_EXCEPTION(sErr);
}
#endif