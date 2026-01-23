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
#include <mrpt/opengl/VertexArrayObject.h>
#include <mrpt/opengl/opengl_api.h>

#include <thread>

using namespace mrpt::opengl;

VertexArrayObject::VertexArrayObject() {}

VertexArrayObject::RAII_Impl::~RAII_Impl()
{
  // Free resources:
  destroy();
}

void VertexArrayObject::RAII_Impl::create()
{
  destroy();
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  GLuint buffer;
  glGenVertexArrays(1, &buffer);
  m_state.get().buffer_id = buffer;
  m_state.get().created = true;
#endif
}

void VertexArrayObject::RAII_Impl::destroy()
{
  if (!m_state.get().created) return;
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  release();
  glDeleteVertexArrays(1, &m_state.get().buffer_id);
#endif
  m_state.get().buffer_id = 0;
  m_state.get().created = false;
}

void VertexArrayObject::RAII_Impl::bind() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  ASSERT_(m_state.get().created);
  glBindVertexArray(m_state.get().buffer_id);
#endif
}

void VertexArrayObject::RAII_Impl::release()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!m_state.get().created) return;
  glBindVertexArray(0);
#endif
}
