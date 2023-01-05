/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/PerThreadDataHolder.h>

#include <memory>
#include <thread>

namespace mrpt::opengl
{
/** A wrapper for an OpenGL vertex array object (VAO).
 * Refer to docs for glGenVertexArrays().
 *
 * \ingroup mrpt_opengl_grp
 * \note OpenGL VAOs *cannot* be shared among threads/GL contexts.
 */
class COpenGLVertexArrayObject
{
   public:
	COpenGLVertexArrayObject();
	~COpenGLVertexArrayObject() = default;

	/** Calls create() only if the buffer has not been created yet. */
	void createOnce()
	{
		if (!isCreated()) m_impl.create();
	}
	bool isCreated() const { return m_impl.m_state.get().created; }

	/** Automatically called upon destructor, no need for the user to call it in
	 * normal situations. */
	void destroy() { m_impl.destroy(); }

	void bind() { m_impl.bind(); }
	void release() { m_impl.bind(); }

	unsigned int bufferId() const { return m_impl.m_state.get().buffer_id; }

   private:
	struct RAII_Impl
	{
		RAII_Impl() = default;
		~RAII_Impl();

		void create();
		void destroy();
		void bind();
		void release();

		struct State
		{
			bool created = false;
			unsigned int buffer_id = 0;
		};

		mrpt::containers::PerThreadDataHolder<State> m_state;
	};
	RAII_Impl m_impl;
};

}  // namespace mrpt::opengl
