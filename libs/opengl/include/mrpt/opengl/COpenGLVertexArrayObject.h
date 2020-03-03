/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <memory>

namespace mrpt::opengl
{
/** A wrapper for an OpenGL vertex array object (VAO).
 * Refer to docs for glGenVertexArrays().
 *
 * \ingroup mrpt_opengl_grp
 */
class COpenGLVertexArrayObject
{
   public:
	COpenGLVertexArrayObject();
	~COpenGLVertexArrayObject() = default;

	/** Actually create the buffer, destroying any previously existing buffer.
	 */
	void create() { m_impl->create(); }

	/** Calls create() only if the buffer has not been created yet. */
	void createOnce()
	{
		if (!isCreated()) create();
	}
	bool isCreated() const { return m_impl->created; }

	/** Automatically called upon destructor, no need for the user to call it in
	 * normal situations. */
	void destroy() { m_impl->destroy(); }

	void bind() { m_impl->bind(); }
	void release() { m_impl->bind(); }

	unsigned int bufferId() const { return m_impl->buffer_id; }

   private:
	struct RAII_Impl
	{
		RAII_Impl() = default;
		~RAII_Impl();

		void create();
		void destroy();
		void bind();
		void release();
		void allocate(const void* data, int byteCount);

		bool created = false;
		unsigned int buffer_id = 0;
	};
	std::shared_ptr<RAII_Impl> m_impl;
};

}  // namespace mrpt::opengl
