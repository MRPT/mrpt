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
/** A wrapper for an OpenGL buffer object.
 * Refer to docs for glGenBuffers() and glBufferData().
 *
 * \ingroup mrpt_opengl_grp
 */
class COpenGLBuffer
{
   public:
	enum class Type : unsigned int
	{
		Vertex = 0x8892,  // GL_ARRAY_BUFFER (Default)
		ElementIndex = 0x8893,  // GL_ELEMENT_ARRAY_BUFFER
		PixelPack = 0x88EB,  // GL_PIXEL_PACK_BUFFER
		PixelUnpack = 0x88EC  // GL_PIXEL_UNPACK_BUFFER
	};

	enum class Usage : unsigned int
	{
		StreamDraw = 0x88E0,  // GL_STREAM_DRAW
		StreamRead = 0x88E1,  // GL_STREAM_READ
		StreamCopy = 0x88E2,  // GL_STREAM_COPY
		StaticDraw = 0x88E4,  // GL_STATIC_DRAW (Default)
		StaticRead = 0x88E5,  // GL_STATIC_READ
		StaticCopy = 0x88E6,  // GL_STATIC_COPY
		DynamicDraw = 0x88E8,  // GL_DYNAMIC_DRAW
		DynamicRead = 0x88E9,  // GL_DYNAMIC_READ
		DynamicCopy = 0x88EA  // GL_DYNAMIC_COPY
	};

	explicit COpenGLBuffer(const Type type);
	COpenGLBuffer() : COpenGLBuffer(Type::Vertex) {}
	~COpenGLBuffer() = default;

	Type type() const { return m_impl->type; }

	Usage usage() const { return m_impl->usage; }
	void setUsage(const Usage u) { m_impl->usage = u; }

	/** Actually create the buffer, destroying any previously existing buffer.
	 * Call after setting the type and usage. \sa allocate()
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

	/** Reserves byteCount bytes in the buffer and copy to it the provided data.
	 * create() and bind() must be called before using this method.
	 */
	void allocate(const void* data, int byteCount)
	{
		m_impl->allocate(data, byteCount);
	}

   private:
	struct RAII_Impl
	{
		RAII_Impl(COpenGLBuffer::Type t);
		~RAII_Impl();

		COpenGLBuffer::Type type;
		COpenGLBuffer::Usage usage = COpenGLBuffer::Usage::StaticDraw;

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

// For use in glVertexAttribPointer()
#define BUFFER_OFFSET(offset) (reinterpret_cast<GLvoid*>(offset))

}  // namespace mrpt::opengl
