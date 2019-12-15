/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CTexturedObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>
#include <memory>  // std::align
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;
using mrpt::img::CImage;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(
	CTexturedObject, CRenderizableDisplayList, mrpt::opengl)

// Whether to profile memory allocations:
//#define TEXTUREOBJ_PROFILE_MEM_ALLOC

// Whether to use a memory pool for the texture buffer:
#define TEXTUREOBJ_USE_MEMPOOL

// Data types for memory pooling CTexturedObject:
#ifdef TEXTUREOBJ_USE_MEMPOOL

#include <mrpt/system/CGenericMemoryPool.h>

struct CTexturedObject_MemPoolParams
{
	/** size of the vector<unsigned char> */
	size_t len = 0;

	inline bool isSuitable(const CTexturedObject_MemPoolParams& req) const
	{
		return len == req.len;
	}
};
struct CTexturedObject_MemPoolData
{
	vector<unsigned char> data;
};

using TMyMemPool = mrpt::system::CGenericMemoryPool<
	CTexturedObject_MemPoolParams, CTexturedObject_MemPoolData>;
#endif

void CTexturedObject::assignImage(const CImage& img, const CImage& imgAlpha)
{
	MRPT_START

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = img;
	m_textureImageAlpha = imgAlpha;

	m_enableTransparency = true;

	MRPT_END
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void CTexturedObject::assignImage(const CImage& img)
{
	MRPT_START

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = img;

	m_enableTransparency = false;

	MRPT_END
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void CTexturedObject::assignImage_fast(CImage& img, CImage& imgAlpha)
{
	MRPT_START

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = std::move(img);
	m_textureImageAlpha = std::move(imgAlpha);

	m_enableTransparency = true;

	MRPT_END
}

/*---------------------------------------------------------------
							assignImage
  ---------------------------------------------------------------*/
void CTexturedObject::assignImage_fast(CImage& img)
{
	MRPT_START

	CRenderizableDisplayList::notifyChange();

	unloadTexture();

	// Make a copy:
	m_textureImage = std::move(img);

	m_enableTransparency = false;

	MRPT_END
}

// Auxiliary function for loadTextureInOpenGL(): reserve memory and return
// 16byte aligned starting point within it:
unsigned char* reserveDataBuffer(const size_t len, vector<unsigned char>& data)
{
#ifdef TEXTUREOBJ_USE_MEMPOOL
	TMyMemPool* pool = TMyMemPool::getInstance();
	if (pool)
	{
		CTexturedObject_MemPoolParams mem_params;
		mem_params.len = len;

		CTexturedObject_MemPoolData* mem_block =
			pool->request_memory(mem_params);
		if (mem_block)
		{
			// Recover the memory block via a swap:
			data.swap(mem_block->data);
			delete mem_block;
		}
	}
#endif
	data.resize(len);
	void* ptr = &data[0];
	size_t space = len;
	return reinterpret_cast<unsigned char*>(
		std::align(16, 1 /*dummy size*/, ptr, space));
}

/*---------------------------------------------------------------
							loadTextureInOpenGL
  ---------------------------------------------------------------*/
void CTexturedObject::loadTextureInOpenGL() const
{
#if MRPT_HAS_OPENGL_GLUT
	unsigned char* dataAligned = nullptr;
	vector<unsigned char> data;

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
	static mrpt::system::CTimeLogger tim;
#endif

	// Do nothing until we are assigned an image.
	if (m_textureImage.isEmpty()) return;

	try
	{
		if (m_texture_is_loaded)
		{
			glBindTexture(GL_TEXTURE_2D, m_glTextureName);
			CHECK_OPENGL_ERROR();
			return;
		}

		// Reserve the new one --------------------------
		ASSERT_(m_textureImage.getPixelDepth() == mrpt::img::PixelDepth::D8U);

		// allocate texture names:
		m_glTextureName = getNewTextureNumber();

		// select our current texture
		glBindTexture(GL_TEXTURE_2D, m_glTextureName);
		CHECK_OPENGL_ERROR();

		// when texture area is small, linear interpolation. Default is
		// GL_LINEAR_MIPMAP_NEAREST but we
		// are not building mipmaps.
		//  See also:
		//  http://www.opengl.org/discussion_boards/ubbthreads.php?ubb=showflat&Number=133116&page=1
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		CHECK_OPENGL_ERROR();

		// when texture area is large, NEAREST: this is mainly thinking of
		// rendering
		//  occupancy grid maps, such as we want those "big pixels" to be
		//  clearly visible ;-)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		CHECK_OPENGL_ERROR();

		// if wrap is true, the texture wraps over at the edges (repeat)
		//       ... false, the texture ends at the edges (clamp)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		CHECK_OPENGL_ERROR();

		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		CHECK_OPENGL_ERROR();

		// Assure that the images do not overpass the maximum dimensions allowed
		// by OpenGL:
		// ------------------------------------------------------------------------------------
		GLint texSize;
		glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize);
		while (m_textureImage.getHeight() > (unsigned int)texSize ||
			   m_textureImage.getWidth() > (unsigned int)texSize)
		{
			m_textureImage =
				m_textureImage.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
			m_textureImageAlpha =
				m_textureImageAlpha.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
		}

		const int width = m_textureImage.getWidth();
		const int height = m_textureImage.getHeight();

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
		{
			const std::string sSec = mrpt::format(
				"opengl_texture: load %ix%i %s %stransp", width, height,
				m_textureImage.isColor() ? "RGB" : "BW",
				m_enableTransparency ? "" : "no ");
			tim.enter(sSec.c_str());
		}
#endif

		r_width = width;  // round2up( width );
		r_height = height;  // round2up( height );

		// Padding pixels:
		m_pad_x_right = (r_width - width);
		m_pad_y_bottom = (r_height - height);

		if (m_enableTransparency)
		{
			ASSERT_(!m_textureImageAlpha.isColor());
			ASSERT_(
				m_textureImageAlpha.getWidth() == m_textureImage.getWidth());
			ASSERT_(
				m_textureImageAlpha.getHeight() == m_textureImage.getHeight());
		}

		if (m_textureImage.isColor())
		{
			// Color texture:
			if (m_enableTransparency)
			{
// Color texture WITH trans.
// --------------------------------------
#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format(
					"opengl_texture_alloc %ix%i (color,trans)", width, height);
				tim.enter(sSec.c_str());
#endif

				dataAligned = reserveDataBuffer(height * width * 4 + 512, data);

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#endif

				for (int y = 0; y < height; y++)
				{
					unsigned char* ptrSrcCol = m_textureImage(0, y, 0);
					unsigned char* ptrSrcAlfa = m_textureImageAlpha(0, y);
					unsigned char* ptr = dataAligned + y * width * 4;

					for (int x = 0; x < width; x++)
					{
						*ptr++ = *ptrSrcCol++;
						*ptr++ = *ptrSrcCol++;
						*ptr++ = *ptrSrcCol++;
						*ptr++ = *ptrSrcAlfa++;
					}
				}

				// Prepare image data types:
				const GLenum img_type = GL_UNSIGNED_BYTE;
				// Reverse RGB <-> BGR order?
				const bool is_RGB_order =
					(m_textureImage.getChannelsOrder() == std::string("RGB"));
				const GLenum img_format = (is_RGB_order ? GL_RGBA : GL_BGRA);

				// Send image data to OpenGL:
				glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
				glPixelStorei(GL_UNPACK_ROW_LENGTH, width);
				glTexImage2D(
					GL_TEXTURE_2D, 0 /*level*/, 4 /* RGB components */, width,
					height, 0 /*border*/, img_format, img_type, dataAligned);
				CHECK_OPENGL_ERROR();
				glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset

				// No need to hide a fill border:
				m_pad_x_right = 0;
				m_pad_y_bottom = 0;

			}  // End of color texture WITH trans.
			else
			{
				// Color texture WITHOUT trans.
				// --------------------------------------
				// Prepare image data types:
				const GLenum img_type = GL_UNSIGNED_BYTE;
				const int nBytesPerPixel = m_textureImage.isColor() ? 3 : 1;
				// Reverse RGB <-> BGR order?
				const bool is_RGB_order =
					(m_textureImage.getChannelsOrder() == std::string("RGB"));
				const GLenum img_format = nBytesPerPixel == 3
											  ? (is_RGB_order ? GL_RGB : GL_BGR)
											  : GL_LUMINANCE;

				// Send image data to OpenGL:
				glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
				glPixelStorei(
					GL_UNPACK_ROW_LENGTH,
					m_textureImage.getRowStride() / nBytesPerPixel);
				glTexImage2D(
					GL_TEXTURE_2D, 0 /*level*/, 3 /* RGB components */, width,
					height, 0 /*border*/, img_format, img_type,
					m_textureImage.ptrLine<uint8_t>(0));
				glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset

				// No need to hide a fill border:
				m_pad_x_right = 0;
				m_pad_y_bottom = 0;

			}  // End of color texture WITHOUT trans.
		}
		else
		{
			// Gray-scale texture:
			if (m_enableTransparency)
			{
#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				const std::string sSec = mrpt::format(
					"opengl_texture_alloc %ix%i (gray,transp)", width, height);
				tim.enter(sSec.c_str());
#endif

				dataAligned =
					reserveDataBuffer(height * width * 2 + 1024, data);

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
				tim.leave(sSec.c_str());
#endif

				for (int y = 0; y < height; y++)
				{
					unsigned char* ptrSrcCol = m_textureImage(0, y);
					unsigned char* ptrSrcAlfa = m_textureImageAlpha(0, y);
					unsigned char* ptr = dataAligned + y * width * 2;
					for (int x = 0; x < width; x++)
					{
						*ptr++ = *ptrSrcCol++;
						*ptr++ = *ptrSrcAlfa++;
					}
				}

				// Prepare image data types:
				const GLenum img_type = GL_UNSIGNED_BYTE;
				const GLenum img_format = GL_LUMINANCE_ALPHA;

				// Send image data to OpenGL:
				glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
				glPixelStorei(GL_UNPACK_ROW_LENGTH, width);
				glTexImage2D(
					GL_TEXTURE_2D, 0 /*level*/, 2 /* RGB components */, width,
					height, 0 /*border*/, img_format, img_type, dataAligned);
				CHECK_OPENGL_ERROR();
				glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset

				// No need to hide a fill border:
				m_pad_x_right = 0;
				m_pad_y_bottom = 0;

			}  // End of gray-scale texture WITH trans.
			else
			{
				// Prepare image data types:
				const GLenum img_type = GL_UNSIGNED_BYTE;
				const GLenum img_format = GL_LUMINANCE;

				// Send image data to OpenGL:
				glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
				glPixelStorei(
					GL_UNPACK_ROW_LENGTH, m_textureImage.getRowStride());
				glTexImage2D(
					GL_TEXTURE_2D, 0 /*level*/, 1 /* RGB components */, width,
					height, 0 /*border*/, img_format, img_type,
					m_textureImage(0, 0));
				CHECK_OPENGL_ERROR();
				glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset

				// No need to hide a fill border:
				m_pad_x_right = 0;
				m_pad_y_bottom = 0;

			}  // End of gray-scale texture WITHOUT trans.
		}

		m_texture_is_loaded = true;

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
		{
			const std::string sSec = mrpt::format(
				"opengl_texture: load %ix%i %s %stransp", width, height,
				m_textureImage.isColor() ? "RGB" : "BW",
				m_enableTransparency ? "" : "no ");
			tim.leave(sSec.c_str());
		}
#endif

#ifdef TEXTUREOBJ_USE_MEMPOOL
		// Before freeing the buffer in "data", donate my memory to the pool:
		if (!data.empty())
		{
			TMyMemPool* pool = TMyMemPool::getInstance();
			if (pool)
			{
				CTexturedObject_MemPoolParams mem_params;
				mem_params.len = data.size();

				auto* mem_block = new CTexturedObject_MemPoolData();
				data.swap(mem_block->data);

				pool->dump_to_pool(mem_params, mem_block);
			}
		}
#endif
	}
	catch (exception& e)
	{
		THROW_EXCEPTION(
			format("m_glTextureName=%i\n%s", m_glTextureName, e.what()));
	}
	catch (...)
	{
		THROW_EXCEPTION("Runtime error!");
	}
#endif
}

/*---------------------------------------------------------------
							~CTexturedObject
  ---------------------------------------------------------------*/
CTexturedObject::~CTexturedObject() { unloadTexture(); }
/*---------------------------------------------------------------
							unloadTexture
  ---------------------------------------------------------------*/
void CTexturedObject::unloadTexture()
{
	if (m_texture_is_loaded)
	{
		m_texture_is_loaded = false;
		releaseTextureName(m_glTextureName);
		m_glTextureName = 0;
	}
}

void CTexturedObject::writeToStreamTexturedObject(
	mrpt::serialization::CArchive& out) const
{
	uint8_t ver = 0;

	out << ver;
	out << m_enableTransparency;
	out << m_textureImage;
	if (m_enableTransparency) out << m_textureImageAlpha;
}

void CTexturedObject::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	render_pre();
	if (glGetError() != GL_NO_ERROR)
		std::cerr << "render_pre: Error" << std::endl;
	render_texturedobj();
	if (glGetError() != GL_NO_ERROR)
		std::cerr << "render_texturedobj: Error" << std::endl;
	render_post();
	if (glGetError() != GL_NO_ERROR)
		std::cerr << "render_post: Error" << std::endl;
#endif
}

void CTexturedObject::render_pre() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START
	glEnable(GL_TEXTURE_2D);
	CHECK_OPENGL_ERROR();

	if (m_enableTransparency || m_color.A != 255)
	{
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	// This will load and/or select our texture, only if "m_texture_is_loaded"
	// is false
	loadTextureInOpenGL();
	MRPT_END
#endif
}

void CTexturedObject::render_post() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	if (m_enableTransparency || m_color.A != 255)
	{
		glDisable(GL_BLEND);
		CHECK_OPENGL_ERROR();

		glBlendFunc(GL_ONE, GL_ZERO);

		glEnable(GL_DEPTH_TEST);
		CHECK_OPENGL_ERROR();
	}

	glDisable(GL_TEXTURE_2D);
	CHECK_OPENGL_ERROR();

	MRPT_END
#endif
}

void CTexturedObject::readFromStreamTexturedObject(
	mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;

	CRenderizableDisplayList::notifyChange();

	switch (version)
	{
		case 0:
		{
			in >> m_enableTransparency;
			in >> m_textureImage;
			if (m_enableTransparency)
			{
				in >> m_textureImageAlpha;
				assignImage(m_textureImage, m_textureImageAlpha);
			}
			else
			{
				assignImage(m_textureImage);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizableDisplayList::notifyChange();
}
