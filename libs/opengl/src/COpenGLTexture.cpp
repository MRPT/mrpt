/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/COpenGLTexture.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>
#include <mutex>
#include <set>

using namespace mrpt::opengl;

const bool MRPT_OPENGL_VERBOSE =
	mrpt::get_env<bool>("MRPT_OPENGL_VERBOSE", false);

// Whether to profile memory allocations:
// #define TEXTUREOBJ_PROFILE_MEM_ALLOC

// Whether to use a memory pool for the texture buffer:
#define TEXTUREOBJ_USE_MEMPOOL

#ifdef TEXTUREOBJ_USE_MEMPOOL
#include <mrpt/system/CGenericMemoryPool.h>
#endif

void COpenGLTexture::unloadTexture()
{
	m_tex.run_on_all([](std::optional<texture_name_unit_t>& tnu) {
		if (!tnu) return;
		releaseTextureName(tnu.value());
		tnu.reset();
	});
}

/** This class is a workaround to crashes and memory leaks caused by not
 * reserving and freeing opengl textures from the same thread. */
class TextureResourceHandler
{
   public:
	static TextureResourceHandler& Instance()
	{
		static TextureResourceHandler o;
		return o;
	}

	/// Return [textureName, textureUnit]
	std::pair<unsigned int, unsigned int> generateTextureID()
	{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
		auto lck = mrpt::lockHelper(m_texturesMtx);

		processDestroyQueue();

		// Create one OpenGL texture
		GLuint textureID;
		glGenTextures(1, &textureID);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		m_textureReservedFrom[textureID] = std::this_thread::get_id();

		int foundUnit = -1;
		for (int i = 0; i < m_maxTextureUnits; i++)
			if (!m_occupiedTextureUnits.count(i))
			{
				foundUnit = i;
				break;
			}
		if (foundUnit < 0)
		{
			foundUnit = 0;
			std::cerr
				<< "[mrpt TextureResourceHandler] **WARNING**: Apparently "
				   "your program reached the maximum number of allowed "
				   "simultaneous OpenGL textures ("
				<< m_maxTextureUnits << ")" << std::endl;
		}
		else
		{
			m_occupiedTextureUnits.insert(foundUnit);
		}

		if (MRPT_OPENGL_VERBOSE)
			std::cout << "[mrpt generateTextureID] textureName:" << textureID
					  << " unit: " << foundUnit << std::endl;

		return {textureID, foundUnit};
#else
		THROW_EXCEPTION("This function needs OpenGL");
#endif
	}

	void releaseTextureID(unsigned int texName, unsigned int texUnit)
	{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
		MRPT_START
		auto lck = mrpt::lockHelper(m_texturesMtx);

		if (MRPT_OPENGL_VERBOSE)
			std::cout << "[mrpt releaseTextureID] textureName: " << texName
					  << " unit: " << texUnit << std::endl;

		m_destroyQueue[m_textureReservedFrom.at(texName)].push_back(texName);
		processDestroyQueue();
		m_occupiedTextureUnits.erase(texUnit);
		MRPT_END
#endif
	}

   private:
	TextureResourceHandler()
	{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
		glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &m_maxTextureUnits);
		if (MRPT_OPENGL_VERBOSE)
			std::cout << "[mrpt TextureResourceHandler] maxTextureUnits:"
					  << m_maxTextureUnits << std::endl;
#endif
	}

	void processDestroyQueue()
	{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
		if (auto itLst = m_destroyQueue.find(std::this_thread::get_id());
			itLst != m_destroyQueue.end())
		{
			auto& lst = itLst->second;
			glDeleteTextures(lst.size(), lst.data());
			CHECK_OPENGL_ERROR_IN_DEBUG();
			lst.clear();
		}
		if (MRPT_OPENGL_VERBOSE)
		{
			std::cout << "[mrpt processDestroyQueue] threadId="
					  << std::this_thread::get_id() << ". At output: ";
			for (const auto& lst : m_destroyQueue)
				std::cout << "[" << lst.first << "]=" << lst.second.size()
						  << " ";
			std::cout << "\n Texture units: " << m_occupiedTextureUnits.size()
					  << "\n";
		}
#endif
	}

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	std::mutex m_texturesMtx;
	std::map<GLuint, std::thread::id> m_textureReservedFrom;
	std::map<std::thread::id, std::vector<GLuint>> m_destroyQueue;
	std::set<GLint> m_occupiedTextureUnits;
	GLint m_maxTextureUnits;
#endif
};

/// Returns: [texture name, texture unit]
texture_name_unit_t mrpt::opengl::getNewTextureNumber()
{
	texture_name_unit_t ret;
	const auto r = TextureResourceHandler::Instance().generateTextureID();
	ret.name = r.first;
	ret.unit = r.second;
	return ret;
}

void mrpt::opengl::releaseTextureName(const texture_name_unit_t& t)
{
	TextureResourceHandler::Instance().releaseTextureID(t.name, t.unit);
}

bool COpenGLTexture::initialized() const
{
	// already assigned an ID?
	return m_tex.get().has_value();
}

void COpenGLTexture::assignImage(const mrpt::img::CImage& rgb, const Options& o)
{
	try
	{
		internalAssignImage(&rgb, nullptr, o);
	}
	catch (std::exception& e)
	{
		THROW_EXCEPTION_FMT("Error assigning texture: %s", e.what());
	}
}

void COpenGLTexture::assignImage(
	const mrpt::img::CImage& rgb, const mrpt::img::CImage& alpha,
	const Options& o)
{
	try
	{
		internalAssignImage(&rgb, &alpha, o);
	}
	catch (std::exception& e)
	{
		THROW_EXCEPTION_FMT("Error assigning texture: %s", e.what());
	}
}

// Data types for memory pooling CRenderizableShaderTexturedTriangles:
#ifdef TEXTUREOBJ_USE_MEMPOOL

struct CRenderizableShaderTexturedTriangles_MemPoolParams
{
	/** size of the vector<unsigned char> */
	size_t len = 0;

	inline bool isSuitable(
		const CRenderizableShaderTexturedTriangles_MemPoolParams& req) const
	{
		return len == req.len;
	}
};
struct CRenderizableShaderTexturedTriangles_MemPoolData
{
	std::vector<uint8_t> data;
};

using TMyMemPool = mrpt::system::CGenericMemoryPool<
	CRenderizableShaderTexturedTriangles_MemPoolParams,
	CRenderizableShaderTexturedTriangles_MemPoolData>;
#endif

// Auxiliary function for loadTextureInOpenGL(): reserve memory and return
// 16byte aligned starting point within it:
static unsigned char* reserveDataBuffer(
	const size_t len, std::vector<uint8_t>& data)
{
#ifdef TEXTUREOBJ_USE_MEMPOOL
	TMyMemPool* pool = TMyMemPool::getInstance();
	if (pool)
	{
		CRenderizableShaderTexturedTriangles_MemPoolParams mem_params;
		mem_params.len = len;

		CRenderizableShaderTexturedTriangles_MemPoolData* mem_block =
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

void COpenGLTexture::internalAssignImage(
	const mrpt::img::CImage* in_rgb, const mrpt::img::CImage* in_alpha,
	const Options& o)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	unsigned char* dataAligned = nullptr;
	std::vector<uint8_t> data;

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
	static mrpt::system::CTimeLogger tim;
#endif

	ASSERT_(in_rgb);

	in_rgb->forceLoad();  // just in case they are lazy-load imgs
	if (in_alpha) in_alpha->forceLoad();

	ASSERT_(in_rgb->getPixelDepth() == mrpt::img::PixelDepth::D8U);

	// Shallow copy of the images, for the case we need to downsample them
	// below:
	mrpt::img::CImage rgb(*in_rgb, mrpt::img::SHALLOW_COPY);
	mrpt::img::CImage alpha;
	if (in_alpha) alpha = mrpt::img::CImage(*in_alpha, mrpt::img::SHALLOW_COPY);

	// allocate texture names:
	get() = getNewTextureNumber();

	// activate the texture unit first before binding texture
	bind();

	// when texture area is small, linear interpolation. Default is
	// GL_LINEAR_MIPMAP_NEAREST but we
	// are not building mipmaps.
	MRPT_TODO("mipmaps!");
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// when texture area is large, NEAREST: this is mainly thinking of
	// rendering
	//  occupancy grid maps, such as we want those "big pixels" to be
	//  clearly visible ;-)
	glTexParameterf(
		GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
		o.magnifyLinearFilter ? GL_LINEAR : GL_NEAREST);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	// if wrap is true, the texture wraps over at the edges (repeat)
	//       ... false, the texture ends at the edges (clamp)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	CHECK_OPENGL_ERROR_IN_DEBUG();

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	CHECK_OPENGL_ERROR_IN_DEBUG();
	MRPT_TODO("wrap options?");

	// Assure that the images do not overpass the maximum dimensions allowed
	// by OpenGL:
	// ------------------------------------------------------------------------------------
	GLint texSize;
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize);
	while (rgb.getHeight() > (unsigned int)texSize ||
		   rgb.getWidth() > (unsigned int)texSize)
	{
		static bool warningEmitted = false;
		if (!warningEmitted)
		{
			warningEmitted = true;
			std::cerr << "[mrpt::opengl::CRenderizableShaderTexturedTriangles] "
						 "**PERFORMACE WARNING**:\n"
					  << " Downsampling texture image of size "
					  << rgb.getWidth() << "x" << rgb.getHeight()
					  << " since maximum allowed OpenGL texture size "
						 "(GL_MAX_TEXTURE_SIZE) is "
					  << texSize << "\n";
		}

		rgb = rgb.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
		if (in_alpha) alpha = alpha.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
	}

	const int width = rgb.getWidth();
	const int height = rgb.getHeight();

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
	{
		const std::string sSec = mrpt::format(
			"opengl_texture: load %ix%i %s %stransp", width, height,
			rgb->isColor() ? "RGB" : "BW", m_enableTransparency ? "" : "no ");
		tim.enter(sSec.c_str());
	}
#endif

	// GL_LUMINANCE and GL_LUMINANCE_ALPHA were removed in OpenGL 3.1
	// Convert grayscale images into color:
	if (!rgb.isColor()) rgb = rgb.colorImage();

	// ----------------------------------------------
	// Color texture WITH alpha channel
	// ----------------------------------------------
	if (o.enableTransparency)
	{
		ASSERT_(!alpha.isColor());
		ASSERT_EQUAL_(alpha.getWidth(), rgb.getWidth());
		ASSERT_EQUAL_(alpha.getHeight(), rgb.getHeight());

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
			unsigned char* ptrSrcCol = rgb(0, y, 0);
			unsigned char* ptrSrcAlfa = alpha(0, y);
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
			(rgb.getChannelsOrder() == std::string("RGB"));
		const GLenum img_format = (is_RGB_order ? GL_RGBA : GL_BGRA);

		// Send image data to OpenGL:
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, width);
		glTexImage2D(
			GL_TEXTURE_2D, 0 /*level*/, GL_RGBA8 /* RGB components */, width,
			height, 0 /*border*/, img_format, img_type, dataAligned);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);	 // Reset
		CHECK_OPENGL_ERROR_IN_DEBUG();

	}  // End of color texture WITH trans.
	else
	{
		// Color texture without transparency,
		// or with integrated RGBA alpha channel
		// --------------------------------------
		// Prepare image data types:
		const GLenum img_type = GL_UNSIGNED_BYTE;
		const int nBytesPerPixel = rgb.channelCount();
		// Reverse RGB <-> BGR order?
		const bool is_RGB_order =
			(rgb.getChannelsOrder() == std::string("RGB"));
		const GLenum img_format = [=]() {
			switch (nBytesPerPixel)
			{
				case 1: return GL_LUMINANCE;
				case 3: return (is_RGB_order ? GL_RGB : GL_BGR);
				case 4: return GL_BGRA;
			};
			THROW_EXCEPTION("Invalid texture image channel count.");
		}();

		// Send image data to OpenGL:
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		glPixelStorei(
			GL_UNPACK_ROW_LENGTH, rgb.getRowStride() / nBytesPerPixel);
		CHECK_OPENGL_ERROR_IN_DEBUG();
		glTexImage2D(
			GL_TEXTURE_2D, 0 /*level*/,
			nBytesPerPixel == 3 ? GL_RGB8 : GL_RGBA8 /* RGB components */,
			width, height, 0 /*border*/, img_format, img_type,
			rgb.ptrLine<uint8_t>(0));
		CHECK_OPENGL_ERROR_IN_DEBUG();
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);	 // Reset
		CHECK_OPENGL_ERROR_IN_DEBUG();

	}  // End of color texture WITHOUT trans.

	// Was: m_texture_is_loaded = true;
	// Now this situation is represented by the optional m_glTexture having
	// a valid value.

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
	{
		const std::string sSec = mrpt::format(
			"opengl_texture: load %ix%i %s %stransp", width, height,
			rgb->isColor() ? "RGB" : "BW", m_enableTransparency ? "" : "no ");
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
			CRenderizableShaderTexturedTriangles_MemPoolParams mem_params;
			mem_params.len = data.size();

			auto* mem_block =
				new CRenderizableShaderTexturedTriangles_MemPoolData();
			data.swap(mem_block->data);

			pool->dump_to_pool(mem_params, mem_block);
		}
	}
#endif
#endif
}

void COpenGLTexture::bind()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	glActiveTexture(GL_TEXTURE0 + get()->unit);
	glBindTexture(GL_TEXTURE_2D, get()->name);
	CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}
