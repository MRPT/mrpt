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
