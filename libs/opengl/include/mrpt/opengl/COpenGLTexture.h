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
#include <mrpt/img/CImage.h>

#include <optional>

namespace mrpt::opengl
{
using texture_name_t = unsigned int;
/// the "i" in GL_TEXTUREi
using texture_unit_t = int;

/** Texture "name" and "unit"
 *  \sa COpenGLTexture
 * \ingroup mrpt_opengl_grp
 */
struct texture_name_unit_t
{
	texture_name_unit_t() = default;
	texture_name_unit_t(texture_name_t Name, texture_unit_t Unit)
		: name(Name), unit(Unit)
	{
	}

	texture_name_t name = 0;
	/// the "i" in GL_TEXTUREi
	texture_unit_t unit = 0;
};

/** Resource management for OpenGL texture IDs.
 *
 *  \sa CRenderizableShaderTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class COpenGLTexture
{
   public:
	COpenGLTexture() = default;
	~COpenGLTexture() = default;

	void unloadTexture();

	const auto& get() const { return m_tex.get(); }
	auto& get() { return m_tex.get(); }

   private:
	template <class T>
	using ptdh = mrpt::containers::PerThreadDataHolder<T>;

	mutable ptdh<std::optional<texture_name_unit_t>> m_tex;
};

// Normally users should not need to call these, but they are exposed just in
// case they are useful someday.
texture_name_unit_t getNewTextureNumber();
void releaseTextureName(const texture_name_unit_t& t);

}  // namespace mrpt::opengl