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
using texture_unit_t = int;	 //!< the "i" in GL_TEXTUREi

/** Texture "name" and "unit". \sa COpenGLTexture \ingroup mrpt_opengl_grp */
struct texture_name_unit_t
{
	texture_name_unit_t() = default;
	texture_name_unit_t(texture_name_t Name, texture_unit_t Unit)
		: name(Name), unit(Unit)
	{
	}

	texture_name_t name = 0;
	texture_unit_t unit = 0;  //!< the "i" in GL_TEXTUREi
};

/** Resource management for OpenGL textures.
 *
 * \sa CRenderizableShaderTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class COpenGLTexture
{
   public:
	COpenGLTexture() = default;
	~COpenGLTexture() = default;

	/// Options while creating a texture from an image.
	struct Options
	{
		Options() = default;

		bool generateMipMaps = true;

		/** If set (true), interpolation will happen when getting closer to the
		 * texture (magnify). Otherwise (false), it will be not interpolated, so
		 * each pixel will be rendered as a square box with constant color (e.g.
		 * suitable for gridmaps) */
		bool magnifyLinearFilter = true;

		bool enableTransparency = false;
	};

	/** This is how an image is loaded into this object, and a texture ID is
	 * generated underneath.
	 * Valid image formats are 8bit per channel RGB or RGBA.
	 */
	void assignImage(const mrpt::img::CImage& rgb, const Options& o);

	/// \overload With alpha (transparency) channel as an independent image
	void assignImage(
		const mrpt::img::CImage& rgb, const mrpt::img::CImage& alpha,
		const Options& o);

	/** Returns true if an image has been already assigned and an OpenGL
	 * texture ID was already generated. */
	bool initialized() const;

	/** Binds the texture */
	void bind();

	void unloadTexture();

	/**  Texture unit = the "i" in GL_TEXTUREi */
	texture_unit_t textureUnit() const { return m_tex.get()->unit; }
	texture_name_t textureNameID() const { return m_tex.get()->name; }

   private:
	template <class T>
	using ptdh = mrpt::containers::PerThreadDataHolder<T>;

	mutable ptdh<std::optional<texture_name_unit_t>> m_tex;

	const auto& get() const { return m_tex.get(); }
	auto& get() { return m_tex.get(); }

	void internalAssignImage(
		const mrpt::img::CImage* in_rgb, const mrpt::img::CImage* in_alpha,
		const Options& o);
};

// Normally users should not need to call these, but they are exposed just in
// case they are useful someday.
texture_name_unit_t getNewTextureNumber();
void releaseTextureName(const texture_name_unit_t& t);

}  // namespace mrpt::opengl
