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

#include <array>
#include <optional>

namespace mrpt::opengl
{
using texture_name_t = unsigned int;
using texture_unit_t = int;	 //!< the "i" in GL_TEXTUREi

/** Texture "name" and "unit". \sa Texture \ingroup mrpt_opengl_grp */
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

/** Resource management for OpenGL 2D or Cube textures.
 *
 * The texture is generated when images are assigned via
 * assignImage2D() or assignCubeImages().
 *
 * \sa CRenderizableShaderTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class Texture
{
   public:
	Texture() = default;
	~Texture() = default;

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

	/** This is how an 2D texture image is loaded into this object, and a
	 * texture ID is generated underneath. Valid image formats are 8bit per
	 * channel RGB or RGBA.
	 */
	void assignImage2D(const mrpt::img::CImage& rgb, const Options& o);

	/// \overload With alpha (transparency) channel as an independent image
	void assignImage2D(
		const mrpt::img::CImage& rgb, const mrpt::img::CImage& alpha,
		const Options& o);

	/** This is how an Cube texture is loaded into this object, and a
	 * texture ID is generated underneath. Valid image formats are 8bit per
	 * channel RGB or RGBA.
	 *
	 * Indices of faces in the array follow the numeric ordering of
	 * mrpt::opengl::CUBE_TEXTURE_FACE values.
	 */
	void assignCubeImages(const std::array<mrpt::img::CImage, 6>& imgs);

	/** Returns true if an image has been already assigned and an OpenGL
	 * texture ID was already generated. */
	bool initialized() const;

	/** Binds the texture to GL_TEXTURE_2D */
	void bindAsTexture2D();

	/** Binds the texture to GL_TEXTURE_CUBE_MAP */
	void bindAsCubeTexture();

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

	void internalAssignImage_2D(
		const mrpt::img::CImage* in_rgb, const mrpt::img::CImage* in_alpha,
		const Options& o);
};

// Normally users should not need to call these, but they are exposed just in
// case they are useful someday.
texture_name_unit_t getNewTextureNumber();
void releaseTextureName(const texture_name_unit_t& t);

}  // namespace mrpt::opengl
