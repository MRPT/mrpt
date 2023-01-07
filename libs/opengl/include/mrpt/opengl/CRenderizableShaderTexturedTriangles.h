/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

#include <shared_mutex>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the triangles-with-a-texture
 * shader.
 *
 *  \sa CTexturedPlane, opengl::CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderTexturedTriangles : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderTexturedTriangles)

   public:
	CRenderizableShaderTexturedTriangles() = default;
	virtual ~CRenderizableShaderTexturedTriangles() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {
			m_enableLight ? DefaultShaderID::TEXTURED_TRIANGLES_LIGHT
						  : DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_TexturedTriangles() = 0;

	// See base docs
	void freeOpenGLResources() override
	{
		m_vertexBuffer.destroy();
		m_vao.destroy();
	}

	/** Assigns a texture and a transparency image, and enables transparency (If
	 * the images are not 2^N x 2^M, they will be internally filled to its
	 * dimensions to be powers of two)
	 * \note Images are copied, the original ones can be deleted.
	 */
	void assignImage(
		const mrpt::img::CImage& img, const mrpt::img::CImage& imgAlpha);

	/** Assigns a texture image, and disable transparency.
	 * \note Images are copied, the original ones can be deleted. */
	void assignImage(const mrpt::img::CImage& img);

	/** Similar to assignImage, but the passed images are moved in (move
	 * semantic). */
	void assignImage(mrpt::img::CImage&& img, mrpt::img::CImage&& imgAlpha);

	/** Similar to assignImage, but with move semantics. */
	void assignImage(mrpt::img::CImage&& img);

	bool isLightEnabled() const { return m_enableLight; }
	void enableLight(bool enable = true) { m_enableLight = enable; }

	/** Control whether to render the FRONT, BACK, or BOTH (default) set of
	 * faces. Refer to docs for glCullFace() */
	void cullFaces(const TCullFace& cf) { m_cullface = cf; }
	TCullFace cullFaces() const { return m_cullface; }

	/** VERY IMPORTANT: If you use a multi-thread application, you MUST call
	 * this from the same thread that will later destruct the object in order to
	 * the OpenGL texture memory to be correctly deleted.
	 *  Calling this method more than once has no effects. If you use one
	 * thread, this method will be automatically called when rendering, so there
	 * is no need to explicitly call it.
	 */
	virtual void initializeTextures() const override;

	const mrpt::img::CImage& getTextureImage() const { return m_textureImage; }

	const mrpt::img::CImage& getTextureAlphaImage() const
	{
		return m_textureImageAlpha;
	}

	bool textureImageHasBeenAssigned() const { return m_textureImageAssigned; }

	/** Enable linear interpolation of textures (default=false, use nearest
	 * pixel) */
	void enableTextureLinearInterpolation(bool enable)
	{
		m_textureInterpolate = enable;
	}

	/** @name Raw access to textured-triangle shader buffer data
	 * @{ */
	const auto& shaderTexturedTrianglesBuffer() const { return m_triangles; }
	auto& shaderTexturedTrianglesBufferMutex() const { return m_trianglesMtx; }
	/** @} */

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;
	mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_trianglesMtx;

	/** Returns the bounding box of m_triangles, or (0,0,0)-(0,0,0) if empty. */
	const mrpt::math::TBoundingBoxf trianglesBoundingBox() const;

	void writeToStreamTexturedObject(mrpt::serialization::CArchive& out) const;
	void readFromStreamTexturedObject(mrpt::serialization::CArchive& in);

	using texture_name_t = unsigned int;
	/// the "i" in GL_TEXTUREi
	using texture_unit_t = int;

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

   private:
	bool m_enableLight = true;
	TCullFace m_cullface = TCullFace::NONE;

	mutable mrpt::containers::PerThreadDataHolder<
		std::optional<texture_name_unit_t>>
		m_glTexture;
	bool m_textureImageAssigned = false;
	mutable mrpt::img::CImage m_textureImage{4, 4};
	mutable mrpt::img::CImage m_textureImageAlpha;

	/** Of the texture using "m_textureImageAlpha" */
	mutable bool m_enableTransparency{false};

	bool m_textureInterpolate = false;

	void unloadTexture();

	/// Returns: [texture name, texture unit]
	static texture_name_unit_t getNewTextureNumber();
	static void releaseTextureName(const texture_name_unit_t& t);

	mutable COpenGLBuffer m_vertexBuffer;
	mutable COpenGLVertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
