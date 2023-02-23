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
#include <mrpt/opengl/COpenGLTexture.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

#include <shared_mutex>

namespace mrpt::opengl
{
/** A Sky Box: 6 textures that are always rendered at "infinity" to give the
 *  impression of the scene to be much larger.
 *
 * \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CSkyBox : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSkyBox, mrpt::opengl)

   public:
	CSkyBox() = default;
	virtual ~CSkyBox() override = default;

	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::SKYBOX};
	}

	// Not needed, only for VAO and VBO
	void freeOpenGLResources() override {}

	/** @} */

	enum class TEXTURE_FACE
	{
		RIGHT = 0,
		LEFT,
		TOP,
		BOTTOM,
		BACK,
		FRONT
	};

	/** Assigns a texture.
	 * \note Images are copied, the original ones can be deleted.
	 */
	void assignImage(const TEXTURE_FACE face, const mrpt::img::CImage& img);

	auto internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf override;

   private:
	/// The textures for the 6 faces
	mutable std::array<COpenGLTexture, 6> m_texs;
	std::array<mrpt::img::CImage, 6> m_textureImage;

	mutable COpenGLBuffer m_vbo;
	mutable COpenGLVertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
