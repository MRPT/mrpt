/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the triangles-with-a-texture
 * shader.
 *
 *  \sa CTexturedPlane, opengl::CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderTexturedTriangles : public CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderTexturedTriangles)

   public:
	CRenderizableShaderTexturedTriangles() = default;
	virtual ~CRenderizableShaderTexturedTriangles() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TEXTURED_TRIANGLES};
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

	/** VERY IMPORTANT: If you use a multi-thread application, you MUST call
	 * this from the same thread that will later destruct the object in order to
	 * the OpenGL texture memory to be correctly deleted.
	 *  Calling this method more than once has no effects. If you use one
	 * thread, this method will be automatically called when rendering, so there
	 * is no need to explicitly call it.
	 */
	void initializeTextures() const;

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;

	void writeToStreamTexturedObject(mrpt::serialization::CArchive& out) const;
	void readFromStreamTexturedObject(mrpt::serialization::CArchive& in);

	/** These are updated in initializeTextures() when uploading a texture to
	 * opengl memory. */
	mutable float m_tex_x_min = .0, m_tex_x_max = 1.0f, m_tex_y_min = .0f,
				  m_tex_y_max = 1.0f;

   private:
	bool m_enableLight = false;

	mutable unsigned int m_glTextureName{0};
	mutable bool m_texture_is_loaded{false};
	mutable mrpt::img::CImage m_textureImage{4, 4};
	mutable mrpt::img::CImage m_textureImageAlpha;

	/** Of the texture using "m_textureImageAlpha" */
	mutable bool m_enableTransparency{false};

	/** Size of the texture image, rounded up to next power of 2 */
	mutable int r_width{1}, r_height{1};

	/** The size of the fill in pixels in the textured image, w.r.t the image
	 * passed by the user. */
	mutable int m_pad_x_right{0}, m_pad_y_bottom{0};

	void unloadTexture();

	static unsigned int getNewTextureNumber();
	static void releaseTextureName(unsigned int i);

	mutable COpenGLBuffer m_vertexBuffer;
	mutable COpenGLVertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
