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
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
/** A base class for all OpenGL objects with loadable textures.
 *  \sa opengl::COpenGLScene, opengl::CTexturedPlane,
 * opengl::CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class CTexturedObject : public CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CTexturedObject)

   protected:
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

	~CTexturedObject() override;
	void unloadTexture();

	virtual void render_pre() const;
	virtual void render_post() const;

	/** Must be implemented by derived classes */
	virtual void render_texturedobj() const = 0;

	void writeToStreamTexturedObject(mrpt::serialization::CArchive& out) const;
	void readFromStreamTexturedObject(mrpt::serialization::CArchive& in);

   public:
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

	/** Similar to assignImage, but the passed images will be returned as empty:
	 * it avoids making a copy of the whole image, just copies a pointer. */
	void assignImage_fast(mrpt::img::CImage& img, mrpt::img::CImage& imgAlpha);

	/** Similar to assignImage, but the passed images will be returned as empty:
	 * it avoids making a copy of the whole image, just copies a pointer.  */
	void assignImage_fast(mrpt::img::CImage& img);

	/** VERY IMPORTANT: If you use a multi-thread application, you MUST call
	 * this from the same thread that will later destruct the object in order to
	 * the OpenGL texture memory to be correctly deleted.
	 *  Calling this method more than once has no effects. If you use one
	 * thread, this method will be automatically called when rendering, so there
	 * is no need to explicitly call it.
	 */
	void loadTextureInOpenGL() const;

	void render() const override;
	void renderUpdateBuffers() const override;
};

}  // namespace mrpt::opengl
