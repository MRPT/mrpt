/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CTexturedObject_H
#define opengl_CTexturedObject_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/math/geometry.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CTexturedObject;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CTexturedObject, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A base class for all OpenGL objects with loadable textures.
		  *  \sa opengl::COpenGLScene, opengl::CTexturedPlane, opengl::CSetOfTexturedTriangles
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CTexturedObject : public CRenderizableDisplayList
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CTexturedObject )

		protected:
			mutable unsigned int		m_glTextureName;
			mutable bool				m_texture_is_loaded;
			mutable mrpt::utils::CImage	m_textureImage;
			mutable mrpt::utils::CImage	m_textureImageAlpha;
			mutable bool				m_enableTransparency;  //!< Of the texture using "m_textureImageAlpha"
			mutable int					r_width,r_height;		//!< Size of the texture image, rounded up to next power of 2
			mutable int					m_pad_x_right,m_pad_y_bottom; //!< The size of the fill in pixels in the textured image, w.r.t the image passed by the user.

			CTexturedObject();
			virtual ~CTexturedObject();
			void unloadTexture();

			virtual void  render_pre()  const;
			virtual void  render_post() const;

			virtual void  render_texturedobj() const = 0;  //!< Must be implemented by derived classes

			void  writeToStreamTexturedObject(mrpt::utils::CStream &out) const;
			void  readFromStreamTexturedObject(mrpt::utils::CStream &in);

		public:
			/** Assigns a texture and a transparency image, and enables transparency (If the images are not 2^N x 2^M, they will be internally filled to its dimensions to be powers of two)
			  * \note Images are copied, the original ones can be deleted.
			  */
			void  assignImage(
				const mrpt::utils::CImage&	img,
				const mrpt::utils::CImage&	imgAlpha );

			/** Assigns a texture image, and disable transparency.
			  * \note Images are copied, the original ones can be deleted. */
			void  assignImage(const mrpt::utils::CImage& img );

			/** Similar to assignImage, but the passed images will be returned as empty: it avoids making a copy of the whole image, just copies a pointer. */
			void  assignImage_fast(
				mrpt::utils::CImage&	img,
				mrpt::utils::CImage&	imgAlpha );

			/** Similar to assignImage, but the passed images will be returned as empty: it avoids making a copy of the whole image, just copies a pointer.  */
			void  assignImage_fast(mrpt::utils::CImage&	img );

			/** VERY IMPORTANT: If you use a multi-thread application, you MUST call this from the same thread that will later destruct the object in order to the OpenGL texture memory to be correctly deleted.
			  *  Calling this method more than once has no effects. If you use one thread, this method will be automatically called when rendering, so there is no need to explicitly call it.
			  */
			void loadTextureInOpenGL() const;

			virtual void  render_dl() const;

		};

	} // end namespace

} // End of namespace


#endif
