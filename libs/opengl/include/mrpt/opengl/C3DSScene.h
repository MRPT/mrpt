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
#ifndef opengl_C3DSScene_H
#define opengl_C3DSScene_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/utils/CMemoryChunk.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP C3DSScene;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( C3DSScene, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** This element keeps a set of objects imported from a 3DStudio file (.3ds).
		  *  This class uses the opensource library <a href="http://lib3ds.sourceforge.net/" >lib3ds</a> internally.
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP C3DSScene : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( C3DSScene )


		public:

			/** Render child objects.
			  */
			void  render_dl() const;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			/**  Loads a scene from a 3DS file (3D Studio format) into this object, from either plain .3ds format, or in gzip compressed .3ds.gz format.
			  *    Previous contents are lost.
			  *    If the file ends in ".gz", it'll be automatically decompressed using gzip (see mrpt::compress::zip).
			  */
			void loadFrom3DSFile( const std::string &file_name );

			/** Initializes all textures in the scene (See opengl::CTexturedPlane::loadTextureInOpenGL)
			  */
			void  initializeAllTextures();

			/** Empty the object */
			void   clear();

			/** Evaluates the scene at a given animation time
			  */
			void evaluateAnimation( double time_anim );

			/** Enables an extra ambient light */
			void enableExtraAmbientLight(bool enable=true) { m_enable_extra_lighting=enable; CRenderizableDisplayList::notifyChange(); }

			/* Simulation of ray-trace. */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const;

		private:
			/** Default constructor
			  */
			C3DSScene( );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~C3DSScene();

			/** A container for automatic deletion of lib3ds's scene when the last reference of the smart_ptr's is destroyed.
			  */
			struct TImpl3DS
			{
				TImpl3DS();
				~TImpl3DS();
				void	*file;	//!< Lib3dsFile*
			};

			/** An internal pointer to the lib3ds library's object of type "Lib3dsFile"
			  */
			stlplus::smart_ptr<TImpl3DS>	m_3dsfile;

			/** Scale of the object */
			//double	m_scale_x,m_scale_y,m_scale_z;
			mrpt::math::TPoint3D   m_bbox_min, m_bbox_max; //!< Bounding box

			bool	m_enable_extra_lighting;

			//float	m_light_cons_attenuation;		//!< OpenGL Light attenuation factor (default=1.0)
			//float	m_light_lin_attenuation;		//!< OpenGL Light attenuation factor (default=0.0)
			//float	m_light_quad_attenuation;		//!< OpenGL Light attenuation factor (default=0.0)
		};


	} // end namespace

} // End of namespace


#endif
