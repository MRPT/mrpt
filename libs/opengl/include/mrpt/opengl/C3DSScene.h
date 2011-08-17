/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

			bool	m_enable_extra_lighting;

			//float	m_light_cons_attenuation;		//!< OpenGL Light attenuation factor (default=1.0)
			//float	m_light_lin_attenuation;		//!< OpenGL Light attenuation factor (default=0.0)
			//float	m_light_quad_attenuation;		//!< OpenGL Light attenuation factor (default=0.0)
		};


	} // end namespace

} // End of namespace


#endif
