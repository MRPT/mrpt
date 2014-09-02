/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CAssimpModel_H
#define opengl_CAssimpModel_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/utils/CMemoryChunk.h>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CAssimpModel;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAssimpModel, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** This class can load & render 3D models in a number of different formats (requires the library assimp).
		  *  - All supported formats: http://assimp.sourceforge.net/main_features_formats.html
		  *  - Most common ones: AutoCAD DXF ( .dxf ), Collada ( .dae ), Blender 3D ( .blend ), 3ds Max 3DS ( .3ds ), 3ds Max ASE ( .ase ), Quake I ( .mdl ), Quake II ( .md2 ), Quake III Mesh ( .md3 ), etc.
		  *
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  * \note Class introduced in MRPT 1.2.2
		  */
		class OPENGL_IMPEXP CAssimpModel : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CAssimpModel )

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
			CAssimpModel( );

			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CAssimpModel();

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
