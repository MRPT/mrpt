/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CAssimpModel_H
#define opengl_CAssimpModel_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/utils/CMemoryChunk.h>
#include <map>

namespace mrpt
{
	namespace opengl
	{


		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAssimpModel, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** This class can load & render 3D models in a number of different formats (requires the library assimp).
		  *  - All supported formats: http://assimp.sourceforge.net/main_features_formats.html
		  *  - Most common ones: AutoCAD DXF ( .dxf ), Collada ( .dae ), Blender 3D ( .blend ), 3ds Max 3DS ( .3ds ), 3ds Max ASE ( .ase ), Quake I ( .mdl ), Quake II ( .md2 ), Quake III Mesh ( .md3 ), etc.
		  *
		  *  Models are loaded via CAssimpModel::loadScene()
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CAssimpModel </td> <td> \image html preview_CAssimpModel.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  *  \sa opengl::COpenGLScene
		  * \ingroup mrpt_opengl_grp
		  * \note Class introduced in MRPT 1.2.2
		  */
		class OPENGL_IMPEXP CAssimpModel : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CAssimpModel )

		public:
			void  render_dl() const MRPT_OVERRIDE; //!< Render child objects

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/**  Loads a scene from a file in any supported file. 
			  * \exception std::runtime_error On any error during loading or importing the file.
			  */
			void loadScene( const std::string &file_name );

			/** Empty the object */
			void   clear();

			/** Evaluates the scene at a given animation time */
			void evaluateAnimation( double time_anim );

			/* Simulation of ray-trace. */
			bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;

			struct TInfoPerTexture
			{
				size_t id_idx; //!< indices in \a m_textureIds. string::npos for non-initialized ones.
				mrpt::utils::CImagePtr img_rgb, img_alpha;
				TInfoPerTexture() : id_idx(std::string::npos) {}
			};

		private:
			CAssimpModel( );
			virtual ~CAssimpModel(); //!< Private, virtual destructor: only can be deleted from smart pointers

			/** A container for automatic deletion of lib3ds's scene when the last reference of the smart_ptr's is destroyed.
			  */
			struct TImplAssimp
			{
				TImplAssimp();
				~TImplAssimp();
				void	*scene;	//!< aiScene*
			};
			stlplus::smart_ptr<TImplAssimp>	m_assimp_scene;

			mrpt::math::TPoint3D   m_bbox_min, m_bbox_max; //!< Bounding box

			mutable bool m_textures_loaded;
			std::string m_modelPath;
			mutable std::vector<unsigned int> m_textureIds;

			mutable std::map<std::string,TInfoPerTexture> m_textureIdMap;

		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CAssimpModel, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace
} // End of namespace

#endif
