/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CRenderizable.h>
#include <map>

namespace mrpt::opengl
{
/** This class can load & render 3D models in a number of different formats
 * (requires the library assimp).
 *  - All supported formats:
 * http://assimp.sourceforge.net/main_features_formats.html
 *  - Most common ones: AutoCAD DXF ( .dxf ), Collada ( .dae ), Blender 3D (
 * .blend ), 3ds Max 3DS ( .3ds ), 3ds Max ASE ( .ase ), Quake I ( .mdl ), Quake
 * II ( .md2 ), Quake III Mesh ( .md3 ), etc.
 *
 *  Models are loaded via CAssimpModel::loadScene()
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CAssimpModel </td> <td> \image html
 * preview_CAssimpModel.png </td> </tr>
 *  </table>
 *  </div>
 *
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 * \note Class introduced in MRPT 1.2.2
 */
class CAssimpModel : public CRenderizable
{
	DEFINE_SERIALIZABLE(CAssimpModel, mrpt::opengl)

   public:
	/** Render child objects */
	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/**  Loads a scene from a file in any supported file.
	 * \exception std::runtime_error On any error during loading or importing
	 * the file.
	 */
	void loadScene(const std::string& file_name);

	/** Empty the object */
	void clear();

	/** Evaluates the scene at a given animation time */
	void evaluateAnimation(double time_anim);

	/* Simulation of ray-trace. */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	struct TInfoPerTexture
	{
		/** indices in \a m_textureIds. string::npos for non-initialized ones.
		 */
		size_t id_idx;
		mrpt::img::CImage::Ptr img_rgb, img_alpha;
		TInfoPerTexture() : id_idx(std::string::npos) {}
	};

	CAssimpModel();
	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CAssimpModel() override;

   private:
	/** A container for automatic deletion of assimp scene when the last
	 * reference of the smart_ptr's is destroyed.
	 */
	struct TImplAssimp
	{
		TImplAssimp();
		~TImplAssimp();
		/** aiScene* */
		void* scene{nullptr};
	};
	std::shared_ptr<TImplAssimp> m_assimp_scene;

	/** Bounding box */
	mrpt::math::TPoint3D m_bbox_min, m_bbox_max;

	mutable bool m_textures_loaded{false};
	std::string m_modelPath;
	mutable std::vector<unsigned int> m_textureIds;

	mutable std::map<std::string, TInfoPerTexture> m_textureIdMap;
};

}  // namespace mrpt::opengl
