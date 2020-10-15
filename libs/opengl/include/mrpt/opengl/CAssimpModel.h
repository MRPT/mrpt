/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <map>
#include <optional>

// Forward decls:
// clang-format off
struct aiScene;
struct aiNode;
namespace mrpt::opengl::internal { struct RenderElements; }
// clang-format on

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
class CAssimpModel : public CRenderizableShaderTriangles,
					 public CRenderizableShaderWireFrame,
					 public CRenderizableShaderPoints
{
	DEFINE_SERIALIZABLE(CAssimpModel, mrpt::opengl)

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	virtual shader_list_t requiredShaders() const override
	{
		// May use up to two shaders (triangles and lines):
		return {DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES,
				DefaultShaderID::POINTS};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	void onUpdateBuffers_Points() override;
	void onUpdateBuffers_all();  // special case for assimp
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
		CRenderizableShaderPoints::freeOpenGLResources();
	}
	void enqueForRenderRecursive(
		const mrpt::opengl::TRenderMatrices& state,
		RenderQueue& rq) const override;
	/** @} */

	CAssimpModel();
	virtual ~CAssimpModel() override;

	/** Import flags for loadScene */
	struct LoadFlags
	{
		enum flags_t : uint32_t
		{
			/** See: aiProcessPreset_TargetRealtime_Fast */
			RealTimeFast = 0x0001,
			/** See: aiProcessPreset_TargetRealtime_Quality */
			RealTimeQuality = 0x0002,
			/** See: aiProcessPreset_TargetRealtime_MaxQuality */
			RealTimeMaxQuality = 0x0004,
			/** See: aiProcess_FlipUVs */
			FlipUVs = 0x0010,
			/** Displays messages on loaded textures, etc. */
			Verbose = 0x1000
		};
	};

	/**  Loads a scene from a file in any supported file.
	 * \exception std::runtime_error On any error during loading or importing
	 * the file.
	 */
	void loadScene(
		const std::string& file_name,
		const int flags = LoadFlags::RealTimeMaxQuality | LoadFlags::FlipUVs |
						  LoadFlags::Verbose);

	/** Empty the object */
	void clear();

	/* Simulation of ray-trace. */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	struct TInfoPerTexture
	{
		// indices in \a m_texturedObjects. string::npos for non-initialized
		// ones
		size_t id_idx{std::string::npos};
		mrpt::img::CImage img_rgb;
		std::optional<mrpt::img::CImage> img_alpha;
	};

   private:
	/** The interface to the file: */
	struct Impl;
	mrpt::pimpl<Impl> m_assimp_scene;

	/** Bounding box */
	mrpt::math::TPoint3D m_bbox_min{0, 0, 0}, m_bbox_max{0, 0, 0};

	std::string m_modelPath;

	mutable bool m_textures_loaded{false};
	mutable std::map<std::string, TInfoPerTexture> m_textureIdMap;

	// We define a textured object per texture image, and delegate texture
	// handling to that class:
	mutable std::vector<CSetOfTexturedTriangles::Ptr> m_texturedObjects;
	bool m_verboseLoad = true;

	void recursive_render(
		const aiScene* sc, const aiNode* nd, const mrpt::poses::CPose3D& transf,
		mrpt::opengl::internal::RenderElements& re);
	void process_textures(const aiScene* scene);

};  // namespace mrpt::opengl

}  // namespace mrpt::opengl
