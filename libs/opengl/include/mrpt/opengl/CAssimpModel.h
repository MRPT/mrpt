/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
 * ![mrpt::opengl::CAssimpModel](preview_CAssimpModel.png)
 *
 * \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
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
		return {
			DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES,
			DefaultShaderID::POINTS};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	void onUpdateBuffers_Points() override;
	void onUpdateBuffers_all();	 // special case for assimp
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
			/** MRPT-specific: ignore materials and replace by the base class
			   CRenderizable uniform color that was defined before calling
			   loadScene(). \note (New in MRPT 2.5.0) */
			IgnoreMaterialColor = 0x0100,
			/** Displays messages on loaded textures, etc. */
			Verbose = 0x1000
		};
	};

	using filepath_t = std::string;

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

	mrpt::math::TBoundingBox getBoundingBox() const override;

	struct TInfoPerTexture
	{
		// indices in \a m_texturedObjects. string::npos for non-initialized
		// ones
		size_t id_idx = std::string::npos;
		mrpt::img::CImage img_rgb;
		std::optional<mrpt::img::CImage> img_alpha;
	};

   private:
	/** The interface to the file: */
	struct Impl;
	mrpt::pimpl<Impl> m_assimp_scene;

	/** Bounding box */
	mrpt::math::TPoint3D m_bbox_min{0, 0, 0}, m_bbox_max{0, 0, 0};

	filepath_t m_modelPath;

	mutable std::map<filepath_t, TInfoPerTexture> m_textureIdMap;

	// We define a textured object per texture image, and delegate texture
	// handling to that class:
	mutable std::vector<CSetOfTexturedTriangles::Ptr> m_texturedObjects;
	bool m_verboseLoad = false;
	bool m_ignoreMaterialColor = false;

	void recursive_render(
		const aiScene* sc, const aiNode* nd, const mrpt::poses::CPose3D& transf,
		mrpt::opengl::internal::RenderElements& re);
	void process_textures(const aiScene* scene);

};	// namespace mrpt::opengl

}  // namespace mrpt::opengl
