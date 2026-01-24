/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/img/CImage.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CVisualObject.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::viz
{
/** This class can load & render 3D models in a number of different formats
 * (requires the library assimp).
 *
 * Supported formats (via Assimp library):
 * http://assimp.sourceforge.net/main_features_formats.html
 *
 * Most common ones: AutoCAD DXF (.dxf), Collada (.dae), Blender 3D (.blend),
 * 3ds Max 3DS (.3ds), 3ds Max ASE (.ase), Quake I (.mdl), Quake II (.md2),
 * Quake III Mesh (.md3), Wavefront OBJ (.obj), Stanford PLY (.ply),
 * glTF (.gltf, .glb), FBX (.fbx), etc.
 *
 * Models are loaded via CAssimpModel::loadScene()
 *
 * **Architecture (MRPT 3.0)**:
 * - This class is part of mrpt::viz (no OpenGL dependencies)
 * - It loads 3D models using Assimp and converts them to mrpt::viz primitives
 * - Textured meshes become CSetOfTexturedTriangles children
 * - Non-textured meshes become triangle data (VisualObjectParams_Triangles)
 * - Point clouds become point data (VisualObjectParams_Points)
 * - Wireframe elements become line data (VisualObjectParams_Lines)
 *
 * The class is a CSetOfObjects container that holds:
 * - One CSetOfTexturedTriangles per texture used in the model
 * - One CSetOfTriangles for all non-textured triangles
 * - Additional child objects for points and lines if present
 *
 * ![mrpt::viz::CAssimpModel](preview_CAssimpModel.png)
 *
 * \sa mrpt::viz::Scene, CSetOfObjects, CSetOfTexturedTriangles
 * \ingroup mrpt_viz_grp
 */
class CAssimpModel : public CSetOfObjects
{
  DEFINE_SERIALIZABLE(CAssimpModel, mrpt::viz)

 public:
  CAssimpModel() = default;

  /** Import flags for loadScene.
   *
   * These can be OR'd together. See Assimp documentation for details.
   * \note Not defined as ``enum class`` to allow C++-valid or-wise combinations
   */
  struct LoadFlags
  {
    enum flags_t : uint16_t
    {
      /** See: aiProcessPreset_TargetRealtime_Fast
       * Basic optimizations, fast loading. */
      RealTimeFast = 0x0001,

      /** See: aiProcessPreset_TargetRealtime_Quality
       * Good balance of quality and speed. */
      RealTimeQuality = 0x0002,

      /** See: aiProcessPreset_TargetRealtime_MaxQuality
       * Maximum quality, slower loading. */
      RealTimeMaxQuality = 0x0004,

      /** See: aiProcess_FlipUVs
       * Flip texture V coordinates (needed for some formats). */
      FlipUVs = 0x0010,

      /** MRPT-specific: ignore material colors from the file and use
       * the base class CVisualObject uniform color instead.
       * Useful when you want to override the model's colors.
       * \note (New in MRPT 2.5.0) */
      IgnoreMaterialColor = 0x0100,

      /** MRPT-specific: ignore textures from the file.
       * The model will be rendered with solid colors only.
       * \note (New in MRPT 3.0.0) */
      IgnoreTextures = 0x0200,

      /** Displays verbose messages during loading (textures, meshes, etc.) */
      Verbose = 0x1000
    };
  };

  /** @name Model Loading
   * @{ */

  /** Loads a 3D scene from a file in any Assimp-supported format.
   *
   * This clears any previously loaded content and populates this object
   * with the loaded model's geometry, materials, and textures.
   *
   * \param file_name Path to the 3D model file
   * \param flags Combination of LoadFlags values (OR'd together)
   *
   * \exception std::runtime_error On any error during loading or importing
   *
   * Example:
   * \code
   * CAssimpModel model;
   * model.loadScene("robot.dae", CAssimpModel::LoadFlags::RealTimeMaxQuality);
   * scene->insert(model);
   * \endcode
   *
   * \note Textures are loaded from paths relative to the model file's directory
   */
  void loadScene(
      const std::string& file_name,
      int flags = LoadFlags::RealTimeMaxQuality | LoadFlags::FlipUVs | LoadFlags::Verbose);

  /** Clear the loaded model and all child objects */
  void clear();

  /** Returns the path of the currently loaded model, or empty string if none */
  [[nodiscard]] const std::string& getModelPath() const { return m_modelPath; }

  /** Returns the flags used when loading the current model */
  [[nodiscard]] uint32_t getModelLoadFlags() const { return m_modelLoadFlags; }

  /** @} */

  /** @name Rendering Options
   * @{ */

  /** Enable (or disable if set to 0.0f) splitting of textured triangles
   * into separate renderable objects based on spatial bounding boxes.
   *
   * This is required only for semi-transparent objects with overlapping
   * regions, to ensure correct depth sorting during rendering.
   *
   * \param bbox_size Size of the bounding box for splitting (0.0 = disabled)
   */
  void setSplitTrianglesRenderingBBox(float bbox_size);

  /** Returns the current triangle splitting bbox size (0.0 = disabled) */
  [[nodiscard]] float getSplitTrianglesRenderingBBox() const
  {
    return m_splitTrianglesRenderingBBox;
  }

  /** @} */

  /** @name Model Information
   * @{ */

  /** Returns the number of textured mesh groups in the loaded model */
  [[nodiscard]] size_t getTexturedMeshCount() const { return m_texturedMeshes.size(); }

  /** Returns the number of non-textured triangles in the model */
  [[nodiscard]] size_t getNonTexturedTriangleCount() const;

  /** Returns the total vertex count across all meshes */
  [[nodiscard]] size_t getTotalVertexCount() const;

  /** Returns the total triangle count across all meshes */
  [[nodiscard]] size_t getTotalTriangleCount() const;

  /** Information about a texture used in the model */
  struct TextureInfo
  {
    std::string filepath;      //!< Path to the texture file
    size_t width = 0;          //!< Texture width in pixels
    size_t height = 0;         //!< Texture height in pixels
    bool hasAlpha = false;     //!< Whether texture has alpha channel
    size_t triangleCount = 0;  //!< Number of triangles using this texture
  };

  /** Returns information about all textures in the model */
  [[nodiscard]] std::vector<TextureInfo> getTextureInfo() const;

  /** @} */

  /** @name CVisualObject Interface
   * @{ */

  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  /** @} */

 private:
  // Model file info
  std::string m_modelPath;
  uint32_t m_modelLoadFlags = 0;
  std::string m_modelDirectory;  //!< Directory containing the model file

  // Rendering options
  float m_splitTrianglesRenderingBBox = 0.0f;

  // Loaded content (pointers to child objects for easy access)
  std::vector<CSetOfTexturedTriangles::Ptr> m_texturedMeshes;
  CSetOfTriangles::Ptr m_nonTexturedMesh;

  // Texture cache: filepath -> texture info
  struct LoadedTexture
  {
    mrpt::img::CImage rgb;
    std::optional<mrpt::img::CImage> alpha;
  };
  std::map<std::string, LoadedTexture> m_textureCache;

  // Cached bounding box
  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedBBox;
  mrpt::math::TPoint3Df m_bboxMin{0, 0, 0};
  mrpt::math::TPoint3Df m_bboxMax{0, 0, 0};

  // Assimp scene (pimpl to avoid including assimp headers)
  struct AssimpSceneWrapper;
  mrpt::pimpl<AssimpSceneWrapper> m_assimpScene;

  /** @name Internal Processing
   * @{ */

  /** Process the loaded Assimp scene and convert to mrpt::viz objects */
  void processAssimpScene();

  /** Recursively process Assimp nodes */
  void processNode(
      const void* node,   // aiNode*
      const void* scene,  // aiScene*
      const mrpt::poses::CPose3D& parentTransform);

  /** Process a single Assimp mesh */
  void processMesh(
      const void* mesh,   // aiMesh*
      const void* scene,  // aiScene*
      const mrpt::poses::CPose3D& transform);

  /** Load and cache a texture */
  const LoadedTexture* loadTexture(const std::string& texturePath);

  /** Get or create a CSetOfTexturedTriangles for a texture */
  CSetOfTexturedTriangles::Ptr getOrCreateTexturedMesh(const std::string& texturePath);

  /** Apply triangle splitting for transparency sorting */
  void applySplitTrianglesRendering();

  /** Update bounding box from mesh data */
  void updateBoundingBox(const mrpt::math::TPoint3Df& point);

  /** @} */
};

}  // namespace mrpt::viz