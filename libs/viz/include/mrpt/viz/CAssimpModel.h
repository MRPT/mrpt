/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/viz/CVisualObject.h>

#include <map>
#include <optional>

namespace mrpt::viz
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
 * ![mrpt::viz::CAssimpModel](preview_CAssimpModel.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CAssimpModel : public CVisualObject
{
  DEFINE_SERIALIZABLE(CAssimpModel, mrpt::viz)

 public:
  CAssimpModel();
  virtual ~CAssimpModel() override;

  /** Import flags for loadScene
   *  \note Not defined as ``enum class`` to allow C++-valid or-wise combinations
   */
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
      const int flags = LoadFlags::RealTimeMaxQuality | LoadFlags::FlipUVs | LoadFlags::Verbose);

  /** Empty the object */
  void clear();

  /* Simulation of ray-trace. */
  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Enable (or disable if set to .0f) a feature in which textured triangles
   *  are split into different renderizable smaller objects.
   *  This is required only for semitransparent objects with overlaping regions.
   */
  void split_triangles_rendering_bbox(const float bbox_size);

  [[nodiscard]] float split_triangles_rendering_bbox() const
  {
    return m_split_triangles_rendering_bbox;
  }

 private:
  filepath_t m_modelPath;
  uint32_t m_modelLoadFlags = 0;
  float m_split_triangles_rendering_bbox = .0f;
};

}  // namespace mrpt::viz
