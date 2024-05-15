/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/Scene.h>

#include <cstdint>

#include "bonxai/bonxai.hpp"

namespace mrpt::maps
{
/** An mrpt CMetricMap wrapper for Bonxai's VoxelMap container.
 *
 * Refer to Davide Faconti's [Bonxai
 * repository](https://github.com/facontidavide/Bonxai) for publication and
 * algorithm details, but in short, this is a sparse generic container for
 * voxels, not needing redimensioning when the map grows, and with efficient
 * insertion and update operations.
 *
 * Users normally use the derived classes, not this generic base template.
 * This base class implements all common aspects to CMetricMap that do not
 * depend on the specific contents to be stored at each voxel.
 *
 * No multi-threading protection is applied at all in the API.
 *
 * \sa CMetricMap, the example in "MRPT/samples/maps_voxelmap_simple",
 * \ingroup mrpt_maps_grp
 */
template <typename node_t>
class CVoxelMapBase : public mrpt::maps::CMetricMap
{
 public:
  using myself_t = CVoxelMapBase<node_t>;
  using voxel_node_t = node_t;

  /** Constructor, defines the resolution of the voxelmap
   *  (length of each voxel side, in meters).
   */
  CVoxelMapBase(double resolution, uint8_t inner_bits = 2, uint8_t leaf_bits = 3) :
      m_impl(std::make_unique<Impl>(resolution, inner_bits, leaf_bits))
  {
  }
  virtual ~CVoxelMapBase() override = default;

  CVoxelMapBase(const CVoxelMapBase& o) : CVoxelMapBase(o.grid().resolution) { *this = o; }
  CVoxelMapBase& operator=(const CVoxelMapBase& o)
  {
    // grid() = o.grid();
    THROW_EXCEPTION("Bonxai voxel grid copy not implemented");
    return *this;
  }

  CVoxelMapBase(CVoxelMapBase&& o) : m_impl(std::move(o.m_impl)) {}
  CVoxelMapBase& operator=(CVoxelMapBase&& o)
  {
    m_impl = std::move(o.m_impl);
    return *this;
  }

  const Bonxai::VoxelGrid<node_t>& grid() const { return m_impl->grid; }

  /** Returns a short description of the map. */
  std::string asString() const override { return "Voxelmap"; }

  /** Returns a 3D object representing the map.
   * \sa renderingOptions
   */
  void getVisualizationInto(mrpt::opengl::CSetOfObjects& o) const override
  {
    auto gl_obj = mrpt::opengl::COctoMapVoxels::Create();
    this->getAsOctoMapVoxels(*gl_obj);
    o.insert(gl_obj);
  }

  /** Builds a renderizable representation of the octomap as a
   * mrpt::opengl::COctoMapVoxels object.
   * Implementation defined for each children class.
   * \sa renderingOptions
   */
  virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels& gl_obj) const = 0;

  virtual void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override
  {
    MRPT_START
    // Save as 3D Scene:
    {
      mrpt::opengl::Scene scene;
      scene.insert(this->getVisualization());
      const std::string fil = filNamePrefix + std::string("_3D.3Dscene");
      scene.saveToFile(fil);
    }
    // Save binary data file?
    MRPT_END
  }

 protected:
  struct Impl
  {
    Impl(double resolution, uint8_t inner_bits, uint8_t leaf_bits) :
        grid(resolution, inner_bits, leaf_bits), accessor(grid.createAccessor())
    {
    }
    Impl(Bonxai::VoxelGrid<node_t>&& g) : grid(std::move(g)), accessor(grid.createAccessor()) {}
    Bonxai::VoxelGrid<node_t> grid;
    mutable typename Bonxai::VoxelGrid<node_t>::Accessor accessor;
  };
  std::unique_ptr<Impl> m_impl;
};

}  // namespace mrpt::maps
