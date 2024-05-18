/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CLogOddsGridMapLUT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMapBase.h>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/maps/OccupancyGridCellType.h>
#include <mrpt/maps/logoddscell_traits.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
struct TVoxelMap_InsertionOptions : public mrpt::config::CLoadableOptions
{
  TVoxelMap_InsertionOptions() = default;

  double max_range = -1;  //!< Maximum insertion ray range (<0: none)

  double prob_miss = 0.45;
  double prob_hit = 0.65;
  double clamp_min = 0.10;
  double clamp_max = 0.95;

  bool ray_trace_free_space = true;
  uint32_t decimation = 1;

  /** If !=0, remove the voxels farther (L1 distance) than this
   * distance, in meters. */
  double remove_voxels_farther_than = .0;

  // See base docs
  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override;
  void saveToConfigFile(mrpt::config::CConfigFileBase& c, const std::string& s) const override;

  void writeToStream(mrpt::serialization::CArchive& out) const;
  void readFromStream(mrpt::serialization::CArchive& in);
};

/** Options used when evaluating "computeObservationLikelihood"
 * \sa CObservation::computeObservationLikelihood
 */
struct TVoxelMap_LikelihoodOptions : public mrpt::config::CLoadableOptions
{
  TVoxelMap_LikelihoodOptions() = default;
  ~TVoxelMap_LikelihoodOptions() override = default;

  // See base docs
  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override;
  void saveToConfigFile(mrpt::config::CConfigFileBase& c, const std::string& s) const override;

  void writeToStream(mrpt::serialization::CArchive& out) const;
  void readFromStream(mrpt::serialization::CArchive& in);

  /// Speed up the likelihood computation by considering only a maximum of
  /// `decimate_up_to` rays. Values <=1 mean use all measurements without
  /// decimation.
  uint32_t decimate_up_to = 0;

  /// Minimum occupancy (0,1) for a voxel to be considered occupied.
  double occupiedThreshold = 0.60;
};

/** Options for the conversion of a mrpt::maps::COctoMap into a
 * mrpt::opengl::COctoMapVoxels */
struct TVoxelMap_RenderingOptions
{
  TVoxelMap_RenderingOptions() = default;

  bool generateOccupiedVoxels = true;
  double occupiedThreshold = 0.60;
  bool visibleOccupiedVoxels = true;

  bool generateFreeVoxels = true;
  double freeThreshold = 0.40;
  bool visibleFreeVoxels = true;

  /** Binary dump to stream */
  void writeToStream(mrpt::serialization::CArchive& out) const;
  /** Binary dump to stream */
  void readFromStream(mrpt::serialization::CArchive& in);
};

namespace internal
{
template <class T, class = void>
struct has_color : std::false_type
{
};
template <class T>
struct has_color<T, std::void_t<decltype(T::color)>> : std::true_type
{
};
}  // namespace internal

/**
 * Base class for log-odds sparse voxel map for cells containing occupancy,
 * and possibly other information, for each voxel.
 *
 * \sa Use derived classes CVoxelMap, CVoxelMapRGB
 *
 * \ingroup mrpt_maps_grp
 */
template <typename voxel_node_t, typename occupancy_t = int8_t>
class CVoxelMapOccupancyBase :
    public CVoxelMapBase<voxel_node_t>,
    public detail::logoddscell_traits<occupancy_t>,
    public mrpt::maps::NearestNeighborsCapable
{
 protected:
  using occupancy_value_t = occupancy_t;
  using traits_t = detail::logoddscell_traits<occupancy_t>;
  using base_t = CVoxelMapBase<voxel_node_t>;

 public:
  CVoxelMapOccupancyBase(double resolution = 0.05, uint8_t inner_bits = 2, uint8_t leaf_bits = 3) :
      CVoxelMapBase<voxel_node_t>(resolution, inner_bits, leaf_bits)
  {
  }
  virtual ~CVoxelMapOccupancyBase() = default;

  bool isEmpty() const override { return base_t::m_impl->grid.activeCellsCount() == 0; }

  void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels& gl_obj) const override;

  /** Manually updates the occupancy of the voxel at (x,y,z) as being occupied
   * (true) or free (false), using the log-odds parameters in \a
   * insertionOptions */
  void updateVoxel(const double x, const double y, const double z, bool occupied);

  /** Get the occupancy probability [0,1] of a point
   * \return false if the point is not mapped, in which case the returned
   * "prob" is undefined. */
  bool getPointOccupancy(
      const double x, const double y, const double z, double& prob_occupancy) const;

  void insertPointCloudAsRays(
      const mrpt::maps::CPointsMap& pts,
      const mrpt::math::TPoint3D& sensorPt,
      const std::optional<const mrpt::poses::CPose3D>& sensorPose = std::nullopt);

  void insertPointCloudAsEndPoints(
      const mrpt::maps::CPointsMap& pts,
      const mrpt::math::TPoint3D& sensorPt,
      const std::optional<const mrpt::poses::CPose3D>& sensorPose = std::nullopt);

  /** Returns all occupied voxels as a point cloud. The shared_ptr is
   *  also hold and updated internally, so it is not safe to read it
   *  while also updating the voxel map in another thread.
   *
   *  The point cloud is cached, and invalidated upon map updates.
   *
   *  A voxel is considered occupied if its occupancy is larger than
   * `likelihoodOptions.occupiedThreshold` (Range: [0,1], default: 0.6)
   */
  mrpt::maps::CSimplePointsMap::Ptr getOccupiedVoxels() const;

  /** This visits all cells to calculate a bounding box, caching the result
   *  so subsequent calls are cheap until the voxelmap is changed in some way.
   */
  mrpt::math::TBoundingBoxf boundingBox() const override;

  /// The options used when inserting observations in the map:
  TVoxelMap_InsertionOptions insertionOptions;

  TVoxelMap_LikelihoodOptions likelihoodOptions;

  TVoxelMap_RenderingOptions renderingOptions;

  /** Performs Bayesian fusion of a new observation of a cell.
   * This method increases the "occupancy-ness" of a cell, managing possible
   * saturation.
   *  \param theCell The cell to modify
   *  \param logodd_obs Observation of the cell, in log-odd form as
   * transformed by p2l.
   *  \param thres  This must be CELLTYPE_MIN+logodd_obs
   * \sa updateCell, updateCell_fast_free
   */
  inline void updateCell_fast_occupied(
      voxel_node_t* theCell, const occupancy_t logodd_obs, const occupancy_t thres)
  {
    if (theCell == nullptr) return;
    occupancy_t& occ = theCell->occupancyRef();
    if (occ > thres)
      occ -= logodd_obs;
    else
      occ = traits_t::CELLTYPE_MIN;
  }

  /** Performs Bayesian fusion of a new observation of a cell.
   * This method increases the "occupancy-ness" of a cell, managing possible
   * saturation.
   *  \param coord Cell indexes.
   *  \param logodd_obs Observation of the cell, in log-odd form as
   * transformed by p2l.
   *  \param thres  This must be CELLTYPE_MIN+logodd_obs
   * \sa updateCell, updateCell_fast_free
   */
  inline void updateCell_fast_occupied(
      const Bonxai::CoordT& coord, const occupancy_t logodd_obs, const occupancy_t thres)
  {
    if (voxel_node_t* cell = base_t::m_impl->accessor.value(coord, true /*create*/); cell)
      updateCell_fast_occupied(cell, logodd_obs, thres);
  }

  /** Performs Bayesian fusion of a new observation of a cell.
   * This method increases the "free-ness" of a cell, managing possible
   * saturation.
   *  \param logodd_obs Observation of the cell, in log-odd form as
   * transformed by p2l.
   *  \param thres  This must be CELLTYPE_MAX-logodd_obs
   * \sa updateCell_fast_occupied
   */
  inline void updateCell_fast_free(
      voxel_node_t* theCell, const occupancy_t logodd_obs, const occupancy_t thres)
  {
    if (theCell == nullptr) return;
    occupancy_t& occ = theCell->occupancyRef();
    if (occ < thres)
      occ += logodd_obs;
    else
      occ = traits_t::CELLTYPE_MAX;
  }

  /** Performs the Bayesian fusion of a new observation of a cell.
   * This method increases the "free-ness" of a cell, managing possible
   * saturation.
   *  \param coord Cell indexes.
   *  \param logodd_obs Observation of the cell, in log-odd form as
   * transformed by p2l.
   *  \param thres  This must be CELLTYPE_MAX-logodd_obs
   * \sa updateCell_fast_occupied
   */
  inline void updateCell_fast_free(
      const Bonxai::CoordT& coord, const occupancy_t logodd_obs, const occupancy_t thres)
  {
    if (voxel_node_t* cell = base_t::m_impl->accessor.value(coord, true /*create*/); cell)
      updateCell_fast_free(cell, logodd_obs, thres);
  }

  /** Lookup tables for log-odds */
  static CLogOddsGridMapLUT<occupancy_value_t>& get_logodd_lut()
  {
    // Static lookup tables for log-odds
    static CLogOddsGridMapLUT<occupancy_value_t> logodd_lut;
    return logodd_lut;
  }

  /** Scales an integer representation of the log-odd into a real valued
   * probability in [0,1], using p=exp(l)/(1+exp(l))  */
  static inline float l2p(const occupancy_value_t l) { return get_logodd_lut().l2p(l); }

  /** Scales an integer representation of the log-odd into a linear scale
   * [0,255], using p=exp(l)/(1+exp(l)) */
  static inline uint8_t l2p_255(const occupancy_value_t l) { return get_logodd_lut().l2p_255(l); }
  /** Scales a real valued probability in [0,1] to an integer representation
   * of: log(p)-log(1-p)  in the valid range of voxel_node_t */
  static inline occupancy_value_t p2l(const float p) { return get_logodd_lut().p2l(p); }

  /** @name API of the NearestNeighborsCapable virtual interface
    @{ */
  // See docs in base class
  void nn_prepare_for_2d_queries() const override
  {
    getOccupiedVoxels()->nn_prepare_for_2d_queries();
  }
  void nn_prepare_for_3d_queries() const override
  {
    getOccupiedVoxels()->nn_prepare_for_3d_queries();
  }
  [[nodiscard]] bool nn_has_indices_or_ids() const override
  {
    return getOccupiedVoxels()->nn_has_indices_or_ids();
  }
  [[nodiscard]] size_t nn_index_count() const override
  {
    return getOccupiedVoxels()->nn_index_count();
  }
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint3Df& query,
      mrpt::math::TPoint3Df& result,
      float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override
  {
    return getOccupiedVoxels()->nn_single_search(query, result, out_dist_sqr, resultIndexOrID);
  }
  [[nodiscard]] bool nn_single_search(
      const mrpt::math::TPoint2Df& query,
      mrpt::math::TPoint2Df& result,
      float& out_dist_sqr,
      uint64_t& resultIndexOrID) const override
  {
    return getOccupiedVoxels()->nn_single_search(query, result, out_dist_sqr, resultIndexOrID);
  }
  void nn_multiple_search(
      const mrpt::math::TPoint3Df& query,
      const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override
  {
    getOccupiedVoxels()->nn_multiple_search(query, N, results, out_dists_sqr, resultIndicesOrIDs);
  }
  void nn_multiple_search(
      const mrpt::math::TPoint2Df& query,
      const size_t N,
      std::vector<mrpt::math::TPoint2Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const override
  {
    getOccupiedVoxels()->nn_multiple_search(query, N, results, out_dists_sqr, resultIndicesOrIDs);
  }
  void nn_radius_search(
      const mrpt::math::TPoint3Df& query,
      const float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs,
      size_t maxPoints) const override
  {
    getOccupiedVoxels()->nn_radius_search(
        query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
  }
  void nn_radius_search(
      const mrpt::math::TPoint2Df& query,
      const float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs,
      size_t maxPoints) const override
  {
    getOccupiedVoxels()->nn_radius_search(
        query, search_radius_sqr, results, out_dists_sqr, resultIndicesOrIDs, maxPoints);
  }
  /** @} */

 protected:
  void internal_clear() override;

  void markAsChanged() { m_cachedOccupied.reset(); }

  void updateCachedProperties() const;
  mutable mrpt::maps::CSimplePointsMap::Ptr m_cachedOccupied;
  mutable mrpt::math::TBoundingBox m_bbox;
};

// ============= Implementations ===============
template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::getAsOctoMapVoxels(
    mrpt::opengl::COctoMapVoxels& gl_obj) const
{
  using mrpt::opengl::COctoMapVoxels;
  using mrpt::opengl::VOXEL_SET_FREESPACE;
  using mrpt::opengl::VOXEL_SET_OCCUPIED;

  const mrpt::img::TColorf general_color = gl_obj.getColor();
  const mrpt::img::TColor general_color_u = general_color.asTColor();

  gl_obj.clear();
  gl_obj.resizeVoxelSets(2);  // 2 sets of voxels: occupied & free

  gl_obj.showVoxels(mrpt::opengl::VOXEL_SET_OCCUPIED, renderingOptions.visibleOccupiedVoxels);
  gl_obj.showVoxels(mrpt::opengl::VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels);

  const size_t nLeafs = base_t::m_impl->grid.activeCellsCount();
  gl_obj.reserveVoxels(VOXEL_SET_OCCUPIED, nLeafs);

  // forEachCell() has no const version
  auto& grid = const_cast<Bonxai::VoxelGrid<voxel_node_t>&>(base_t::m_impl->grid);

  const mrpt::math::TBoundingBoxf bbox = this->boundingBox();
  double bbox_span_z = bbox.max.z - bbox.min.z;
  if (bbox_span_z < 0) bbox_span_z = 1;
  const double bbox_span_z_inv = 1.0 / bbox_span_z;

  // Go thru all voxels:
  auto lmbdPerVoxel = [this, &grid, &gl_obj, general_color_u, general_color, bbox, bbox_span_z_inv](
                          voxel_node_t& data, const Bonxai::CoordT& coord)
  {
    using mrpt::img::TColor;

    // log-odds to probability:
    const double occ = 1.0 - this->l2p(data.occupancyRef());
    const auto pt = Bonxai::CoordToPos(coord, grid.resolution);

    if ((occ >= renderingOptions.occupiedThreshold && renderingOptions.generateOccupiedVoxels) ||
        (occ < renderingOptions.freeThreshold && renderingOptions.generateFreeVoxels))
    {
      mrpt::img::TColor vx_color;
      double coefc, coeft;
      switch (gl_obj.getVisualizationMode())
      {
        case COctoMapVoxels::FIXED:
          vx_color = general_color_u;
          break;

        case COctoMapVoxels::COLOR_FROM_HEIGHT:
          vx_color = mrpt::img::colormap(gl_obj.colorMap(), (pt.z - bbox.min.z) * bbox_span_z_inv);
          break;

        case COctoMapVoxels::COLOR_FROM_OCCUPANCY:
          coefc = 240 * (1 - occ) + 15;
          vx_color = TColor(
              coefc * general_color.R, coefc * general_color.G, coefc * general_color.B,
              255.0 * general_color.A);
          break;

        case COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY:
          coeft = 255 - 510 * (1 - occ);
          if (coeft < 0)
          {
            coeft = 0;
          }
          vx_color =
              TColor(255 * general_color.R, 255 * general_color.G, 255 * general_color.B, coeft);
          break;

        case COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY:
          coefc = 240 * (1 - occ) + 15;
          vx_color =
              TColor(coefc * general_color.R, coefc * general_color.G, coefc * general_color.B, 50);
          break;

        case COctoMapVoxels::MIXED:
          THROW_EXCEPTION("MIXED not supported yet for this class");
          break;

        case COctoMapVoxels::COLOR_FROM_RGB_DATA:
          if constexpr (internal::has_color<voxel_node_t>::value)
          {
            vx_color.R = data.color.R;
            vx_color.G = data.color.G;
            vx_color.B = data.color.B;
          }
          else
          {
            THROW_EXCEPTION(
                "COLOR_FROM_RGB_DATA used with unsupported voxel "
                "data type");
          }
          break;

        default:
          THROW_EXCEPTION("Unknown coloring scheme!");
      }

      const size_t vx_set =
          (occ > renderingOptions.occupiedThreshold) ? VOXEL_SET_OCCUPIED : VOXEL_SET_FREESPACE;

      gl_obj.push_back_Voxel(
          vx_set, COctoMapVoxels::TVoxel(
                      mrpt::math::TPoint3Df(pt.x, pt.y, pt.z), grid.resolution, vx_color));
    }
  };  // end lambda for each voxel

  grid.forEachCell(lmbdPerVoxel);

  // if we use transparency, sort cubes by "Z" as an approximation to
  // far-to-near render ordering:
  if (gl_obj.isCubeTransparencyEnabled()) gl_obj.sort_voxels_by_z();

  // Set bounding box:
  gl_obj.setBoundingBox(bbox.min, bbox.max);
}

template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::internal_clear()
{
  // Is this enough?
  base_t::m_impl->grid.root_map.clear();

  markAsChanged();
}

template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::updateVoxel(
    const double x, const double y, const double z, bool occupied)
{
  markAsChanged();

  voxel_node_t* cell = base_t::m_impl->accessor.value(
      Bonxai::PosToCoord({x, y, z}, base_t::m_impl->grid.inv_resolution), true /*create*/);
  if (!cell) return;  // should never happen?

  if (occupied)
  {
    const occupancy_t logodd_observation_occupied =
        std::max<occupancy_t>(1, p2l(insertionOptions.prob_hit));
    const occupancy_t logodd_thres_occupied = p2l(1.0 - insertionOptions.clamp_max);

    updateCell_fast_occupied(cell, logodd_observation_occupied, logodd_thres_occupied);
  }
  else
  {
    const occupancy_t logodd_observation_free =
        std::max<occupancy_t>(1, p2l(insertionOptions.prob_miss));
    const occupancy_t logodd_thres_free = p2l(1.0 - insertionOptions.clamp_min);

    updateCell_fast_free(cell, logodd_observation_free, logodd_thres_free);
  }
}
template <typename voxel_node_t, typename occupancy_t>
bool CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::getPointOccupancy(
    const double x, const double y, const double z, double& prob_occupancy) const
{
  voxel_node_t* cell = base_t::m_impl->accessor.value(
      Bonxai::PosToCoord({x, y, z}, base_t::m_impl->grid.inv_resolution), false /*create*/);

  if (!cell) return false;

  prob_occupancy = 1.0 - l2p(cell->occupancyRef());
  return true;
}

template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::insertPointCloudAsRays(
    const mrpt::maps::CPointsMap& pts,
    const mrpt::math::TPoint3D& sensorPt,
    const std::optional<const mrpt::poses::CPose3D>& sensorPose)
{
  markAsChanged();

  const occupancy_t logodd_observation_occupied =
      std::max<occupancy_t>(1, p2l(insertionOptions.prob_hit));
  const occupancy_t logodd_thres_occupied = p2l(1.0 - insertionOptions.clamp_max);

  const auto& xs = pts.getPointsBufferRef_x();
  const auto& ys = pts.getPointsBufferRef_y();
  const auto& zs = pts.getPointsBufferRef_z();

  const auto maxSqrDist = mrpt::square(insertionOptions.max_range);

  // Starting cell index at sensor pose:
  Bonxai::CoordT sensorCoord =
      Bonxai::PosToCoord({sensorPt.x, sensorPt.y, sensorPt.z}, base_t::m_impl->grid.inv_resolution);

  // Use fixed comma for the ray tracing direction:
  constexpr unsigned int FRBITS = 9;

  const occupancy_t logodd_observation_free =
      std::max<occupancy_t>(1, p2l(insertionOptions.prob_miss));
  const occupancy_t logodd_thres_free = p2l(1.0 - insertionOptions.clamp_min);

  // for each ray:
  for (size_t i = 0; i < xs.size(); i += insertionOptions.decimation)
  {
    const auto pt = sensorPose ? sensorPose->composePoint(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]))
                               : mrpt::math::TPoint3D(xs[i], ys[i], zs[i]);

    if (insertionOptions.max_range > 0 && (pt - sensorPt).sqrNorm() > maxSqrDist) continue;  // skip

    const Bonxai::CoordT endCoord =
        Bonxai::PosToCoord({pt.x, pt.y, pt.z}, base_t::m_impl->grid.inv_resolution);

    // jump in discrete steps from sensorCoord to endCoord:
    // Use "fractional integers" to approximate float operations
    //  during the ray tracing:
    const Bonxai::CoordT Ac = endCoord - sensorCoord;

    uint32_t Acx_ = std::abs(Ac.x);
    uint32_t Acy_ = std::abs(Ac.y);
    uint32_t Acz_ = std::abs(Ac.z);

    const auto nStepsRay = std::max(Acx_, std::max(Acy_, Acz_));
    if (!nStepsRay) continue;  // May be...

    // Integers store "float values * 128"
    float N_1 = 1.0f / nStepsRay;  // Avoid division twice.

    // Increments at each raytracing step:
    int frAcx = (Ac.x < 0 ? -1 : +1) * round((Acx_ << FRBITS) * N_1);
    int frAcy = (Ac.y < 0 ? -1 : +1) * round((Acy_ << FRBITS) * N_1);
    int frAcz = (Ac.z < 0 ? -1 : +1) * round((Acz_ << FRBITS) * N_1);

    int frCX = sensorCoord.x << FRBITS;
    int frCY = sensorCoord.y << FRBITS;
    int frCZ = sensorCoord.z << FRBITS;

    // free space ray:
    for (unsigned int nStep = 0; nStep < nStepsRay; nStep++)
    {
      if (voxel_node_t* cell = base_t::m_impl->accessor.value(
              {frCX >> FRBITS, frCY >> FRBITS, frCZ >> FRBITS}, true /*create*/);
          cell)
      {
        updateCell_fast_free(cell, logodd_observation_free, logodd_thres_free);
      }

      frCX += frAcx;
      frCY += frAcy;
      frCZ += frAcz;
    }

    // and occupied end point:
    if (voxel_node_t* cell = base_t::m_impl->accessor.value(endCoord, true /*create*/); cell)
    {
      updateCell_fast_occupied(cell, logodd_observation_occupied, logodd_thres_occupied);
    }
  }  // for each point/ray
}

template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::insertPointCloudAsEndPoints(
    const mrpt::maps::CPointsMap& pts,
    const mrpt::math::TPoint3D& sensorPt,
    const std::optional<const mrpt::poses::CPose3D>& sensorPose)
{
  markAsChanged();

  const occupancy_t logodd_observation_occupied =
      std::max<occupancy_t>(1, p2l(insertionOptions.prob_hit));
  const occupancy_t logodd_thres_occupied = p2l(1.0 - insertionOptions.clamp_max);

  const auto& xs = pts.getPointsBufferRef_x();
  const auto& ys = pts.getPointsBufferRef_y();
  const auto& zs = pts.getPointsBufferRef_z();

  const auto maxSqrDist = mrpt::square(insertionOptions.max_range);

  for (size_t i = 0; i < xs.size(); i += insertionOptions.decimation)
  {
    const auto pt = sensorPose ? sensorPose->composePoint(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]))
                               : mrpt::math::TPoint3D(xs[i], ys[i], zs[i]);

    if (insertionOptions.max_range > 0 && (pt - sensorPt).sqrNorm() > maxSqrDist) continue;  // skip

    voxel_node_t* cell = base_t::m_impl->accessor.value(
        Bonxai::PosToCoord({pt.x, pt.y, pt.z}, base_t::m_impl->grid.inv_resolution),
        true /*create*/);
    if (!cell) continue;  // should never happen?

    updateCell_fast_occupied(cell, logodd_observation_occupied, logodd_thres_occupied);
  }
}

template <typename voxel_node_t, typename occupancy_t>
void CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::updateCachedProperties() const
{
  if (m_cachedOccupied) return;  // done

  m_cachedOccupied = mrpt::maps::CSimplePointsMap::Create();
  m_bbox = mrpt::math::TBoundingBox::PlusMinusInfinity();

  // forEachCell() has no const version
  auto& grid = const_cast<Bonxai::VoxelGrid<voxel_node_t>&>(base_t::m_impl->grid);

  const double freenessThreshold = 1.0 - likelihoodOptions.occupiedThreshold;

  // Go thru all voxels:
  auto lmbdPerVoxel =
      [this, freenessThreshold, &grid](voxel_node_t& data, const Bonxai::CoordT& coord)
  {
    using mrpt::img::TColor;

    // log-odds to probability:
    const double occFreeness = this->l2p(data.occupancyRef());
    const auto pt = Bonxai::CoordToPos(coord, grid.resolution);

    m_bbox.updateWithPoint({pt.x, pt.y, pt.z});

    if (occFreeness < freenessThreshold)
    {
      m_cachedOccupied->insertPointFast(pt.x, pt.y, pt.z);
    }
  };  // end lambda for each voxel

  grid.forEachCell(lmbdPerVoxel);

  // If no cell is active, use default bbox:
  if (m_bbox == mrpt::math::TBoundingBox::PlusMinusInfinity()) m_bbox = {};
}

template <typename voxel_node_t, typename occupancy_t>
mrpt::maps::CSimplePointsMap::Ptr
CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::getOccupiedVoxels() const
{
  updateCachedProperties();
  return m_cachedOccupied;
}

template <typename voxel_node_t, typename occupancy_t>
mrpt::math::TBoundingBoxf CVoxelMapOccupancyBase<voxel_node_t, occupancy_t>::boundingBox() const
{
  updateCachedProperties();
  return {m_bbox.min.cast<float>(), m_bbox.max.cast<float>()};
}

}  // namespace mrpt::maps
