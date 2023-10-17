/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CLogOddsGridMapLUT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMapBase.h>
#include <mrpt/maps/OccupancyGridCellType.h>
#include <mrpt/maps/logoddscell_traits.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt::maps
{
/**
 * Log-odds occupancy sparse voxel map.
 *
 * \ingroup mrpt_maps_grp
 */
class CVoxelMap : public CVoxelMapBase<int8_t>,
				  public detail::logoddscell_traits<int8_t>
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE(CVoxelMap, mrpt::maps)

   protected:
	using traits_t = detail::logoddscell_traits<voxel_node_t>;

   public:
	CVoxelMap(
		double resolution = 0.05, uint8_t inner_bits = 2, uint8_t leaf_bits = 3)
		: CVoxelMapBase(resolution, inner_bits, leaf_bits)
	{
	}
	~CVoxelMap();

	bool isEmpty() const override;
	void getAsOctoMapVoxels(
		mrpt::opengl::COctoMapVoxels& gl_obj) const override;

	/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied
	 * (true) or free (false), using the log-odds parameters in \a
	 * insertionOptions */
	void updateVoxel(
		const double x, const double y, const double z, bool occupied);

	/** Get the occupancy probability [0,1] of a point
	 * \return false if the point is not mapped, in which case the returned
	 * "prob" is undefined. */
	bool getPointOccupancy(
		const double x, const double y, const double z,
		double& prob_occupancy) const;

	void insertPointCloudAsRays(
		const mrpt::maps::CPointsMap& pts,
		const mrpt::math::TPoint3D& sensorPt);

	void insertPointCloudAsEndPoints(const mrpt::maps::CPointsMap& pts);

	/** Returns all occupied voxels as a point cloud. The shared_ptr is
	 *  also hold and updated internally, so it is not safe to read it
	 *  while also updating the voxel map in another thread.
	 *
	 *  The point cloud is cached, and invalidated upon map updates.
	 */
	mrpt::maps::CSimplePointsMap::Ptr getOccupiedVoxels() const;

	struct TInsertionOptions : public mrpt::config::CLoadableOptions
	{
		TInsertionOptions() = default;

		double max_range = -1;	//!< Maximum insertion ray range (<0: none)

		double prob_miss = 0.45;
		double prob_hit = 0.65;
		double clamp_min = 0.10;
		double clamp_max = 0.95;

		bool ray_trace_free_space = true;
		uint32_t decimation = 1;

		// See base docs
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c, const std::string& s) const;

		void writeToStream(mrpt::serialization::CArchive& out) const;
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	/// The options used when inserting observations in the map:
	TInsertionOptions insertionOptions;

	/** Options used when evaluating "computeObservationLikelihood"
	 * \sa CObservation::computeObservationLikelihood
	 */
	struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
	{
		TLikelihoodOptions() = default;
		~TLikelihoodOptions() override = default;

		// See base docs
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c, const std::string& s) const;

		void writeToStream(mrpt::serialization::CArchive& out) const;
		void readFromStream(mrpt::serialization::CArchive& in);

		/// Speed up the likelihood computation by considering only one out of N
		/// rays (default=1)
		uint32_t decimation = 1;
	};

	TLikelihoodOptions likelihoodOptions;

	/** Options for the conversion of a mrpt::maps::COctoMap into a
	 * mrpt::opengl::COctoMapVoxels */
	struct TRenderingOptions
	{
		TRenderingOptions() = default;

		bool generateOccupiedVoxels = true;
		bool visibleOccupiedVoxels = true;

		bool generateFreeVoxels = true;
		bool visibleFreeVoxels = true;

		/** Binary dump to stream */
		void writeToStream(mrpt::serialization::CArchive& out) const;
		/** Binary dump to stream */
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	TRenderingOptions renderingOptions;

	MAP_DEFINITION_START(CVoxelMap)
	double resolution = 0.10;
	uint8_t inner_bits = 2;
	uint8_t leaf_bits = 3;
	mrpt::maps::CVoxelMap::TInsertionOptions insertionOpts;
	mrpt::maps::CVoxelMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CVoxelMap)

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
		voxel_node_t* theCell, const voxel_node_t logodd_obs,
		const voxel_node_t thres)
	{
		if (theCell == nullptr) return;
		if (*theCell > thres) *theCell -= logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MIN;
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
		const Bonxai::CoordT& coord, const voxel_node_t logodd_obs,
		const voxel_node_t thres)
	{
		if (voxel_node_t* cell = m_impl->accessor.value(coord, true /*create*/);
			cell)
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
		voxel_node_t* theCell, const voxel_node_t logodd_obs,
		const voxel_node_t thres)
	{
		if (theCell == nullptr) return;
		if (*theCell < thres) *theCell += logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MAX;
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
		const Bonxai::CoordT& coord, const voxel_node_t logodd_obs,
		const voxel_node_t thres)
	{
		if (voxel_node_t* cell = m_impl->accessor.value(coord, true /*create*/);
			cell)
			updateCell_fast_free(cell, logodd_obs, thres);
	}

	/** Lookup tables for log-odds */
	static CLogOddsGridMapLUT<voxel_node_t>& get_logodd_lut();

	/** Scales an integer representation of the log-odd into a real valued
	 * probability in [0,1], using p=exp(l)/(1+exp(l))  */
	static inline float l2p(const voxel_node_t l)
	{
		return get_logodd_lut().l2p(l);
	}

	/** Scales an integer representation of the log-odd into a linear scale
	 * [0,255], using p=exp(l)/(1+exp(l)) */
	static inline uint8_t l2p_255(const voxel_node_t l)
	{
		return get_logodd_lut().l2p_255(l);
	}
	/** Scales a real valued probability in [0,1] to an integer representation
	 * of: log(p)-log(1-p)  in the valid range of voxel_node_t */
	static inline voxel_node_t p2l(const float p)
	{
		return get_logodd_lut().p2l(p);
	}

   protected:
	void internal_clear() override;
	bool internal_insertObservation(
		const mrpt::obs::CObservation& obs,
		const std::optional<const mrpt::poses::CPose3D>& robotPose =
			std::nullopt) override;
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D& takenFrom) const override;

	void invalidateOccupiedCache() { m_cachedOccupied.reset(); }

	void updateOccupiedPointsCache() const;
	mutable mrpt::maps::CSimplePointsMap::Ptr m_cachedOccupied;
};

}  // namespace mrpt::maps