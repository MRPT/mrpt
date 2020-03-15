/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CLogOddsGridMap3D.h>
#include <mrpt/maps/CLogOddsGridMapLUT.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/OccupancyGridCellType.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt::maps
{
/** A 3D occupancy grid map with a regular, even distribution of voxels.
 *
 * This is a faster alternative to COctoMap, but use with caution with limited
 *map extensions, since it could easily exaust available memory.
 *
 * Each voxel follows a Bernoulli probability distribution: a value of 0 means
 *certainly occupied, 1 means a certainly empty voxel. Initially 0.5 means
 *uncertainty.
 *
 * \ingroup mrpt_maps_grp
 **/
class COccupancyGridMap3D
	: public CMetricMap,
	  public CLogOddsGridMap3D<OccGridCellTraits::cellType>
{
	DEFINE_SERIALIZABLE(COccupancyGridMap3D, mrpt::maps)
   public:
	/** The type of the map voxels: */
	using voxelType = OccGridCellTraits::cellType;
	using voxelTypeUnsigned = OccGridCellTraits::cellTypeUnsigned;

   protected:
	/** Lookup tables for log-odds */
	static CLogOddsGridMapLUT<voxelType>& get_logodd_lut();

	/** True upon construction; used by isEmpty() */
	bool m_is_empty{true};

	/** See base class */
	void OnPostSuccesfulInsertObs(const mrpt::obs::CObservation&) override;

	/** Clear the map: It set all voxels to their default occupancy value (0.5),
	 * without changing the resolution (the grid extension is reset to the
	 * default values). */
	void internal_clear() override;

	/** Insert the observation information into this map.
	 *
	 * \param obs The observation
	 * \param robotPose The 3D pose of the robot mobile base in the map
	 * reference system, or nullptr (default) if you want to use
	 * CPose2D(0,0,deg)
	 *
	 * \sa insertionOptions, CObservation::insertObservationInto
	 */
	bool internal_insertObservation(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

	void internal_insertObservationScan2D(
		const mrpt::obs::CObservation2DRangeScan& o,
		const mrpt::poses::CPose3D& robotPose);

	void internal_insertObservationScan3D(
		const mrpt::obs::CObservation3DRangeScan& o,
		const mrpt::poses::CPose3D& robotPose);

   public:
	/** Constructor */
	COccupancyGridMap3D(
		const mrpt::math::TPoint3D& corner_min = {-5.0f, -5.0f, -5.0f},
		const mrpt::math::TPoint3D& corner_max = {5.0f, 5.0f, 5.0f},
		float resolution = 0.25f);

	/** Fills all the voxels with a default value. */
	void fill(float default_value = 0.5f);

	/** Change the size of gridmap, erasing all its previous contents.
	 * \param resolution The new size of voxels.
	 * \param default_value The value of voxels, tipically 0.5.
	 * \sa ResizeGrid
	 */
	void setSize(
		const mrpt::math::TPoint3D& corner_min,
		const mrpt::math::TPoint3D& corner_max, double resolution,
		float default_value = 0.5f);

	/** Change the size of gridmap, maintaining previous contents.
	 * \param new_voxels_default_value Value of new voxels, tipically 0.5
	 * \sa setSize()
	 */
	void resizeGrid(
		const mrpt::math::TPoint3D& corner_min,
		const mrpt::math::TPoint3D& corner_max,
		float new_voxels_default_value = 0.5f);

	/** Scales an integer representation of the log-odd into a real valued
	 * probability in [0,1], using p=exp(l)/(1+exp(l))  */
	static inline float l2p(const voxelType l)
	{
		return get_logodd_lut().l2p(l);
	}
	/** Scales an integer representation of the log-odd into a linear scale
	 * [0,255], using p=exp(l)/(1+exp(l)) */
	static inline uint8_t l2p_255(const voxelType l)
	{
		return get_logodd_lut().l2p_255(l);
	}
	/** Scales a real valued probability in [0,1] to an integer representation
	 * of: log(p)-log(1-p)  in the valid range of voxelType */
	static inline voxelType p2l(const float p)
	{
		return get_logodd_lut().p2l(p);
	}

	/** Performs the Bayesian fusion of a new observation of a cell  \sa
	 * updateInfoChangeOnly, updateCell_fast_occupied, updateCell_fast_free */
	void updateCell(int cx_idx, int cy_idx, int cz_idx, float v);

	/** Change the contents [0,1] (0:occupied, 1:free) of a voxel, given its
	 * index. */
	inline void setCellFreeness(
		unsigned int cx, unsigned int cy, unsigned int cz, float value)
	{
		if (auto* c = m_grid.cellByIndex(cx, cy, cz); c != nullptr)
			*c = p2l(value);
	}

	/** Read the real valued [0,1] (0:occupied, 1:free) contents of a voxel,
	 * given its index */
	inline float getCellFreeness(
		unsigned int cx, unsigned int cy, unsigned int cz) const
	{
		if (auto* c = m_grid.cellByIndex(cx, cy, cz); c != nullptr)
			return l2p(*c);
		else
			return .5f;
	}

	/** Change the contents [0,1] of a voxel, given its coordinates */
	inline void setFreenessByPos(float x, float y, float z, float value)
	{
		setCellFreeness(
			m_grid.x2idx(x), m_grid.y2idx(y), m_grid.z2idx(z), value);
	}

	/** Read the real valued [0,1] contents of a voxel, given its coordinates */
	inline float getFreenessByPos(float x, float y, float z) const
	{
		return getCellFreeness(
			m_grid.x2idx(x), m_grid.y2idx(y), m_grid.z2idx(z));
	}

	/** Increases the freeness of a ray segment, and the occupancy of the voxel
	 * at its end point (unless endIsOccupied=false).
	 * Normally, users would prefer the higher-level method
	 * CMetricMap::insertObservation()
	 */
	void insertRay(
		const mrpt::math::TPoint3D& sensor, const mrpt::math::TPoint3D& end,
		bool endIsOccupied = true);

	/** Calls insertRay() for each point in the point cloud, using as sensor
	 * central point (the origin of all rays), the given `sensorCenter`.
	 * \param[in] maxValidRange If a point has larger distance from
	 * `sensorCenter` than `maxValidRange`, it will be considered a non-echo,
	 * and NO occupied voxel will be created at the end of the segment.
	 * \sa insertionOptions parameters are observed in this method.
	 */
	void insertPointCloud(
		const mrpt::math::TPoint3D& sensorCenter,
		const mrpt::maps::CPointsMap& pts,
		const float maxValidRange = std::numeric_limits<float>::max());

	/** \sa renderingOptions */
	void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels& gl_obj) const;

	/** Returns a 3D object representing the map. \sa renderingOptions */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** With this struct options are provided to the observation insertion
	 * process.
	 * \sa CObservation::insertIntoGridMap */
	class TInsertionOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		TInsertionOptions() = default;

		/** This method load the options from a ".ini" file.
		 *   Only those parameters found in the given "section" and having
		 *   the same name that the variable are loaded. Those not found in
		 *   the file will stay with their previous values (usually the default
		 *   values loaded at initialization). An example of an ".ini" file:
		 *  \code
		 *  [section]
		 *	resolution=0.10		; blah blah...
		 *	modeSelection=1		; 0=blah, 1=blah,...
		 *  \endcode
		 */
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		// See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& target,
			const std::string& section) const override;

		/** Largest distance at which voxels are updated (Default: 15 meters) */
		float maxDistanceInsertion{15.0f};
		/** A value in the range [0.5,1] used for updating voxel with a Bayesian
		 * approach (default 0.65) */
		float maxOccupancyUpdateCertainty{0.65f};
		/** A value in the range [0.5,1] for updating a free voxel. (default=0
		 * means use the same than maxOccupancyUpdateCertainty) */
		float maxFreenessUpdateCertainty{.0f};
		/** Specify the decimation of 3D range scans.
		 * "N" means keeping the minimum range of each block of "NxN" range
		 * pixels.
		 * (default=8)
		 */
		uint16_t decimation_3d_range{8};

		/** Decimation for insertPointCloud() or 2D range scans (Default: 1) */
		uint16_t decimation{1};
	};

	/** With this struct options are provided to the observation insertion
	 * process \sa CObservation::insertIntoGridMap */
	TInsertionOptions insertionOptions;

	/** The type for selecting a likelihood computation method */
	enum TLikelihoodMethod
	{
		lmLikelihoodField_Thrun = 0,
		lmRayTracing
		// Remember: Update TEnumType below if new values are added here!
	};

	/** With this struct options are provided to the observation likelihood
	 * computation process */
	class TLikelihoodOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		TLikelihoodOptions() = default;

		/** This method load the options from a ".ini" file.
		 *   Only those parameters found in the given "section" and having
		 *   the same name that the variable are loaded. Those not found in
		 *   the file will stay with their previous values (usually the default
		 *   values loaded at initialization). An example of an ".ini" file:
		 *  \code
		 *  [section]
		 *	resolution=0.10		; blah blah...
		 *	modeSelection=1		; 0=blah, 1=blah,...
		 *  \endcode
		 */
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		// See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& s) const override;

		/** The selected method to compute an observation likelihood */
		TLikelihoodMethod likelihoodMethod{lmLikelihoodField_Thrun};
		/** [LikelihoodField] The laser range "sigma" used in computations;
		 * Default value = 0.35 */
		float LF_stdHit{0.35f};
		/** [LikelihoodField] */
		float LF_zHit{0.95f}, LF_zRandom{0.05f};
		/** [LikelihoodField] The max. range of the sensor (Default= 81 m) */
		float LF_maxRange{20.0f};
		/** [LikelihoodField] The decimation of the points in a scan, default=1
		 * == no decimation */
		uint32_t LF_decimation{1};
		/** [LikelihoodField] The max. distance for searching correspondences
		 * around each sensed point */
		float LF_maxCorrsDistance{0.3f};
		/** [LikelihoodField] (Default:false) Use `exp(dist^2/std^2)` instead of
		 * `exp(dist^2/std^2)` */
		bool LF_useSquareDist{false};
		/** [rayTracing] One out of "rayTracing_decimation" rays will be
		 * simulated and compared only: set to 1 to use all the sensed ranges.
		 */
		int32_t rayTracing_decimation{10};
		/** [rayTracing] The laser range sigma. */
		float rayTracing_stdHit{1.0f};
	};

	TLikelihoodOptions likelihoodOptions;

	/** Options for the conversion of a mrpt::maps::COccupancyGridMap3D into a
	 * mrpt::opengl::COctoMapVoxels */
	struct TRenderingOptions
	{
		TRenderingOptions() = default;

		/** Generate grid lines for all octree nodes, useful to draw the
		 "structure" of the
		 octree, but computationally costly (Default: false) */
		bool generateGridLines{false};

		/** Generate voxels for the occupied volumes  (Default=true) */
		bool generateOccupiedVoxels{true};

		/** Set occupied voxels visible (requires generateOccupiedVoxels=true)
		 * (Default=true)*/
		bool visibleOccupiedVoxels{true};

		/** Generate voxels for the freespace (Default=true) */
		bool generateFreeVoxels{true};

		/** Set free voxels visible (requires generateFreeVoxels=true)
		 * (Default=true) */
		bool visibleFreeVoxels{true};

		/** Binary dump to stream */
		void writeToStream(mrpt::serialization::CArchive& out) const;
		/** Binary dump to stream */
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	TRenderingOptions renderingOptions;

	/** Simulate just one "ray" in the grid map. This method is used internally
	 * to sonarSimulator and laserScanSimulator. \sa
	 * COccupancyGridMap3D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS */
	void simulateScanRay(
		const double x, const double y, const double angle_direction,
		float& out_range, bool& out_valid, const double max_range_meters,
		const float threshold_free = 0.4f, const double noiseStd = .0,
		const double angleNoiseStd = .0) const;

	/** Computes the likelihood [0,1] of a set of points, given the current grid
	 * map as reference.
	 * \param pm The points map
	 * \param relativePose The relative pose of the points map in this map's
	 * coordinates, or nullptr for (0,0,0).
	 *  See "likelihoodOptions" for configuration parameters.
	 */
	double computeLikelihoodField_Thrun(
		const CPointsMap& pm,
		const mrpt::poses::CPose3D& relativePose = mrpt::poses::CPose3D());

	/** Returns true upon map construction or after calling clear(), the return
	 *  changes to false upon successful insertObservation() or any other
	 * method to load data in the map.
	 */
	bool isEmpty() const override;

	/** See docs in base class: in this class this always returns 0 */
	void determineMatching2D(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose2D& otherMapPose,
		mrpt::tfest::TMatchingPairList& correspondences,
		const TMatchingParams& params,
		TMatchingExtraResults& extraResults) const override;

	/** See docs in base class: in this class this always returns 0 */
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;

	void saveMetricMapRepresentationToFile(const std::string& f) const override;

   private:
	// See docs in base class
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation& obs,
		const mrpt::poses::CPose3D& takenFrom) override;
	// See docs in base class
	bool internal_canComputeObservationLikelihood(
		const mrpt::obs::CObservation& obs) const override;

	MAP_DEFINITION_START(COccupancyGridMap3D)
	/** See COccupancyGridMap3D::COccupancyGridMap3D */
	float min_x{-5.0f}, max_x{5.0f};
	float min_y{-5.0f}, max_y{5.0f};
	float min_z{-5.0f}, max_z{5.0f};
	float resolution{0.25f};

	/** Observations insertion options */
	mrpt::maps::COccupancyGridMap3D::TInsertionOptions insertionOpts;
	/** Probabilistic observation likelihood options */
	mrpt::maps::COccupancyGridMap3D::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(COccupancyGridMap3D)
};

}  // namespace mrpt::maps

MRPT_ENUM_TYPE_BEGIN(mrpt::maps::COccupancyGridMap3D::TLikelihoodMethod)
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap3D, lmRayTracing);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap3D, lmLikelihoodField_Thrun);
MRPT_ENUM_TYPE_END()
