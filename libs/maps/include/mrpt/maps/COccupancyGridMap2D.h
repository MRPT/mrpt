/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/img/CImage.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/maps/CLogOddsGridMap2D.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/obs/CObservation2DRangeScanWithUncertainty.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/typemeta/TEnumType.h>

#include <mrpt/config.h>
#if (                                                \
	!defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) &&   \
	!defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)) || \
	(defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) &&   \
	 defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS))
#error One of OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS or OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS must be defined.
#endif

namespace mrpt::maps
{
/** A class for storing an occupancy grid map.
 *  COccupancyGridMap2D is a class for storing a metric map
 *   representation in the form of a probabilistic occupancy
 *   grid map: value of 0 means certainly occupied, 1 means
 *   a certainly empty cell. Initially 0.5 means uncertainty.
 *
 * The cells keep the log-odd representation of probabilities instead of the
 *probabilities themselves.
 *  More details can be found at http://www.mrpt.org/Occupancy_Grids
 *
 * The algorithm for updating the grid from a laser scanner can optionally take
 *into account the progressive widening of the beams, as
 *   described in [this page](http://www.mrpt.org/Occupancy_Grids)
 *
 *   Some implemented methods are:
 *		- Update of individual cells
 *		- Insertion of observations
 *		- Voronoi diagram and critical points (\a buildVoronoiDiagram)
 *		- Saving and loading from/to a bitmap
 *		- Laser scans simulation for the map contents
 *		- Entropy and information methods (See computeEntropy)
 *
 * \ingroup mrpt_maps_grp
 **/
class COccupancyGridMap2D : public CMetricMap,
// Inherit from the corresponding specialization of CLogOddsGridMap2D<>:
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
							public CLogOddsGridMap2D<int8_t>
#else
							public CLogOddsGridMap2D<int16_t>
#endif
{
	DEFINE_SERIALIZABLE(COccupancyGridMap2D)
   public:
/** The type of the map cells: */
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	using cellType = int8_t;
	using cellTypeUnsigned = uint8_t;
#else
	using cellType = int16_t;
	using cellTypeUnsigned = uint16_t;
#endif

	/** Discrete to float conversion factors: The min/max values of the integer
	 * cell type, eg.[0,255] or [0,65535] */
	static const cellType OCCGRID_CELLTYPE_MIN =
		CLogOddsGridMap2D<cellType>::CELLTYPE_MIN;
	static const cellType OCCGRID_CELLTYPE_MAX =
		CLogOddsGridMap2D<cellType>::CELLTYPE_MAX;
	static const cellType OCCGRID_P2LTABLE_SIZE =
		CLogOddsGridMap2D<cellType>::P2LTABLE_SIZE;

	/** (Default:1.0) Can be set to <1 if a more fine raytracing is needed in
	 * sonarSimulator() and laserScanSimulator(), or >1 to speed it up. */
	static double RAYTRACE_STEP_SIZE_IN_CELL_UNITS;

   protected:
	friend class CMultiMetricMap;
	friend class CMultiMetricMapPDF;

	/** Frees the dynamic memory buffers of map. */
	void freeMap();
	/** Lookup tables for log-odds */
	static CLogOddsGridMapLUT<cellType>& get_logodd_lut();

	/** Store of cell occupancy values. Order: row by row, from left to right */
	std::vector<cellType> map;
	/** The size of the grid in cells */
	uint32_t size_x{0}, size_y{0};
	/** The limits of the grid in "units" (meters) */
	float x_min{}, x_max{}, y_min{}, y_max{};
	/** Cell size, i.e. resolution of the grid map. */
	float resolution{};

	/** Auxiliary variables to speed up the computation of observation
	 * likelihood values for LF method among others, at a high cost in memory
	 * (see TLikelihoodOptions::enableLikelihoodCache). */
	std::vector<double> precomputedLikelihood;
	bool precomputedLikelihoodToBeRecomputed{true};

	/** Used for Voronoi calculation.Same struct as "map", but contains a "0" if
	 * not a basis point. */
	mrpt::containers::CDynamicGrid<uint8_t> m_basis_map;

	/** Used to store the Voronoi diagram.
	 *    Contains the distance of each cell to its closer obstacles
	 *    in 1/100th distance units (i.e. in centimeters), or 0 if not into the
	 * Voronoi diagram  */
	mrpt::containers::CDynamicGrid<uint16_t> m_voronoi_diagram;

	/** True upon construction; used by isEmpty() */
	bool m_is_empty{true};

	/** See base class */
	void OnPostSuccesfulInsertObs(const mrpt::obs::CObservation*) override;

	/** The free-cells threshold used to compute the Voronoi diagram. */
	float voroni_free_threshold{};

	/** Entropy computation internal function: */
	static double H(double p);
	/** Internally used to speed-up entropy calculation */
	static std::vector<float> entropyTable;

	/** Change the contents [0,1] of a cell, given its index */
	inline void setCell_nocheck(int x, int y, float value)
	{
		map[x + y * size_x] = p2l(value);
	}

	/** Read the real valued [0,1] contents of a cell, given its index */
	inline float getCell_nocheck(int x, int y) const
	{
		return l2p(map[x + y * size_x]);
	}
	/** Changes a cell by its absolute index (Do not use it normally) */
	inline void setRawCell(unsigned int cellIndex, cellType b)
	{
		if (cellIndex < size_x * size_y) map[cellIndex] = b;
	}

	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood" (This method is the Range-Scan Likelihood
	 * Consensus for gridmaps, see the ICRA2007 paper by Blanco et al.)  */
	double computeObservationLikelihood_Consensus(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood". TODO: This method is described in....  */
	double computeObservationLikelihood_ConsensusOWA(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood"  */
	double computeObservationLikelihood_CellsDifference(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood" */
	double computeObservationLikelihood_MI(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood" */
	double computeObservationLikelihood_rayTracing(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood".*/
	double computeObservationLikelihood_likelihoodField_Thrun(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);
	/** One of the methods that can be selected for implementing
	 * "computeObservationLikelihood". */
	double computeObservationLikelihood_likelihoodField_II(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose2D& takenFrom);

	/** Clear the map: It set all cells to their default occupancy value (0.5),
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
	 *  After successfull execution, "lastObservationInsertionInfo" is updated.
	 *
	 * \sa insertionOptions, CObservation::insertObservationInto
	 */
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

   public:
	/** Read-only access to the raw cell contents (cells are in log-odd units)
	 */
	const std::vector<cellType>& getRawMap() const { return this->map; }
	/** Performs the Bayesian fusion of a new observation of a cell  \sa
	 * updateInfoChangeOnly, updateCell_fast_occupied, updateCell_fast_free */
	void updateCell(int x, int y, float v);

	/** An internal structure for storing data related to counting the new
	 * information apported by some observation */
	struct TUpdateCellsInfoChangeOnly
	{
		TUpdateCellsInfoChangeOnly(
			bool enabled_ = false, double I_change_ = 0, int cellsUpdated_ = 0)
			: enabled(enabled_),
			  I_change(I_change_),
			  cellsUpdated(cellsUpdated_)

		{
		}
		/** If set to false (default), this struct is not used. Set to true only
		 * when measuring the info of an observation. */
		bool enabled;
		/** The cummulative change in Information: This is updated only from the
		 * "updateCell" method. */
		double I_change;
		/** The cummulative updated cells count: This is updated only from the
		 * "updateCell" method. */
		int cellsUpdated;
		/** In this mode, some laser rays can be skips to speep-up */
		int laserRaysSkip{1};
	} updateInfoChangeOnly;

	/** Fills all the cells with a default value. */
	void fill(float default_value = 0.5f);

	/** Constructor */
	COccupancyGridMap2D(
		float min_x = -20.0f, float max_x = 20.0f, float min_y = -20.0f,
		float max_y = 20.0f, float resolution = 0.05f);
	/** Destructor */
	~COccupancyGridMap2D() override;

	/** Change the size of gridmap, erasing all its previous contents.
	 * \param x_min The "x" coordinates of left most side of grid.
	 * \param x_max The "x" coordinates of right most side of grid.
	 * \param y_min The "y" coordinates of top most side of grid.
	 * \param y_max The "y" coordinates of bottom most side of grid.
	 * \param resolution The new size of cells.
	 * \param default_value The value of cells, tipically 0.5.
	 * \sa ResizeGrid
	 */
	void setSize(
		float x_min, float x_max, float y_min, float y_max, float resolution,
		float default_value = 0.5f);

	/** Change the size of gridmap, maintaining previous contents.
	 * \param new_x_min The "x" coordinates of new left most side of grid.
	 * \param new_x_max The "x" coordinates of new right most side of grid.
	 * \param new_y_min The "y" coordinates of new top most side of grid.
	 * \param new_y_max The "y" coordinates of new bottom most side of grid.
	 * \param new_cells_default_value The value of the new cells, tipically 0.5.
	 * \param additionalMargin If set to true (default), an additional margin of
	 * a few meters will be added to the grid, ONLY if the new coordinates are
	 * larger than current ones.
	 * \sa setSize
	 */
	void resizeGrid(
		float new_x_min, float new_x_max, float new_y_min, float new_y_max,
		float new_cells_default_value = 0.5f,
		bool additionalMargin = true) noexcept;

	/** Returns the area of the gridmap, in square meters */
	inline double getArea() const
	{
		return size_x * size_y * mrpt::square(resolution);
	}

	/** Returns the horizontal size of grid map in cells count */
	inline unsigned int getSizeX() const { return size_x; }
	/** Returns the vertical size of grid map in cells count */
	inline unsigned int getSizeY() const { return size_y; }
	/** Returns the "x" coordinate of left side of grid map */
	inline float getXMin() const { return x_min; }
	/** Returns the "x" coordinate of right side of grid map */
	inline float getXMax() const { return x_max; }
	/** Returns the "y" coordinate of top side of grid map */
	inline float getYMin() const { return y_min; }
	/** Returns the "y" coordinate of bottom side of grid map */
	inline float getYMax() const { return y_max; }
	/** Returns the resolution of the grid map */
	inline float getResolution() const { return resolution; }
	/** Transform a coordinate value into a cell index */
	inline int x2idx(float x) const
	{
		return static_cast<int>((x - x_min) / resolution);
	}
	inline int y2idx(float y) const
	{
		return static_cast<int>((y - y_min) / resolution);
	}

	inline int x2idx(double x) const
	{
		return static_cast<int>((x - x_min) / resolution);
	}
	inline int y2idx(double y) const
	{
		return static_cast<int>((y - y_min) / resolution);
	}

	/** Transform a cell index into a coordinate value */
	inline float idx2x(const size_t cx) const
	{
		return x_min + (cx + 0.5f) * resolution;
	}
	inline float idx2y(const size_t cy) const
	{
		return y_min + (cy + 0.5f) * resolution;
	}

	/** Transform a coordinate value into a cell index, using a diferent "x_min"
	 * value */
	inline int x2idx(float x, float xmin) const
	{
		return static_cast<int>((x - xmin) / resolution);
	}
	inline int y2idx(float y, float ymin) const
	{
		return static_cast<int>((y - ymin) / resolution);
	}

	/** Scales an integer representation of the log-odd into a real valued
	 * probability in [0,1], using p=exp(l)/(1+exp(l))  */
	static inline float l2p(const cellType l)
	{
		return get_logodd_lut().l2p(l);
	}
	/** Scales an integer representation of the log-odd into a linear scale
	 * [0,255], using p=exp(l)/(1+exp(l)) */
	static inline uint8_t l2p_255(const cellType l)
	{
		return get_logodd_lut().l2p_255(l);
	}
	/** Scales a real valued probability in [0,1] to an integer representation
	 * of: log(p)-log(1-p)  in the valid range of cellType */
	static inline cellType p2l(const float p)
	{
		return get_logodd_lut().p2l(p);
	}
	/** Change the contents [0,1] of a cell, given its index */
	inline void setCell(int x, int y, float value)
	{
		// The x> comparison implicitly holds if x<0
		if (static_cast<unsigned int>(x) >= size_x ||
			static_cast<unsigned int>(y) >= size_y)
			return;
		else
			map[x + y * size_x] = p2l(value);
	}

	/** Read the real valued [0,1] contents of a cell, given its index */
	inline float getCell(int x, int y) const
	{
		// The x> comparison implicitly holds if x<0
		if (static_cast<unsigned int>(x) >= size_x ||
			static_cast<unsigned int>(y) >= size_y)
			return 0.5f;
		else
			return l2p(map[x + y * size_x]);
	}

	/** Access to a "row": mainly used for drawing grid as a bitmap efficiently,
	 * do not use it normally */
	inline cellType* getRow(int cy)
	{
		if (cy < 0 || static_cast<unsigned int>(cy) >= size_y)
			return nullptr;
		else
			return &map[0 + cy * size_x];
	}

	/** Access to a "row": mainly used for drawing grid as a bitmap efficiently,
	 * do not use it normally */
	inline const cellType* getRow(int cy) const
	{
		if (cy < 0 || static_cast<unsigned int>(cy) >= size_y)
			return nullptr;
		else
			return &map[0 + cy * size_x];
	}

	/** Change the contents [0,1] of a cell, given its coordinates */
	inline void setPos(float x, float y, float value)
	{
		setCell(x2idx(x), y2idx(y), value);
	}

	/** Read the real valued [0,1] contents of a cell, given its coordinates */
	inline float getPos(float x, float y) const
	{
		return getCell(x2idx(x), y2idx(y));
	}

	/** Returns "true" if cell is "static", i.e.if its occupancy is below a
	 * given threshold */
	inline bool isStaticPos(float x, float y, float threshold = 0.7f) const
	{
		return isStaticCell(x2idx(x), y2idx(y), threshold);
	}
	inline bool isStaticCell(int cx, int cy, float threshold = 0.7f) const
	{
		return (getCell(cx, cy) <= threshold);
	}

	/** Change a cell in the "basis" maps.Used for Voronoi calculation */
	inline void setBasisCell(int x, int y, uint8_t value)
	{
		uint8_t* cell = m_basis_map.cellByIndex(x, y);
#ifdef _DEBUG
		ASSERT_ABOVEEQ_(x, 0);
		ASSERT_ABOVEEQ_(y, 0);
		ASSERT_BELOWEQ_(x, int(m_basis_map.getSizeX()));
		ASSERT_BELOWEQ_(y, int(m_basis_map.getSizeY()));
#endif
		*cell = value;
	}

	/** Reads a cell in the "basis" maps.Used for Voronoi calculation */
	inline unsigned char getBasisCell(int x, int y) const
	{
		const uint8_t* cell = m_basis_map.cellByIndex(x, y);
#ifdef _DEBUG
		ASSERT_ABOVEEQ_(x, 0);
		ASSERT_ABOVEEQ_(y, 0);
		ASSERT_BELOWEQ_(x, int(m_basis_map.getSizeX()));
		ASSERT_BELOWEQ_(y, int(m_basis_map.getSizeY()));
#endif
		return *cell;
	}

	/** copy the gridmap contents, but not all the options, from another map
	 * instance */
	void copyMapContentFrom(const COccupancyGridMap2D& otherMap);

	/** Used for returning entropy related information \sa computeEntropy */
	struct TEntropyInfo
	{
		/** The target variable for absolute entropy, computed
		 * as:<br><center>H(map)=Sum<sub>x,y</sub>{ -p(x,y)*ln(p(x,y))
		 * -(1-p(x,y))*ln(1-p(x,y)) }</center><br><br> */
		double H{0};
		/** The target variable for absolute "information", defining I(x) = 1 -
		 * H(x) */
		double I{0};
		/** The target variable for mean entropy, defined as entropy per cell:
		 * mean_H(map) = H(map) / (cells) */
		double mean_H{0};
		/** The target variable for mean information, defined as information per
		 * cell: mean_I(map) = I(map) / (cells) */
		double mean_I{0};
		/** The target variable for the area of cells with information, i.e.
		 * p(x)!=0.5 */
		double effectiveMappedArea{0};
		/** The mapped area in cells. */
		unsigned long effectiveMappedCells{0};
	};

	/** With this struct options are provided to the observation insertion
	 * process.
	 * \sa CObservation::insertIntoGridMap */
	class TInsertionOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		/** Initilization of default parameters
		 */
		TInsertionOptions();

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
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** The altitude (z-axis) of 2D scans (within a 0.01m tolerance) for
		 * they to be inserted in this map! */
		float mapAltitude{0};
		/** The parameter "mapAltitude" has effect while inserting observations
		 * in the grid only if this is true. */
		bool useMapAltitude{false};
		/** The largest distance at which cells will be updated (Default 15
		 * meters) */
		float maxDistanceInsertion{15.0f};
		/** A value in the range [0.5,1] used for updating cell with a bayesian
		 * approach (default 0.8) */
		float maxOccupancyUpdateCertainty{0.65f};
		/** A value in the range [0.5,1] for updating a free cell. (default=0
		 * means use the same than maxOccupancyUpdateCertainty) */
		float maxFreenessUpdateCertainty{.0f};
		/** Like maxFreenessUpdateCertainty, but for invalid ranges (no echo).
		 * (default=0 means same than maxOccupancyUpdateCertainty)*/
		float maxFreenessInvalidRanges{.0f};
		/** If set to true (default), invalid range values (no echo rays) as
		 * consider as free space until "maxOccupancyUpdateCertainty", but ONLY
		 * when the previous and next rays are also an invalid ray. */
		bool considerInvalidRangesAsFreeSpace{true};
		/** Specify the decimation of the range scan (default=1 : take all the
		 * range values!) */
		uint16_t decimation{1};
		/** The tolerance in rads in pitch & roll for a laser scan to be
		 * considered horizontal, then processed by calls to this class
		 * (default=0). */
		float horizontalTolerance;
		/** Gaussian sigma of the filter used in getAsImageFiltered (for
		 * features detection) (Default=1) (0:Disabled)  */
		float CFD_features_gaussian_size{1};
		/** Size of the Median filter used in getAsImageFiltered (for features
		 * detection) (Default=3) (0:Disabled) */
		float CFD_features_median_size{3};
		/** Enabled: Rays widen with distance to approximate the real behavior
		 * of lasers, disabled: insert rays as simple lines (Default=false) */
		bool wideningBeamsWithDistance{false};
	};

	/** With this struct options are provided to the observation insertion
	 * process \sa CObservation::insertIntoGridMap */
	TInsertionOptions insertionOptions;

	/** The type for selecting a likelihood computation method */
	enum TLikelihoodMethod
	{
		lmMeanInformation = 0,
		lmRayTracing,
		lmConsensus,
		lmCellsDifference,
		lmLikelihoodField_Thrun,
		lmLikelihoodField_II,
		lmConsensusOWA
		// Remember: Update TEnumType below if new values are added here!
	};

	/** With this struct options are provided to the observation likelihood
	 * computation process */
	class TLikelihoodOptions : public mrpt::config::CLoadableOptions
	{
	   public:
		/** Initilization of default parameters */
		TLikelihoodOptions();

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
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** The selected method to compute an observation likelihood */
		TLikelihoodMethod likelihoodMethod{lmLikelihoodField_Thrun};
		/** [LikelihoodField] The laser range "sigma" used in computations;
		 * Default value = 0.35 */
		float LF_stdHit{0.35f};
		/** [LikelihoodField] */
		float LF_zHit{0.95f}, LF_zRandom{0.05f};
		/** [LikelihoodField] The max. range of the sensor (Default= 81 m) */
		float LF_maxRange{81.0f};
		/** [LikelihoodField] The decimation of the points in a scan, default=1
		 * == no decimation */
		uint32_t LF_decimation{5};
		/** [LikelihoodField] The max. distance for searching correspondences
		 * around each sensed point */
		float LF_maxCorrsDistance{0.3f};
		/** [LikelihoodField] (Default:false) Use `exp(dist^2/std^2)` instead of
		 * `exp(dist^2/std^2)` */
		bool LF_useSquareDist{false};
		/** [LikelihoodField] Set this to "true" ot use an alternative method,
		 * where the likelihood of the whole range scan is computed by
		 * "averaging" of individual ranges, instead of by the "product". */
		bool LF_alternateAverageMethod{false};
		/** [MI] The exponent in the MI likelihood computation. Default value =
		 * 5 */
		float MI_exponent{2.5f};
		/** [MI] The scan rays decimation: at every N rays, one will be used to
		 * compute the MI */
		uint32_t MI_skip_rays{10};
		/** [MI] The ratio for the max. distance used in the MI computation and
		 * in the insertion of scans, e.g. if set to 2.0 the MI will use twice
		 * the distance that the update distance. */
		float MI_ratio_max_distance{1.5f};
		/** [rayTracing] If true (default), the rayTracing method will ignore
		 * measured ranges shorter than the simulated ones. */
		bool rayTracing_useDistanceFilter{true};
		/** [rayTracing] One out of "rayTracing_decimation" rays will be
		 * simulated and compared only: set to 1 to use all the sensed ranges.
		 */
		int32_t rayTracing_decimation{10};
		/** [rayTracing] The laser range sigma. */
		float rayTracing_stdHit{1.0f};
		/** [Consensus] The down-sample ratio of ranges (default=1, consider all
		 * the ranges) */
		int32_t consensus_takeEachRange{1};
		/** [Consensus] The power factor for the likelihood (default=5) */
		float consensus_pow{5};
		/** [OWA] The sequence of weights to be multiplied to of the ordered
		 * list of likelihood values (first one is the largest); the size of
		 * this vector determines the number of highest likelihood values to
		 * fuse. */
		std::vector<float> OWA_weights;

		/** Enables the usage of a cache of likelihood values (for LF methods),
		 * if set to true (default=false). */
		bool enableLikelihoodCache{true};
	} likelihoodOptions;

	/** Auxiliary private class. */
	using TPairLikelihoodIndex = std::pair<double, mrpt::math::TPoint2D>;

	/** Some members of this struct will contain intermediate or output data
	 * after calling "computeObservationLikelihood" for some likelihood
	 * functions */
	class TLikelihoodOutput
	{
	   public:
		TLikelihoodOutput() : OWA_pairList(), OWA_individualLikValues() {}
		/** [OWA method] This will contain the ascending-ordered list of
		 * pairs:(likelihood values, 2D point in map coordinates). */
		std::vector<TPairLikelihoodIndex> OWA_pairList;
		/** [OWA method] This will contain the ascending-ordered list of
		 * likelihood values for individual range measurements in the scan. */
		std::vector<double> OWA_individualLikValues;
	} likelihoodOutputs;

	/** Performs a downsampling of the gridmap, by a given factor:
	 * resolution/=ratio */
	void subSample(int downRatio);

	/** Computes the entropy and related values of this grid map.
	 *  The entropy is computed as the summed entropy of each cell, taking them
	 * as discrete random variables following a Bernoulli distribution:
	 * \param info The output information is returned here */
	void computeEntropy(TEntropyInfo& info) const;

	/** @name Voronoi methods
		@{ */

	/** Build the Voronoi diagram of the grid map.
	 * \param threshold The threshold for binarizing the map.
	 * \param robot_size Size in "units" (meters) of robot, approx.
	 * \param x1 Left coordinate of area to be computed. Default, entire map.
	 * \param x2 Right coordinate of area to be computed. Default, entire map.
	 * \param y1 Top coordinate of area to be computed. Default, entire map.
	 * \param y2 Bottom coordinate of area to be computed. Default, entire map.
	 * \sa findCriticalPoints
	 */
	void buildVoronoiDiagram(
		float threshold, float robot_size, int x1 = 0, int x2 = 0, int y1 = 0,
		int y2 = 0);

	/** Reads a the clearance of a cell (in centimeters), after building the
	 * Voronoi diagram with \a buildVoronoiDiagram */
	inline uint16_t getVoroniClearance(int cx, int cy) const
	{
#ifdef _DEBUG
		ASSERT_ABOVEEQ_(cx, 0);
		ASSERT_ABOVEEQ_(cy, 0);
		ASSERT_BELOWEQ_(cx, int(m_voronoi_diagram.getSizeX()));
		ASSERT_BELOWEQ_(cy, int(m_voronoi_diagram.getSizeY()));
#endif
		const uint16_t* cell = m_voronoi_diagram.cellByIndex(cx, cy);
		return *cell;
	}

   protected:
	/** Used to set the clearance of a cell, while building the Voronoi diagram.
	 */
	inline void setVoroniClearance(int cx, int cy, uint16_t dist)
	{
		uint16_t* cell = m_voronoi_diagram.cellByIndex(cx, cy);
#ifdef _DEBUG
		ASSERT_ABOVEEQ_(cx, 0);
		ASSERT_ABOVEEQ_(cy, 0);
		ASSERT_BELOWEQ_(cx, int(m_voronoi_diagram.getSizeX()));
		ASSERT_BELOWEQ_(cy, int(m_voronoi_diagram.getSizeY()));
#endif
		*cell = dist;
	}

   public:
	/** Return the auxiliary "basis" map built while building the Voronoi
	 * diagram \sa buildVoronoiDiagram */
	inline const mrpt::containers::CDynamicGrid<uint8_t>& getBasisMap() const
	{
		return m_basis_map;
	}

	/** Return the Voronoi diagram; each cell contains the distance to its
	 * closer obstacle, or 0 if not part of the Voronoi diagram \sa
	 * buildVoronoiDiagram */
	inline const mrpt::containers::CDynamicGrid<uint16_t>& getVoronoiDiagram()
		const
	{
		return m_voronoi_diagram;
	}

	/** Builds a list with the critical points from Voronoi diagram, which must
	 *    must be built before calling this method.
	 * \param filter_distance The minimum distance between two critical points.
	 * \sa buildVoronoiDiagram
	 */
	void findCriticalPoints(float filter_distance);

	/** @} */  // End of Voronoi methods

	/** Compute the clearance of a given cell, and returns its two first
	 *   basis (closest obstacle) points.Used to build Voronoi and critical
	 * points.
	 * \return The clearance of the cell, in 1/100 of "cell".
	 * \param cx The cell index
	 * \param cy The cell index
	 * \param basis_x Target buffer for coordinates of basis, having a size of
	 * two "ints".
	 * \param basis_y Target buffer for coordinates of basis, having a size of
	 * two "ints".
	 * \param nBasis The number of found basis: Can be 0,1 or 2.
	 * \param GetContourPoint If "true" the basis are not returned, but the
	 * closest free cells.Default at false.
	 * \sa Build_VoronoiDiagram
	 */
	int computeClearance(
		int cx, int cy, int* basis_x, int* basis_y, int* nBasis,
		bool GetContourPoint = false) const;

	/** An alternative method for computing the clearance of a given location
	 * (in meters).
	 *  \return The clearance (distance to closest OCCUPIED cell), in meters.
	 */
	float computeClearance(float x, float y, float maxSearchDistance) const;

	/** Compute the 'cost' of traversing a segment of the map according to the
	 * occupancy of traversed cells.
	 *  \return This returns '1-mean(traversed cells occupancy)', i.e. 0.5 for
	 * unknown cells, 1 for a free path.
	 */
	float computePathCost(float x1, float y1, float x2, float y2) const;

	/** \name Sensor simulators
		@{ */

	/** Simulates a laser range scan into the current grid map.
	 *   The simulated scan is stored in a CObservation2DRangeScan object, which
	 *is also used
	 *    to pass some parameters: all previously stored characteristics (as
	 *aperture,...) are
	 *	  taken into account for simulation. Only a few more parameters are
	 *needed. Additive gaussian noise can be optionally added to the simulated
	 *scan.
	 * \param inout_Scan [IN/OUT] This must be filled with desired parameters
	 *before calling, and will contain the scan samples on return.
	 * \param robotPose [IN] The robot pose in this map coordinates. Recall that
	 *sensor pose relative to this robot pose must be specified in the
	 *observation object.
	 * \param threshold [IN] The minimum occupancy threshold to consider a cell
	 *to be occupied (Default: 0.5f)
	 * \param N [IN] The count of range scan "rays", by default to 361.
	 * \param noiseStd [IN] The standard deviation of measurement noise. If not
	 *desired, set to 0.
	 * \param decimation [IN] The rays that will be simulated are at indexes: 0,
	 *D, 2D, 3D, ... Default is D=1
	 * \param angleNoiseStd [IN] The sigma of an optional Gaussian noise added
	 *to the angles at which ranges are measured (in radians).
	 *
	 * \sa laserScanSimulatorWithUncertainty(), sonarSimulator(),
	 *COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS
	 */
	void laserScanSimulator(
		mrpt::obs::CObservation2DRangeScan& inout_Scan,
		const mrpt::poses::CPose2D& robotPose, float threshold = 0.6f,
		size_t N = 361, float noiseStd = 0, unsigned int decimation = 1,
		float angleNoiseStd = mrpt::DEG2RAD(.0)) const;

	/** Simulates the observations of a sonar rig into the current grid map.
	 *   The simulated ranges are stored in a CObservationRange object, which is
	 * also used
	 *    to pass in some needed parameters, as the poses of the sonar sensors
	 * onto the mobile robot.
	 * \param inout_observation [IN/OUT] This must be filled with desired
	 * parameters before calling, and will contain the simulated ranges on
	 * return.
	 * \param robotPose [IN] The robot pose in this map coordinates. Recall that
	 * sensor pose relative to this robot pose must be specified in the
	 * observation object.
	 * \param threshold [IN] The minimum occupancy threshold to consider a cell
	 * to be occupied (Default: 0.5f)
	 * \param rangeNoiseStd [IN] The standard deviation of measurement noise. If
	 * not desired, set to 0.
	 * \param angleNoiseStd [IN] The sigma of an optional Gaussian noise added
	 * to the angles at which ranges are measured (in radians).
	 *
	 * \sa laserScanSimulator(),
	 * COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS
	 */
	void sonarSimulator(
		mrpt::obs::CObservationRange& inout_observation,
		const mrpt::poses::CPose2D& robotPose, float threshold = 0.5f,
		float rangeNoiseStd = 0.f,
		float angleNoiseStd = mrpt::DEG2RAD(0.f)) const;

	/** Simulate just one "ray" in the grid map. This method is used internally
	 * to sonarSimulator and laserScanSimulator. \sa
	 * COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS */
	void simulateScanRay(
		const double x, const double y, const double angle_direction,
		float& out_range, bool& out_valid, const double max_range_meters,
		const float threshold_free = 0.4f, const double noiseStd = .0,
		const double angleNoiseStd = .0) const;

	/** Methods for TLaserSimulUncertaintyParams in
	 * laserScanSimulatorWithUncertainty() */
	enum TLaserSimulUncertaintyMethod
	{
		/** Performs an unscented transform */
		sumUnscented = 0,
		/** Montecarlo-based estimation */
		sumMonteCarlo
	};

	/** Input params for laserScanSimulatorWithUncertainty() */
	struct TLaserSimulUncertaintyParams
	{
		/** (Default: sumMonteCarlo) Select the method to do the uncertainty
		 * propagation */
		TLaserSimulUncertaintyMethod method{sumUnscented};
		/** @name Parameters for each uncertainty method
			@{ */
		/** [sumUnscented] UT parameters. Defaults: alpha=0.99, kappa=0,
		 * betta=2.0 */
		double UT_alpha{0.99}, UT_kappa{.0}, UT_beta{2.0};
		/** [sumMonteCarlo] MonteCarlo parameter: number of samples (Default:
		 * 10) */
		size_t MC_samples{10};
		/** @} */

		/** @name Generic parameters for all methods
			@{ */
		/** The robot pose Gaussian, in map coordinates. Recall that sensor pose
		 * relative to this robot pose must be specified in the observation
		 * object */
		mrpt::poses::CPosePDFGaussian robotPose;
		/** (Default: M_PI) The "aperture" or field-of-view of the range finder,
		 * in radians (typically M_PI = 180 degrees). */
		float aperture{M_PIf};
		/** (Default: true) The scanning direction: true=counterclockwise;
		 * false=clockwise */
		bool rightToLeft{true};
		/** (Default: 80) The maximum range allowed by the device, in meters
		 * (e.g. 80m, 50m,...) */
		float maxRange{80.f};
		/** (Default: at origin) The 6D pose of the sensor on the robot at the
		 * moment of starting the scan. */
		mrpt::poses::CPose3D sensorPose;
		size_t nRays{361};
		/** (Default: 0) The standard deviation of measurement noise. If not
		 * desired, set to 0 */
		float rangeNoiseStd{.0f};
		/** (Default: 0) The sigma of an optional Gaussian noise added to the
		 * angles at which ranges are measured (in radians) */
		float angleNoiseStd{.0f};
		/** (Default: 1) The rays that will be simulated are at indexes: 0, D,
		 * 2D, 3D,... */
		unsigned int decimation{1};
		/** (Default: 0.6f) The minimum occupancy threshold to consider a cell
		 * to be occupied */
		float threshold{.6f};
		/** @} */

		TLaserSimulUncertaintyParams();
	};

	/** Output params for laserScanSimulatorWithUncertainty() */
	struct TLaserSimulUncertaintyResult
	{
		/** The scan + its uncertainty */
		mrpt::obs::CObservation2DRangeScanWithUncertainty scanWithUncert;

		TLaserSimulUncertaintyResult();
	};

	/** Like laserScanSimulatorWithUncertainty() (see it for a discussion of
	 * most parameters) but taking into account
	 *  the robot pose uncertainty and generating a vector of predicted
	 * variances for each ray.
	 *  Range uncertainty includes both, sensor noise and large non-linear
	 * effects caused by borders and discontinuities in the environment
	 *  as seen from different robot poses.
	 *
	 * \param in_params [IN] Input settings. See TLaserSimulUncertaintyParams
	 * \param in_params [OUT] Output range + uncertainty.
	 *
	 * \sa laserScanSimulator(),
	 * COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS
	 */
	void laserScanSimulatorWithUncertainty(
		const TLaserSimulUncertaintyParams& in_params,
		TLaserSimulUncertaintyResult& out_results) const;

	/** @} */

	/** Computes the likelihood [0,1] of a set of points, given the current grid
	 * map as reference.
	 * \param pm The points map
	 * \param relativePose The relative pose of the points map in this map's
	 * coordinates, or nullptr for (0,0,0).
	 *  See "likelihoodOptions" for configuration parameters.
	 */
	double computeLikelihoodField_Thrun(
		const CPointsMap* pm,
		const mrpt::poses::CPose2D* relativePose = nullptr);

	/** Computes the likelihood [0,1] of a set of points, given the current grid
	 * map as reference.
	 * \param pm The points map
	 * \param relativePose The relative pose of the points map in this map's
	 * coordinates, or nullptr for (0,0,0).
	 *  See "likelihoodOptions" for configuration parameters.
	 */
	double computeLikelihoodField_II(
		const CPointsMap* pm,
		const mrpt::poses::CPose2D* relativePose = nullptr);

	/** Saves the gridmap as a graphical file (BMP,PNG,...).
	 * The format will be derived from the file extension (see
	 * CImage::saveToFile )
	 * \return False on any error.
	 */
	bool saveAsBitmapFile(const std::string& file) const;

	/** Saves a composite image with two gridmaps and lines representing a set
	 * of correspondences between them.
	 * \sa saveAsEMFTwoMapsWithCorrespondences
	 * \return False on any error.
	 */
	static bool saveAsBitmapTwoMapsWithCorrespondences(
		const std::string& fileName, const COccupancyGridMap2D* m1,
		const COccupancyGridMap2D* m2,
		const mrpt::tfest::TMatchingPairList& corrs);

	/** Saves a composite image with two gridmaps and numbers for the
	 * correspondences between them.
	 * \sa saveAsBitmapTwoMapsWithCorrespondences
	 * \return False on any error.
	 */
	static bool saveAsEMFTwoMapsWithCorrespondences(
		const std::string& fileName, const COccupancyGridMap2D* m1,
		const COccupancyGridMap2D* m2,
		const mrpt::tfest::TMatchingPairList& corrs);

	/** Saves the gridmap as a graphical bitmap file, 8 bit gray scale, 1 pixel
	 * is 1 cell, and with an overlay of landmarks.
	 * \note The template parameter CLANDMARKSMAP is assumed to be
	 * mrpt::maps::CLandmarksMap normally.
	 * \return False on any error.
	 */
	template <class CLANDMARKSMAP>
	bool saveAsBitmapFileWithLandmarks(
		const std::string& file, const CLANDMARKSMAP* landmarks,
		bool addTextLabels = false,
		const mrpt::img::TColor& marks_color =
			mrpt::img::TColor(0, 0, 255)) const
	{
		MRPT_START
		mrpt::img::CImage img(1, 1, 3);
		getAsImageFiltered(img, false, true);  // in RGB
		const bool topleft = img.isOriginTopLeft();
		for (unsigned int i = 0; i < landmarks->landmarks.size(); i++)
		{
			const typename CLANDMARKSMAP::landmark_type* lm =
				landmarks->landmarks.get(i);
			int px = x2idx(lm->pose_mean.x);
			int py = topleft ? size_y - 1 - y2idx(lm->pose_mean.y)
							 : y2idx(lm->pose_mean.y);
			img.rectangle(px - 7, (py + 7), px + 7, (py - 7), marks_color);
			img.rectangle(px - 6, (py + 6), px + 6, (py - 6), marks_color);
			if (addTextLabels)
				img.textOut(
					px, py - 8, format("%u", i), mrpt::img::TColor::black());
		}
		return img.saveToFile(file.c_str());
		MRPT_END
	}

	/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell
	 * (output image is RGB only if forceRGB is true)
	 *  If "tricolor" is true, only three gray levels will appear in the image:
	 * gray for unobserved cells, and black/white for occupied/empty cells
	 * respectively.
	 * \sa getAsImageFiltered
	 */
	void getAsImage(
		mrpt::img::CImage& img, bool verticalFlip = false,
		bool forceRGB = false, bool tricolor = false) const;

	/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell
	 * (output image is RGB only if forceRGB is true) - This method filters the
	 * image for easy feature detection
	 *  If "tricolor" is true, only three gray levels will appear in the image:
	 * gray for unobserved cells, and black/white for occupied/empty cells
	 * respectively.
	 * \sa getAsImage
	 */
	void getAsImageFiltered(
		img::CImage& img, bool verticalFlip = false,
		bool forceRGB = false) const;

	/** Returns a 3D plane with its texture being the occupancy grid and
	 * transparency proportional to "uncertainty" (i.e. a value of 0.5 is fully
	 * transparent)
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Get a point cloud with all (border) occupied cells as points */
	void getAsPointCloud(
		mrpt::maps::CSimplePointsMap& pm,
		const float occup_threshold = 0.5f) const;

	/** Returns true upon map construction or after calling clear(), the return
	 *  changes to false upon successful insertObservation() or any other
	 * method to load data in the map.
	 */
	bool isEmpty() const override;

	/** Load the gridmap from a image in a file (the format can be any supported
	 * by CImage::loadFromFile).
	 * \param file The file to be loaded.
	 * \param resolution The size of a pixel (cell), in meters. Recall cells are
	 * always squared, so just a dimension is needed.
	 * \param origin The `x` (0=first, increases <b>left to right</b> on the
	 * image) and `y` (0=first, increases <b>BOTTOM upwards</b> on the image)
	 * coordinates for the pixel which will be taken at the origin of map
	 * coordinates (0,0). (Default=center of the image) \return False on any
	 * error. \sa loadFromBitmap
	 */
	bool loadFromBitmapFile(
		const std::string& file, float resolution,
		const mrpt::math::TPoint2D& origin = mrpt::math::TPoint2D(
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max()));

	/** Load the gridmap from a image in a file (the format can be any supported
	 * by CImage::loadFromFile).
	 *  See loadFromBitmapFile() for the meaning of parameters */
	bool loadFromBitmap(
		const mrpt::img::CImage& img, float resolution,
		const mrpt::math::TPoint2D& origin = mrpt::math::TPoint2D(
			std::numeric_limits<double>::max(),
			std::numeric_limits<double>::max()));

	/** See the base class for more details: In this class it is implemented as
	 * correspondences of the passed points map to occupied cells.
	 * NOTICE: That the "z" dimension is ignored in the points. Clip the points
	 * as appropiated if needed before calling this method.
	 *
	 * \sa computeMatching3DWith
	 */
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

	/** This virtual method saves the map to a file "filNamePrefix"+<
	 * some_file_extension >, as an image or in any other applicable way (Notice
	 * that other methods to save the map may be implemented in classes
	 * implementing this virtual interface).  */
	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	/** The structure used to store the set of Voronoi diagram
	 *    critical points.
	 * \sa findCriticalPoints
	 */
	struct TCriticalPointsList
	{
		TCriticalPointsList()
			: x(),
			  y(),
			  clearance(),
			  x_basis1(),
			  y_basis1(),
			  x_basis2(),
			  y_basis2()
		{
		}

		/** The coordinates of critical point. */
		std::vector<int> x, y;
		/** The clearance of critical points, in 1/100 of cells. */
		std::vector<int> clearance;
		/** Their two first basis points coordinates. */
		std::vector<int> x_basis1, y_basis1, x_basis2, y_basis2;
	} CriticalPointsList;

   private:
	// See docs in base class
	double internal_computeObservationLikelihood(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D& takenFrom) override;
	// See docs in base class
	bool internal_canComputeObservationLikelihood(
		const mrpt::obs::CObservation* obs) const override;

	/** Returns a byte with the occupancy of the 8 sorrounding cells.
	 * \param cx The cell index
	 * \param cy The cell index
	 * \sa direction2idx
	 */
	inline unsigned char GetNeighborhood(int cx, int cy) const;

	/** Used to store the 8 possible movements from a cell to the
	 *   sorrounding ones.Filled in the constructor.
	 * \sa direction2idx
	 */
	int direccion_vecino_x[8], direccion_vecino_y[8];

	/** Returns the index [0,7] of the given movement, or
	 *  -1 if invalid one.
	 * \sa direccion_vecino_x,direccion_vecino_y,GetNeighborhood
	 */
	int direction2idx(int dx, int dy);

	MAP_DEFINITION_START(COccupancyGridMap2D)
	/** See COccupancyGridMap2D::COccupancyGridMap2D */
	float min_x{-10.0f}, max_x{10.0f}, min_y{-10.0f}, max_y{10.0f},
		resolution{0.10f};
	/** Observations insertion options */
	mrpt::maps::COccupancyGridMap2D::TInsertionOptions insertionOpts;
	/** Probabilistic observation likelihood options */
	mrpt::maps::COccupancyGridMap2D::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(COccupancyGridMap2D)
};

bool operator<(
	const COccupancyGridMap2D::TPairLikelihoodIndex& e1,
	const COccupancyGridMap2D::TPairLikelihoodIndex& e2);
}  // namespace mrpt::maps
MRPT_ENUM_TYPE_BEGIN(mrpt::maps::COccupancyGridMap2D::TLikelihoodMethod)
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmMeanInformation);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmRayTracing);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmConsensus);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmCellsDifference);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmLikelihoodField_Thrun);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmLikelihoodField_II);
MRPT_FILL_ENUM_MEMBER(mrpt::maps::COccupancyGridMap2D, lmConsensusOWA);
MRPT_ENUM_TYPE_END()
