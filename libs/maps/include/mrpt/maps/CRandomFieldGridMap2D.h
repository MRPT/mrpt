/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/img/CImage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/graphs/ScalarFactorGraph.h>

#include <list>

namespace mrpt::maps
{
class COccupancyGridMap2D;

// Pragma defined to ensure no structure packing: since we'll serialize
// TRandomFieldCell to streams, we want it not to depend on compiler options,
// etc.
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(push, 1)
#endif

/** The contents of each cell in a CRandomFieldGridMap2D map.
 * \ingroup mrpt_maps_grp
 **/
struct TRandomFieldCell
{
	/** Constructor */
	TRandomFieldCell(double kfmean_dm_mean = 1e-20, double kfstd_dmmeanw = 0)
		: kf_mean(kfmean_dm_mean),
		  kf_std(kfstd_dmmeanw),

		  last_updated(mrpt::system::now()),
		  updated_std(kfstd_dmmeanw)
	{
	}

	// *Note*: Use unions to share memory between data fields, since only a set
	//          of the variables will be used for each mapping strategy.
	// You can access to a "TRandomFieldCell *cell" like: cell->kf_mean,
	// cell->kf_std, etc..
	//  but accessing cell->kf_mean would also modify (i.e. ARE the same memory
	//  slot) cell->dm_mean, for example.

	// Note 2: If the number of type of fields are changed in the future,
	//   *PLEASE* also update the writeToStream() and readFromStream() methods!!

	union {
		/** [KF-methods only] The mean value of this cell */
		double kf_mean;
		/** [Kernel-methods only] The cumulative weighted readings of this cell
		 */
		double dm_mean;
		/** [GMRF only] The mean value of this cell */
		double gmrf_mean;
	};

	union {
		/** [KF-methods only] The standard deviation value of this cell */
		double kf_std;
		/** [Kernel-methods only] The cumulative weights (concentration = alpha
		 * * dm_mean / dm_mean_w + (1-alpha)*r0 ) */
		double dm_mean_w;
		double gmrf_std;
	};

	/** [Kernel DM-V only] The cumulative weighted variance of this cell */
	double dmv_var_mean{0};

	/** [Dynamic maps only] The timestamp of the last time the cell was updated
	 */
	mrpt::system::TTimeStamp last_updated;
	/** [Dynamic maps only] The std cell value that was updated (to be used in
	 * the Forgetting_curve */
	double updated_std;
};

#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)
#endif

/** CRandomFieldGridMap2D represents a 2D grid map where each cell is associated
 *one real-valued property which is estimated by this map, either
 *   as a simple value or as a probility distribution (for each cell).
 *
 *  There are a number of methods available to build the MRF grid-map,
 *depending on the value of
 *    `TMapRepresentation maptype` passed in the constructor.
 *
 *  The following papers describe the mapping alternatives implemented here:
 *		- `mrKernelDM`: A Gaussian kernel-based method. See:
 *			- "Building gas concentration gridmaps with a mobile robot",
 *Lilienthal,
 *A. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.
 *		- `mrKernelDMV`: A kernel-based method. See:
 *			- "A Statistical Approach to Gas Distribution Modelling with Mobile
 *Robots--The Kernel DM+ V Algorithm", Lilienthal, A.J. and Reggente, M. and
 *Trincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.
 *		- `mrKalmanFilter`: A "brute-force" approach to estimate the entire map
 *with a dense (linear) Kalman filter. Will be very slow for mid or large maps.
 *It's provided just for comparison purposes, not useful in practice.
 *		- `mrKalmanApproximate`: A compressed/sparse Kalman filter approach.
 *See:
 *			- "A Kalman Filter Based Approach to Probabilistic Gas Distribution
 *Mapping", JL Blanco, JG Monroy, J Gonzalez-Jimenez, A Lilienthal, 28th
 *Symposium On Applied Computing (SAC), 2013.
 *		- `mrGMRF_SD`: A Gaussian Markov Random Field (GMRF) estimator, with
 *these
 *constraints:
 *			- `mrGMRF_SD`: Each cell only connected to its 4 immediate neighbors
 *(Up,
 *down, left, right).
 *			- (Removed in MRPT 1.5.0: `mrGMRF_G`: Each cell connected to a
 *square
 *area
 *of neighbors cells)
 *			- See papers:
 *				- "Time-variant gas distribution mapping with obstacle
 *information",
 *Monroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots,
 *40(1), 1-16, 2016.
 *
 *  Note that this class is virtual, since derived classes still have to
 *implement:
 *		- mrpt::maps::CMetricMap::internal_computeObservationLikelihood()
 *		- mrpt::maps::CMetricMap::internal_insertObservation()
 *		- Serialization methods: writeToStream() and readFromStream()
 *
 * [GMRF only] A custom connectivity pattern between cells can be defined by
 *calling setCellsConnectivity().
 *
 * \sa mrpt::maps::CGasConcentrationGridMap2D,
 *mrpt::maps::CWirelessPowerGridMap2D, mrpt::maps::CMetricMap,
 *mrpt::containers::CDynamicGrid, The application icp-slam,
 *mrpt::maps::CMultiMetricMap
 * \ingroup mrpt_maps_grp
 */
class CRandomFieldGridMap2D
	: public mrpt::maps::CMetricMap,
	  public mrpt::containers::CDynamicGrid<TRandomFieldCell>,
	  public mrpt::system::COutputLogger
{
	using BASE = mrpt::containers::CDynamicGrid<TRandomFieldCell>;

	DEFINE_VIRTUAL_SERIALIZABLE(CRandomFieldGridMap2D)
   public:
	/** Calls the base CMetricMap::clear
	 * Declared here to avoid ambiguity between the two clear() in both base
	 * classes.
	 */
	inline void clear() { CMetricMap::clear(); }
	// This method is just used for the ::saveToTextFile() method in base class.
	float cell2float(const TRandomFieldCell& c) const override
	{
		return c.kf_mean;
	}

	/** The type of map representation to be used, see CRandomFieldGridMap2D for
	 * a discussion.
	 */
	enum TMapRepresentation
	{
		/** Gaussian kernel-based estimator (see discussion in
		   mrpt::maps::CRandomFieldGridMap2D) */
		mrKernelDM = 0,
		/** Another alias for "mrKernelDM", for backwards compatibility (see
		   discussion in mrpt::maps::CRandomFieldGridMap2D) */
		mrAchim = 0,
		/** "Brute-force" Kalman filter (see discussion in
		   mrpt::maps::CRandomFieldGridMap2D) */
		mrKalmanFilter,
		/** (see discussion in mrpt::maps::CRandomFieldGridMap2D) */
		mrKalmanApproximate,
		/** Double mean + variance Gaussian kernel-based estimator (see
		   discussion in mrpt::maps::CRandomFieldGridMap2D) */
		mrKernelDMV,
		// Removed in MRPT 1.5.0: mrGMRF_G,   //!< Gaussian Markov Random Field,
		// Gaussian prior weights between neighboring cells up to a certain
		// distance (see discussion in mrpt::maps::CRandomFieldGridMap2D)
		/** Gaussian Markov Random Field, squared differences prior weights
		   between 4 neighboring cells (see discussion in
		   mrpt::maps::CRandomFieldGridMap2D) */
		mrGMRF_SD
	};

	/** Constructor */
	CRandomFieldGridMap2D(
		TMapRepresentation mapType = mrKernelDM, double x_min = -2,
		double x_max = 2, double y_min = -2, double y_max = 2,
		double resolution = 0.1);

	/** Destructor */
	~CRandomFieldGridMap2D() override;

	/** Returns true if the map is empty/no observation has been inserted (in
	 * this class it always return false,
	 * unless redefined otherwise in base classes)
	 */
	bool isEmpty() const override;

	/** Save the current map as a graphical file (BMP,PNG,...).
	 * The file format will be derived from the file extension (see
	 *CImage::saveToFile )
	 *  It depends on the map representation model:
	 *		mrAchim: Each pixel is the ratio \f$ \sum{\frac{wR}{w}} \f$
	 *		mrKalmanFilter: Each pixel is the mean value of the Gaussian that
	 *represents each cell.
	 *
	 * \sa \a getAsBitmapFile()
	 */
	virtual void saveAsBitmapFile(const std::string& filName) const;

	/** Returns an image just as described in \a saveAsBitmapFile */
	virtual void getAsBitmapFile(mrpt::img::CImage& out_img) const;

	/** Like saveAsBitmapFile(), but returns the data in matrix form (first row
	 * in the matrix is the upper (y_max) part of the map) */
	virtual void getAsMatrix(mrpt::math::CMatrixDouble& out_mat) const;

	/** Parameters common to any derived class.
	 *  Derived classes should derive a new struct from this one, plus "public
	 * utils::CLoadableOptions",
	 *  and call the internal_* methods where appropiate to deal with the
	 * variables declared here.
	 *  Derived classes instantions of their "TInsertionOptions" MUST set the
	 * pointer "m_insertOptions_common" upon construction.
	 */
	struct TInsertionOptionsCommon
	{
		/** Default values loader */
		TInsertionOptionsCommon();

		/** See utils::CLoadableOptions */
		void internal_loadFromConfigFile_common(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section);

		/** See utils::CLoadableOptions */
		void internal_dumpToTextStream_common(std::ostream& out) const;

		/** @name Kernel methods (mrKernelDM, mrKernelDMV)
			@{ */
		/** The sigma of the "Parzen"-kernel Gaussian */
		float sigma{0.15f};
		/** The cutoff radius for updating cells. */
		float cutoffRadius;
		/** Limits for normalization of sensor readings. */
		float R_min{0}, R_max{3};
		/** [DM/DM+V methods] The scaling parameter for the confidence "alpha"
		 * values (see the IROS 2009 paper; see CRandomFieldGridMap2D) */
		double dm_sigma_omega{0.05};
		/** @} */

		/** @name Kalman-filter methods (mrKalmanFilter, mrKalmanApproximate)
			@{ */
		/** The "sigma" for the initial covariance value between cells (in
		 * meters). */
		float KF_covSigma{0.35f};
		/** The initial standard deviation of each cell's concentration (will be
		 * stored both at each cell's structure and in the covariance matrix as
		 * variances in the diagonal) (in normalized concentration units). */
		float KF_initialCellStd{1.0};
		/** The sensor model noise (in normalized concentration units). */
		float KF_observationModelNoise{0};
		/** The default value for the mean of cells' concentration. */
		float KF_defaultCellMeanValue{0};
		/** [mrKalmanApproximate] The size of the window of neighbor cells. */
		uint16_t KF_W_size{4};
		/** @} */

		/** @name Gaussian Markov Random Fields methods (mrGMRF_SD)
			@{ */
		/** The information (Lambda) of fixed map constraints */
		double GMRF_lambdaPrior{0.01f};
		/** The initial information (Lambda) of each observation (this
		 * information will decrease with time) */
		double GMRF_lambdaObs{10.0f};
		/** The loss of information of the observations with each iteration */
		double GMRF_lambdaObsLoss{0.0f};

		/** whether to use information of an occupancy_gridmap map for building
		 * the GMRF */
		bool GMRF_use_occupancy_information{false};
		/** simplemap_file name of the occupancy_gridmap */
		std::string GMRF_simplemap_file;
		/** image name of the occupancy_gridmap */
		std::string GMRF_gridmap_image_file;
		/** occupancy_gridmap resolution: size of each pixel (m) */
		double GMRF_gridmap_image_res{0.01f};
		/** Pixel coordinates of the origin for the occupancy_gridmap */
		size_t GMRF_gridmap_image_cx{0};
		/** Pixel coordinates of the origin for the occupancy_gridmap */
		size_t GMRF_gridmap_image_cy{0};

		/** (Default:-inf,+inf) Saturate the estimated mean in these limits */
		double GMRF_saturate_min, GMRF_saturate_max;
		/** (Default:false) Skip the computation of the variance, just compute
		 * the mean */
		bool GMRF_skip_variance{false};
		/** @} */
	};

	/** Changes the size of the grid, maintaining previous contents. \sa setSize
	 */
	void resize(
		double new_x_min, double new_x_max, double new_y_min, double new_y_max,
		const TRandomFieldCell& defaultValueNewCells,
		double additionalMarginMeters = 1.0f) override;

	/** Changes the size of the grid, erasing previous contents.
	 *  \param[in] connectivity_descriptor Optional user-supplied object that
	 * will visit all grid cells to define their connectivity with neighbors and
	 * the strength of existing edges. If present, it overrides all options in
	 * insertionOptions
	 * \sa resize
	 */
	virtual void setSize(
		const double x_min, const double x_max, const double y_min,
		const double y_max, const double resolution,
		const TRandomFieldCell* fill_value = nullptr);

	/** Base class for user-supplied objects capable of describing cells
	 * connectivity, used to build prior factors of the MRF graph. \sa
	 * setCellsConnectivity() */
	struct ConnectivityDescriptor
	{
		using Ptr = std::shared_ptr<ConnectivityDescriptor>;
		ConnectivityDescriptor();
		virtual ~ConnectivityDescriptor();

		/** Implement the check of whether node i=(icx,icy) is connected with
		 * node j=(jcx,jcy).
		 * This visitor method will be called only for immediate neighbors.
		 * \return true if connected (and the "information" value should be
		 * also updated in out_edge_information), false otherwise.
		 */
		virtual bool getEdgeInformation(
			/** The parent map on which we are running */
			const CRandomFieldGridMap2D* parent,
			/** (cx,cy) for node "i" */
			size_t icx, size_t icy,
			/** (cx,cy) for node "j" */
			size_t jcx, size_t jcy,
			/** Must output here the inverse of the variance of the constraint
			   edge. */
			double& out_edge_information) = 0;
	};

	/** Sets a custom object to define the connectivity between cells. Must call
	 * clear() or setSize() afterwards for the changes to take place. */
	void setCellsConnectivity(
		const ConnectivityDescriptor::Ptr& new_connectivity_descriptor);

	/** See docs in base class: in this class this always returns 0 */
	float compute3DMatchingRatio(
		const mrpt::maps::CMetricMap* otherMap,
		const mrpt::poses::CPose3D& otherMapPose,
		const TMatchingRatioParams& params) const override;

	/** The implementation in this class just calls all the corresponding method
	 * of the contained metric maps */
	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override;

	/** Save a matlab ".m" file which represents as 3D surfaces the mean and a
	 * given confidence level for the concentration of each cell.
	 *  This method can only be called in a KF map model.
	 *  \sa getAsMatlab3DGraphScript    */
	virtual void saveAsMatlab3DGraph(const std::string& filName) const;

	/** Return a large text block with a MATLAB script to plot the contents of
	 * this map \sa saveAsMatlab3DGraph
	 *  This method can only be called in a KF map model  */
	void getAsMatlab3DGraphScript(std::string& out_script) const;

	/** Returns a 3D object representing the map (mean) */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** Returns two 3D objects representing the mean and variance maps */
	virtual void getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr& meanObj,
		mrpt::opengl::CSetOfObjects::Ptr& varObj) const;

	/** Return the type of the random-field grid map, according to parameters
	 * passed on construction. */
	TMapRepresentation getMapType();

	/** Direct update of the map with a reading in a given position of the map,
	 * using
	 *  the appropriate method according to mapType passed in the constructor.
	 *
	 * This is a direct way to update the map, an alternative to the generic
	 * insertObservation() method which works with mrpt::obs::CObservation
	 * objects.
	 */
	void insertIndividualReading(
		/** [in] The value observed in the (x,y) position */
		const double sensorReading,
		/** [in] The (x,y) location */
		const mrpt::math::TPoint2D& point,
		/** [in] Run a global map update after inserting this observatin
		   (algorithm-dependant) */
		const bool update_map = true,
		/** [in] Whether the observation "vanishes" with time (false) or not
		   (true) [Only for GMRF methods] */
		const bool time_invariant = true,
		/** [in] The uncertainty (standard deviation) of the reading.
		   Default="0.0" means use the default settings per map-wide parameters.
		   */
		const double reading_stddev = .0);

	enum TGridInterpolationMethod
	{
		gimNearest = 0,
		gimBilinear
	};

	/** Returns the prediction of the measurement at some (x,y) coordinates, and
	 * its certainty (in the form of the expected variance).  */
	virtual void predictMeasurement(
		/** [in] Query X coordinate */
		const double x,
		/** [in] Query Y coordinate */
		const double y,
		/** [out] The output value */
		double& out_predict_response,
		/** [out] The output variance */
		double& out_predict_response_variance,
		/** [in] Whether to renormalize the prediction to a predefined
		   interval (`R` values in insertionOptions) */
		bool do_sensor_normalization,
		/** [in] Interpolation method */
		const TGridInterpolationMethod interp_method = gimNearest);

	/** Return the mean and covariance vector of the full Kalman filter estimate
	 * (works for all KF-based methods). */
	void getMeanAndCov(
		mrpt::math::CVectorDouble& out_means,
		mrpt::math::CMatrixDouble& out_cov) const;

	/** Return the mean and STD vectors of the full Kalman filter estimate
	 * (works for all KF-based methods). */
	void getMeanAndSTD(
		mrpt::math::CVectorDouble& out_means,
		mrpt::math::CVectorDouble& out_STD) const;

	/** Load the mean and STD vectors of the full Kalman filter estimate (works
	 * for all KF-based methods). */
	void setMeanAndSTD(
		mrpt::math::CVectorDouble& out_means,
		mrpt::math::CVectorDouble& out_STD);

	/** Run the method-specific procedure required to ensure that the mean &
	 * variances are up-to-date with all inserted observations. */
	void updateMapEstimation();

	void enableVerbose(bool enable_verbose)
	{
		this->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
	}
	bool isEnabledVerbose() const
	{
		return this->getMinLoggingLevel() == mrpt::system::LVL_DEBUG;
	}

	void enableProfiler(bool enable = true)
	{
		this->m_gmrf.enableProfiler(enable);
	}
	bool isProfilerEnabled() const { return this->m_gmrf.isProfilerEnabled(); }

   protected:
	bool m_rfgm_run_update_upon_clear{true};

	/** Common options to all random-field grid maps: pointer that is set to the
	 * derived-class instance of "insertOptions" upon construction of this
	 * class. */
	TInsertionOptionsCommon* m_insertOptions_common{nullptr};

	/** Get the part of the options common to all CRandomFieldGridMap2D classes
	 */
	virtual CRandomFieldGridMap2D::TInsertionOptionsCommon*
		getCommonInsertOptions() = 0;

	/** The map representation type of this map, as passed in the constructor */
	TMapRepresentation m_mapType;

	/** The whole covariance matrix, used for the Kalman Filter map
	 * representation. */
	mrpt::math::CMatrixD m_cov;

	/** The compressed band diagonal matrix for the KF2 implementation.
	 *   The format is a Nx(W^2+2W+1) matrix, one row per cell in the grid map
	 * with the
	 *    cross-covariances between each cell and half of the window around it
	 * in the grid.
	 */
	mrpt::math::CMatrixD m_stackedCov;
	/** Only for the KF2 implementation. */
	mutable bool m_hasToRecoverMeanAndCov{true};

	/** @name Auxiliary vars for DM & DM+V methods
		@{ */
	float m_DM_lastCutOff{0};
	std::vector<float> m_DM_gaussWindow;
	double m_average_normreadings_mean{0}, m_average_normreadings_var{0};
	size_t m_average_normreadings_count{0};
	/** @} */

	/** Empty: default */
	ConnectivityDescriptor::Ptr m_gmrf_connectivity;

	mrpt::graphs::ScalarFactorGraph m_gmrf;

	struct TObservationGMRF
		: public mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase
	{
		/** Observation value */
		double obsValue;
		/** "Information" of the observation (=inverse of the variance) */
		double Lambda;
		/** whether the observation will lose weight (lambda) as time goes on
		 * (default false) */
		bool time_invariant;

		double evaluateResidual() const override;
		double getInformation() const override;
		void evalJacobian(double& dr_dx) const override;

		TObservationGMRF(CRandomFieldGridMap2D& parent)
			: obsValue(.0), Lambda(.0), time_invariant(false), m_parent(&parent)
		{
		}

	   private:
		CRandomFieldGridMap2D* m_parent;
	};

	struct TPriorFactorGMRF
		: public mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase
	{
		/** "Information" of the observation (=inverse of the variance) */
		double Lambda;

		double evaluateResidual() const override;
		double getInformation() const override;
		void evalJacobian(double& dr_dx_i, double& dr_dx_j) const override;

		TPriorFactorGMRF(CRandomFieldGridMap2D& parent)
			: Lambda(.0), m_parent(&parent)
		{
		}

	   private:
		CRandomFieldGridMap2D* m_parent;
	};

	// Important: converted to a std::list<> so pointers are NOT invalidated
	// upon deletion.
	/** Vector with the active observations and their respective Information */
	std::vector<std::list<TObservationGMRF>> m_mrf_factors_activeObs;
	/** Vector with the precomputed priors for each GMRF model */
	std::deque<TPriorFactorGMRF> m_mrf_factors_priors;

	/** The implementation of "insertObservation" for Achim Lilienthal's map
	 * models DM & DM+V.
	 * \param normReading Is a [0,1] normalized concentration reading.
	 * \param point Is the sensor location on the map
	 * \param is_DMV = false -> map type is Kernel DM; true -> map type is DM+V
	 */
	void insertObservation_KernelDM_DMV(
		double normReading, const mrpt::math::TPoint2D& point, bool is_DMV);

	/** The implementation of "insertObservation" for the (whole) Kalman Filter
	 * map model.
	 * \param normReading Is a [0,1] normalized concentration reading.
	 * \param point Is the sensor location on the map
	 */
	void insertObservation_KF(
		double normReading, const mrpt::math::TPoint2D& point);

	/** The implementation of "insertObservation" for the Efficient Kalman
	 * Filter map model.
	 * \param normReading Is a [0,1] normalized concentration reading.
	 * \param point Is the sensor location on the map
	 */
	void insertObservation_KF2(
		double normReading, const mrpt::math::TPoint2D& point);

	/** The implementation of "insertObservation" for the Gaussian Markov Random
	 * Field map model.
	 * \param normReading Is a [0,1] normalized concentration reading.
	 * \param point Is the sensor location on the map
	 */
	void insertObservation_GMRF(
		double normReading, const mrpt::math::TPoint2D& point,
		const bool update_map, const bool time_invariant,
		const double reading_information);

	/** solves the minimum quadratic system to determine the new concentration
	 * of each cell */
	void updateMapEstimation_GMRF();

	/** Computes the confidence of the cell concentration (alpha) */
	double computeConfidenceCellValue_DM_DMV(
		const TRandomFieldCell* cell) const;

	/** Computes the average cell concentration, or the overall average value if
	 * it has never been observed  */
	double computeMeanCellValue_DM_DMV(const TRandomFieldCell* cell) const;

	/** Computes the estimated variance of the cell concentration, or the
	 * overall average variance if it has never been observed  */
	double computeVarCellValue_DM_DMV(const TRandomFieldCell* cell) const;

	/** In the KF2 implementation, takes the auxiliary matrices and from them
	 * update the cells' mean and std values.
	 * \sa m_hasToRecoverMeanAndCov
	 */
	void recoverMeanAndCov() const;

	/** Erase all the contents of the map */
	void internal_clear() override;

	/** Check if two cells of the gridmap (m_map) are connected, based on the
	 * provided occupancy gridmap*/
	bool exist_relation_between2cells(
		const mrpt::maps::COccupancyGridMap2D* m_Ocgridmap, size_t cxo_min,
		size_t cxo_max, size_t cyo_min, size_t cyo_max, const size_t seed_cxo,
		const size_t seed_cyo, const size_t objective_cxo,
		const size_t objective_cyo);
};

}  // namespace mrpt::maps
MRPT_ENUM_TYPE_BEGIN(mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation)
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, mrKernelDM);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, mrKalmanFilter);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, mrKalmanApproximate);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, mrKernelDMV);
MRPT_FILL_ENUM_MEMBER(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation, mrGMRF_SD);
MRPT_ENUM_TYPE_END()
