/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CRandomFieldGridMap2D_H
#define CRandomFieldGridMap2D_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>

#include <mrpt/maps/link_pragmas.h>
#if EIGEN_VERSION_AT_LEAST(3,1,0) // eigen 3.1+
	#include <Eigen/SparseCore>
	#include <Eigen/SparseCholesky>
#endif

namespace mrpt
{
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRandomFieldGridMap2D , CMetricMap, MAPS_IMPEXP )

	// Pragma defined to ensure no structure packing: since we'll serialize TRandomFieldCell to streams, we want it not to depend on compiler options, etc.
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(push,1)
#endif

	/** The contents of each cell in a CRandomFieldGridMap2D map.
	  * \ingroup mrpt_maps_grp
	 **/
	struct MAPS_IMPEXP TRandomFieldCell
	{
		/** Constructor */
		TRandomFieldCell(double kfmean_dm_mean = 1e-20, double kfstd_dmmeanw = 0) :
			kf_mean      (kfmean_dm_mean),
			kf_std       (kfstd_dmmeanw),
			dmv_var_mean (0),
			last_updated(mrpt::system::now()),
			updated_std (kfstd_dmmeanw)
		{ }

		// *Note*: Use unions to share memory between data fields, since only a set
		//          of the variables will be used for each mapping strategy.
		// You can access to a "TRandomFieldCell *cell" like: cell->kf_mean, cell->kf_std, etc..
		//  but accessing cell->kf_mean would also modify (i.e. ARE the same memory slot) cell->dm_mean, for example.

		// Note 2: If the number of type of fields are changed in the future,
		//   *PLEASE* also update the writeToStream() and readFromStream() methods!!

		union
		{
			double kf_mean;		//!< [KF-methods only] The mean value of this cell
			double dm_mean;		//!< [Kernel-methods only] The cumulative weighted readings of this cell
			double gmrf_mean;	//!< [GMRF only] The mean value of this cell
		};

		union
		{
			double kf_std;    //!< [KF-methods only] The standard deviation value of this cell
			double dm_mean_w; //!< [Kernel-methods only] The cumulative weights (concentration = alpha * dm_mean / dm_mean_w + (1-alpha)*r0 )
			double gmrf_std;
		};

		double dmv_var_mean;   //!< [Kernel DM-V only] The cumulative weighted variance of this cell

		mrpt::system::TTimeStamp last_updated;	//!< [Dynamic maps only] The timestamp of the last time the cell was updated
		double updated_std;			//!< [Dynamic maps only] The std cell value that was updated (to be used in the Forgetting_curve
	};

#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)
#endif

	/** CRandomFieldGridMap2D represents a 2D grid map where each cell is associated one real-valued property which is estimated by this map, either
	  *   as a simple value or as a probility distribution (for each cell).
	  *
	  *  There are a number of methods available to build the gas grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor.
	  *
	  *  The following papers describe the mapping alternatives implemented here:
	  *		- `mrKernelDM`: A kernel-based method. See:
	  *			- "Building gas concentration gridmaps with a mobile robot", Lilienthal, A. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.
	  *		- `mrKernelDMV`: A kernel-based method. See:
	  *			- "A Statistical Approach to Gas Distribution Modelling with Mobile Robots--The Kernel DM+ V Algorithm", Lilienthal, A.J. and Reggente, M. and Trincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.
	  *		- `mrKalmanFilter`: A "brute-force" approach to estimate the entire map with a dense (linear) Kalman filter. Will be very slow for mid or large maps. It's provided just for comparison purposes, not useful in practice.
	  *		- `mrKalmanApproximate`: A compressed/sparse Kalman filter approach. See:
	  *			- "A Kalman Filter Based Approach to Probabilistic Gas Distribution Mapping", JL Blanco, JG Monroy, J Gonzalez-Jimenez, A Lilienthal, 28th Symposium On Applied Computing (SAC), 2013.
	  *		- `mrGMRF_G` and `mrGMRF_SD`: A Gaussian Markov Random Field (GMRF) estimator, with these constraints:
	  *			- `mrGMRF_G`: Each cell connected to a square area of neighbors cells.
	  *			- `mrGMRF_SD`: Each cell only connected to its 4 immediate neighbors (Up, down, left, right).
	  *			- See papers: 
	  *				- "Time-variant gas distribution mapping with obstacle information", Monroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots, 40(1), 1-16, 2016.
	  *
	  *  Note that this class is virtual, since derived classes still have to implement:
	  *		- mrpt::maps::CMetricMap::internal_computeObservationLikelihood()
	  *		- mrpt::maps::CMetricMap::internal_insertObservation()
	  *		- Serialization methods: writeToStream() and readFromStream()
	  *
	  * \sa mrpt::maps::CGasConcentrationGridMap2D, mrpt::maps::CWirelessPowerGridMap2D, mrpt::maps::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::maps::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class CRandomFieldGridMap2D : public mrpt::maps::CMetricMap, public utils::CDynamicGrid<TRandomFieldCell>
	{
		typedef utils::CDynamicGrid<TRandomFieldCell> BASE;

		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CRandomFieldGridMap2D )
	public:

		/** Calls the base CMetricMap::clear
		  * Declared here to avoid ambiguity between the two clear() in both base classes.
		  */
		inline void clear() { CMetricMap::clear(); }

		// This method is just used for the ::saveToTextFile() method in base class.
		float cell2float(const TRandomFieldCell& c) const MRPT_OVERRIDE
		{
			return c.kf_mean;
		}

		/** The type of map representation to be used, see CRandomFieldGridMap2D for a discussion.
		  */
		enum TMapRepresentation
		{
			mrKernelDM = 0,   
			mrAchim = 0,      //!< Another alias for "mrKernelDM", for backwards compatibility
			mrKalmanFilter,
			mrKalmanApproximate,
			mrKernelDMV,
			mrGMRF_G,   //!< Gaussian Markov Random Field, Gaussian prior weights between neighboring cells up to a certain distance
			mrGMRF_SD,  //!< Gaussian Markov Random Field, squared differences prior weights between 4 neighboring cells
		};

		/** Constructor
		  */
		CRandomFieldGridMap2D(
			TMapRepresentation	mapType = mrAchim,
			float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CRandomFieldGridMap2D();

		 /** Returns true if the map is empty/no observation has been inserted (in this class it always return false,
		   * unless redefined otherwise in base classes)
		   */
		virtual bool isEmpty() const MRPT_OVERRIDE;

		/** Save the current map as a graphical file (BMP,PNG,...).
		  * The file format will be derived from the file extension (see  CImage::saveToFile )
		  *  It depends on the map representation model:
		  *		mrAchim: Each pixel is the ratio \f$ \sum{\frac{wR}{w}} \f$
		  *		mrKalmanFilter: Each pixel is the mean value of the Gaussian that represents each cell.
		  *
		  * \sa \a getAsBitmapFile()
		  */
		virtual void  saveAsBitmapFile(const std::string  &filName) const;

		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** Parameters common to any derived class.
		  *  Derived classes should derive a new struct from this one, plus "public utils::CLoadableOptions",
		  *  and call the internal_* methods where appropiate to deal with the variables declared here.
		  *  Derived classes instantions of their "TInsertionOptions" MUST set the pointer "m_insertOptions_common" upon construction.
		  */
		struct MAPS_IMPEXP TInsertionOptionsCommon
		{
			TInsertionOptionsCommon();	//!< Default values loader

			/** See utils::CLoadableOptions */
			void  internal_loadFromConfigFile_common(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  internal_dumpToTextStream_common(mrpt::utils::CStream	&out) const; //!< See utils::CLoadableOptions

			/** @name Kernel methods (mrKernelDM, mrKernelDMV)
			    @{ */
			float	sigma;	//!< The sigma of the "Parzen"-kernel Gaussian
			float	cutoffRadius;	//!< The cutoff radius for updating cells.
            float	R_min,R_max;	//!< Limits for normalization of sensor readings.
			double	dm_sigma_omega;	//!< [DM/DM+V methods] The scaling parameter for the confidence "alpha" values (see the IROS 2009 paper; see CRandomFieldGridMap2D) */
			/** @} */

			/** @name Kalman-filter methods (mrKalmanFilter, mrKalmanApproximate)
			    @{ */
			float	KF_covSigma;	//!< The "sigma" for the initial covariance value between cells (in meters).
			float	KF_initialCellStd;	//!< The initial standard deviation of each cell's concentration (will be stored both at each cell's structure and in the covariance matrix as variances in the diagonal) (in normalized concentration units).
			float	KF_observationModelNoise;	//!< The sensor model noise (in normalized concentration units).
			float	KF_defaultCellMeanValue;	//!< The default value for the mean of cells' concentration.
			uint16_t	KF_W_size;	//!< [mrKalmanApproximate] The size of the window of neighbor cells.
			/** @} */

			/** @name Gaussian Markov Random Fields methods (mrGMRF_G & mrGMRF_SD)
			    @{ */
			float		GMRF_lambdaPrior;		//!< The information (Lambda) of fixed map constraints
			float		GMRF_lambdaObs;			//!< The initial information (Lambda) of each observation (this information will decrease with time)
			float		GMRF_lambdaObsLoss;		//!< The loss of information of the observations with each iteration
			
			bool GMRF_use_occupancy_information;	//!< wether to use information of an occupancy_gridmap map for buidling the GMRF
			std::string GMRF_simplemap_file;		//!< simplemap_file name of the occupancy_gridmap
			std::string GMRF_gridmap_image_file;	//!< image name of the occupancy_gridmap
			float GMRF_gridmap_image_res;			//!< occupancy_gridmap resolution: size of each pixel (m)
			size_t GMRF_gridmap_image_cx;			//!< Pixel coordinates of the origin for the occupancy_gridmap
			size_t GMRF_gridmap_image_cy;			//!< Pixel coordinates of the origin for the occupancy_gridmap

			uint16_t	GMRF_constraintsSize;	//!< [mrGMRF_G only] The size of the Gaussian window to impose fixed restrictions between cells.
			float		GMRF_constraintsSigma;  //!< [mrGMRF_G only] The sigma of the Gaussian window to impose fixed restrictions between cells.
			/** @} */
		};

		/** Changes the size of the grid, maintaining previous contents.
		  * \sa setSize */
		virtual void  resize(float	new_x_min, float new_x_max, float new_y_min, float	new_y_max, const TRandomFieldCell& defaultValueNewCells, float	additionalMarginMeters = 1.0f ) MRPT_OVERRIDE;

		/** See docs in base class: in this class this always returns 0 */
		float  compute3DMatchingRatio(
				const mrpt::maps::CMetricMap						*otherMap,
				const mrpt::poses::CPose3D							&otherMapPose,
				float									maxDistForCorr = 0.10f,
				float									maxMahaDistForCorr = 2.0f
				) const MRPT_OVERRIDE;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps */
		virtual void  saveMetricMapRepresentationToFile(const std::string &filNamePrefix) const MRPT_OVERRIDE;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  *  \sa getAsMatlab3DGraphScript    */
		virtual void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Return a large text block with a MATLAB script to plot the contents of this map \sa saveAsMatlab3DGraph
		  *  This method can only be called in a KF map model  */
		void  getAsMatlab3DGraphScript(std::string  &out_script) const;

		/** Returns a 3D object representing the map (mean) */
		virtual void getAs3DObject( mrpt::opengl::CSetOfObjectsPtr &outObj ) const MRPT_OVERRIDE;

		/** Returns two 3D objects representing the mean and variance maps */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&meanObj, mrpt::opengl::CSetOfObjectsPtr	&varObj ) const;

		TMapRepresentation	 getMapType(); //!< Return the type of the random-field grid map, according to parameters passed on construction.

		/** Direct update of the map with a reading in a given position of the map, using 
		  *  the appropriate method according to mapType passed in the constructor.
		  *
		  * This is a direct way to update the map, an alternative to the generic insertObservation() method which works with mrpt::obs::CObservation objects.
		  */
		void insertIndividualReading(const float sensorReading,const mrpt::math::TPoint2D & point, const bool update_map = true);

		/** Returns the prediction of the measurement at some (x,y) coordinates, and its certainty (in the form of the expected variance).
		  *  This methods is implemented differently for the different gas map types.
		  */
		virtual void predictMeasurement(
			const double	&x,
			const double	&y,
			double			&out_predict_response,
			double			&out_predict_response_variance );

		/** Return the mean and covariance vector of the full Kalman filter estimate (works for all KF-based methods). */
		void getMeanAndCov( mrpt::math::CVectorDouble &out_means, mrpt::math::CMatrixDouble &out_cov) const;

		/** Return the mean and STD vectors of the full Kalman filter estimate (works for all KF-based methods). */
		void getMeanAndSTD( mrpt::math::CVectorDouble &out_means, mrpt::math::CVectorDouble &out_STD) const;

		/** Load the mean and STD vectors of the full Kalman filter estimate (works for all KF-based methods). */
		void setMeanAndSTD( mrpt::math::CVectorDouble &out_means, mrpt::math::CVectorDouble &out_STD);

		void updateMapEstimation();

	protected:
		/** Common options to all random-field grid maps: pointer that is set to the derived-class instance of "insertOptions" upon construction of this class. */
		TInsertionOptionsCommon * m_insertOptions_common;

		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon* getCommonInsertOptions() = 0;

		TMapRepresentation	m_mapType; //!< The map representation type of this map, as passed in the constructor

		mrpt::math::CMatrixD				m_cov;	//!< The whole covariance matrix, used for the Kalman Filter map representation.

		/** The compressed band diagonal matrix for the KF2 implementation.
		  *   The format is a Nx(W^2+2W+1) matrix, one row per cell in the grid map with the
		  *    cross-covariances between each cell and half of the window around it in the grid.
		  */
		mrpt::math::CMatrixD		m_stackedCov;
		mutable bool	m_hasToRecoverMeanAndCov;       //!< Only for the KF2 implementation.

		/** @name Auxiliary vars for DM & DM+V methods
		    @{ */
		float               m_DM_lastCutOff;
		std::vector<float>	m_DM_gaussWindow;
		double				m_average_normreadings_mean, m_average_normreadings_var;
		size_t              m_average_normreadings_count;
		/** @} */

		/** @name Auxiliary vars for GMRF method
		    @{ */
#if EIGEN_VERSION_AT_LEAST(3,1,0)
		std::vector<Eigen::Triplet<double> >  H_prior;	// the prior part of H
#endif
		Eigen::VectorXd g;								// Gradient vector
		size_t nPriorFactors;							// L
		size_t nObsFactors;								// M
		size_t nFactors;								// L+M
		std::multimap<size_t,size_t> cell_interconnections;		//Store the interconnections (relations) of each cell with its neighbourds

		std::vector<float> gauss_val;					// For factor Weigths (only for mrGMRF_G)

		struct TobservationGMRF
		{
			float	obsValue;
			float	Lambda;
			bool	time_invariant;						//if the observation will lose weight (lambda) as time goes on (default false)
		};

		std::vector<std::vector<TobservationGMRF> > activeObs;		//Vector with the active observations and their respective Information


		/** @} */

		/** The implementation of "insertObservation" for Achim Lilienthal's map models DM & DM+V.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param point Is the sensor location on the map
		  * \param is_DMV = false -> map type is Kernel DM; true -> map type is DM+V
		  */
		void  insertObservation_KernelDM_DMV(
			float            normReading,
			const mrpt::math::TPoint2D &point,
			bool             is_DMV );

		/** The implementation of "insertObservation" for the (whole) Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param point Is the sensor location on the map
		  */
		void  insertObservation_KF(
			float			normReading,
			const mrpt::math::TPoint2D &point );

		/** The implementation of "insertObservation" for the Efficient Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param point Is the sensor location on the map
		  */
		void  insertObservation_KF2(
			float			normReading,
			const mrpt::math::TPoint2D &point );

		/** The implementation of "insertObservation" for the Gaussian Markov Random Field map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param point Is the sensor location on the map
		  */
		void  insertObservation_GMRF(float normReading,const mrpt::math::TPoint2D &point, const bool update_map);

		/** solves the minimum quadratic system to determine the new concentration of each cell */
		void  updateMapEstimation_GMRF();

		/** Computes the confidence of the cell concentration (alpha) */
		double computeConfidenceCellValue_DM_DMV (const TRandomFieldCell *cell ) const;

		/** Computes the average cell concentration, or the overall average value if it has never been observed  */
		double computeMeanCellValue_DM_DMV (const TRandomFieldCell *cell ) const;

		/** Computes the estimated variance of the cell concentration, or the overall average variance if it has never been observed  */
		double computeVarCellValue_DM_DMV (const TRandomFieldCell *cell ) const;

		/** In the KF2 implementation, takes the auxiliary matrices and from them update the cells' mean and std values.
		  * \sa m_hasToRecoverMeanAndCov
		  */
		void  recoverMeanAndCov() const;

		/** Erase all the contents of the map */
		virtual void  internal_clear() MRPT_OVERRIDE;

		/** Check if two cells of the gridmap (m_map) are connected, based on the provided occupancy gridmap*/
		bool exist_relation_between2cells(
			const mrpt::maps::COccupancyGridMap2D *m_Ocgridmap,
			size_t cxo_min, 
			size_t cxo_max, 
			size_t cyo_min, 
			size_t cyo_max, 
			const size_t seed_cxo, 
			const size_t seed_cyo, 
			const size_t objective_cxo, 
			const size_t objective_cyo);
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRandomFieldGridMap2D , CMetricMap, MAPS_IMPEXP )


	} // End of namespace


	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<maps::CRandomFieldGridMap2D::TMapRepresentation>
		{
			typedef maps::CRandomFieldGridMap2D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(maps::CRandomFieldGridMap2D::mrKernelDM,          "mrKernelDM");
				m_map.insert(maps::CRandomFieldGridMap2D::mrKalmanFilter,      "mrKalmanFilter");
				m_map.insert(maps::CRandomFieldGridMap2D::mrKalmanApproximate, "mrKalmanApproximate");
				m_map.insert(maps::CRandomFieldGridMap2D::mrKernelDMV,         "mrKernelDMV");
				m_map.insert(maps::CRandomFieldGridMap2D::mrGMRF_G,			   "mrGMRF_G");
				m_map.insert(maps::CRandomFieldGridMap2D::mrGMRF_SD,		   "mrGMRF_SD");
			}
		};
	} // End of namespace
} // End of namespace

#endif
