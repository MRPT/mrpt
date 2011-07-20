/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef CWirelessPowerGridMap2D_H
#define CWirelessPowerGridMap2D_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CObservationWirelessPower.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CWirelessPowerGridMap2D , CMetricMap, MAPS_IMPEXP )

	// Pragma defined to ensure no structure packing: since we'll serialize TWirelessPowerCell to streams, we want it not to depend on compiler options, etc.
#pragma pack(push,1)

	/** The contents of each cell in a CWirelessPowerGridMap2D map.
	 **/
	struct MAPS_IMPEXP TWirelessPowerCell
	{
		/** Constructor */
		TWirelessPowerCell(double kfmean_dm_mean = 1e-20, double kfstd_dmmeanw = 0) :
			kf_mean      (kfmean_dm_mean),
			kf_std       (kfstd_dmmeanw),
			dmv_var_mean (0)
		{ }

		// *Note*: Use unions to share memory between data fields, since only a set
		//          of the variables will be used for each mapping strategy.
		// You can access to a "TWirelessPowerCell *cell" like: cell->kf_mean, cell->kf_std, etc..
		//  but accessing cell->kf_mean would also modify (i.e. ARE the same memory slot) cell->dm_mean, for example.

		// Note 2: If the number of type of fields are changed in the future,
		//   *PLEASE* also update the writeToStream() and readFromStream() methods!!

		union
		{
			double kf_mean; //!< [KF-methods only] The mean value of this cell
			double dm_mean; //!< [Kernel-methods only] The cumulative weighted readings of this cell

		};
		union
		{
			double kf_std;    //!< [KF-methods only] The standard deviation value of this cell
			double dm_mean_w; //!< [Kernel-methods only] The cumulative weights (concentration = alpha * dm_mean / dm_mean_w + (1-alpha)*r0 )
		};

		double dmv_var_mean;   //!< [Kernel DM-V only] The cumulative weighted variance of this cell
	};
#pragma pack(pop)

	/** CWirelessPowerGridMap2D represents a PDF of wifi concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the wifi grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor.
	  *
	  *  The following papers describe the mapping alternatives implemented here:
	  *		- mrKernelDM: A kernel-based method:
	  *		"Building gas concentration gridmaps with a mobile robot", Lilienthal, A. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.
	  *
	  *		- mrKernelDMV: A kernel-based method:
	  *		"A Statistical Approach to Gas Distribution Modelling with Mobile Robots--The Kernel DM+ V Algorithm"
	  * 	  , Lilienthal, A.J. and Reggente, M. and Trincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.
	  *
	  * \sa mrpt::slam::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::slam::CMultiMetricMap
	  */
	class MAPS_IMPEXP CWirelessPowerGridMap2D : public CMetricMap, public utils::CDynamicGrid<TWirelessPowerCell>
	{
		typedef utils::CDynamicGrid<TWirelessPowerCell> BASE;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CWirelessPowerGridMap2D )
	public:

		/** Calls the base CMetricMap::clear
		  * Declared here to avoid ambiguity between the two clear() in both base classes.
		  */
		inline void clear() { CMetricMap::clear(); }

		// This method is just used for the ::saveToTextFile() method in base class.
		float cell2float(const TWirelessPowerCell& c) const
		{
			return c.kf_mean;
		}

		/** The type of map representation to be used.
		  */
		enum TMapRepresentation
		{
			mrKernelDM = 0,   //
			mrAchim = 0,      // Another alias for "mrKernelDM", for backward compatibility
			mrKalmanFilter,
			mrKalmanApproximate,
			mrKernelDMV
		};

		/** Constructor
		  */
		CWirelessPowerGridMap2D(
			TMapRepresentation	mapType = mrAchim,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CWirelessPowerGridMap2D();

		 /** Returns true if the map is empty/no observation has been inserted.
		   */
		 bool  isEmpty() const;


		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		 double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

		/** Save the current map as a graphical file (BMP,PNG,...).
		  * The file format will be derived from the file extension (see  CImage::saveToFile )
		  *  It depends on the map representation model:
		  *		mrAchim: Each pixel is the ratio \f$ \sum{\frac{wR}{w}} \f$
		  *		mrKalmanFilter: Each pixel is the mean value of the Gaussian that represents each cell.
		  *		mrInformationFilter:  Id.
		  */
		void  saveAsBitmapFile(const std::string  &filName) const;


		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		{
			TInsertionOptions();	//!< Default values loader

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  dumpToTextStream(CStream	&out) const; //!< See utils::CLoadableOptions

			/** @name For all mapping methods
			    @{ */
			uint16_t sensorType;	//!< The sensor type for the gas concentration map (0x0000 ->mean of all installed sensors, 0x2600, 0x6810, ...)
			/** @} */

			/** @name Kernel methods (mrKernelDM, mrKernelDMV)
			    @{ */
			float	sigma;	//!< The sigma of the "Parzen"-kernel Gaussian
			float	cutoffRadius;	//!< The cutoff radius for updating cells.
            float	R_min,R_max;	//!< Limits for normalization of sensor readings.
			double	dm_sigma_omega;	//!< [DM/DM+V methods] The scaling parameter for the confidence "alpha" values (see the IROS 2009 paper; see CWirelessPowerGridMap2D) */
			/** @} */

			/** @name Kalman-filter methods (mrKalmanFilter, mrKalmanApproximate)
			    @{ */
			float	KF_covSigma;	//!< The "sigma" for the initial covariance value between cells (in meters).
			float	KF_initialCellStd;	//!< The initial standard deviation of each cell's concentration (will be stored both at each cell's structure and in the covariance matrix as variances in the diagonal) (in normalized concentration units).
			float	KF_observationModelNoise;	//!< The sensor model noise (in normalized concentration units).
			float	KF_defaultCellMeanValue;	//!< The default value for the mean of cells' concentration.
			uint16_t	KF_W_size;	//!< [mrKalmanApproximate] The size of the window of neighbor cells.
			/** @} */

			/** @name Parameters of the "MOS model"
			    @{ */
			bool useMOSmodel;	//!< If true use MOS model before map algorithm

			float	tauR;	//!< Tau values for the rise sensor's phases.
			float	tauD;	//!< Tau values for the decay (tauD) sensor's phases.

			uint16_t lastObservations_size;	//!< The number of observations to keep in m_lastObservations
			size_t	winNoise_size;	//!< The number of observations used to reduce noise on signal.
			uint16_t	decimate_value;	//!< The decimate frecuency applied after noise filtering

			vector_float calibrated_tauD_voltages;	//!< Measured values of K= 1/tauD for different volatile concentrations
			vector_float calibrated_tauD_values;

			vector_float calibrated_delay_RobotSpeeds;	//!< Measured values of the delay (memory effect) for different robot speeds
			vector_float calibrated_delay_values;

			uint16_t enose_id;	//!< id for the enose used to generate this map (must be < gasGrid_count)
			bool save_maplog;	//!< If true save generated wifi map as a log file
			/** @} */ // end: Parameters of the "MOS model"

		} insertionOptions;

		/** Changes the size of the grid, maintaining previous contents.
		  * \sa setSize
		  */
		void  resize(		float	new_x_min,
									float	new_x_max,
									float	new_y_min,
									float	new_y_max,
									const TWirelessPowerCell& defaultValueNewCells,
									float	additionalMarginMeters = 1.0f );

		/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
		 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
		 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
		 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
		 *
		 * \return The matching ratio [0,1]
		 * \sa computeMatchingWith2D
		 */
		float  compute3DMatchingRatio(
				const CMetricMap						*otherMap,
				const CPose3D							&otherMapPose,
				float									minDistForCorr = 0.10f,
				float									minMahaDistForCorr = 2.0f
				) const;


		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  */
		void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Returns a 3D object representing the map.
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Return the type of the wifi distribution map, according to parameters passed on construction.
		  */
		TMapRepresentation	 getMapType();

		/** Returns the prediction of the measurement at some (x,y) coordinates, and its certainty (in the form of the expected variance).
		  *  This methods is implemented differently for the different wifi map types.
		  */
		void predictMeasurement(
			const double	&x,
			const double	&y,
			double			&out_predict_response,
			double			&out_predict_response_variance );

		/** Return the mean and covariance vector of the full Kalman filter estimate (works for all KF-based methods). */
		void getMeanAndCov( vector_double &out_means, CMatrixDouble &out_cov) const;

	protected:

		/** The content of each m_lastObservations in the estimation when using the option : MOS_MODEl (insertionOptions.useMOSmodel =1)
			*/
		struct MAPS_IMPEXP TdataMap
		{
				float						reading;
				mrpt::system::TTimeStamp	timestamp;
				float						k;
				CPose3D						sensorPose;
				float						estimation;
				float						reading_filtered;
				float						speed;
		};

		/** [useMOSmodel] The last N GasObservations, used for the MOS MODEL estimation. */
		TdataMap m_new_Obs, m_new_ANS;
		std::vector<TdataMap> m_lastObservations;
		std::vector<TdataMap> m_antiNoise_window;

		/** [useMOSmodel] Ofstream to save to file option "save_maplog"
		  */
		std::ofstream			*m_debug_dump;

		/** [useMOSmodel] Decimate value for oversampled enose readings
		  */
		uint16_t				decimate_count;

		/** [useMOSmodel] To force e-nose samples to have fixed time increments
		  */
		double					fixed_incT;
		bool					first_incT;

		/** The map representation type of this map, as passed in the constructor */
		TMapRepresentation	m_mapType;

		CMatrixD				m_cov;	//!< The whole covariance matrix, used for the Kalman Filter map representation.

		/** The compressed band diagonal matrix for the KF2 implementation.
		  *   The format is a Nx(W^2+2W+1) matrix, one row per cell in the grid map with the
		  *    cross-covariances between each cell and half of the window around it in the grid.
		  */
		CMatrixD		m_stackedCov;

		mutable bool	m_hasToRecoverMeanAndCov;       //!< Only for the KF2 implementation.

		/** @name Auxiliary vars for DM & DM+V methods
		    @{ */
		float               m_DM_lastCutOff;
		std::vector<float>	m_DM_gaussWindow;
		double				m_average_normreadings_mean, m_average_normreadings_var;
		size_t              m_average_normreadings_count;
		/** @} */

		/** The implementation of "insertObservation" for Achim Lilienthal's map models DM & DM+V.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose on the robot
		  * \param is_DMV = false -> map type is Kernel DM; true -> map type is DM+V
		  */
		void  insertObservation_KernelDM_DMV(
			float            normReading,
			const CPose3D   &sensorPose,
			bool             is_DMV );

		/** The implementation of "insertObservation" for the (whole) Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_KF(
			float			normReading,
			const CPose3D	&sensorPose );

		/** The implementation of "insertObservation" for the Efficient Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_KF2(
			float			normReading,
			const CPose3D	&sensorPose );


		/** Estimates the gas concentration based on readings and sensor model
		  */
		void CWirelessPower_estimation (
			float							reading,
			const CPose3D					&sensorPose,
			const mrpt::system::TTimeStamp	timestamp);

		/** Reduce noise by averaging with a mobile window
		  */
		void noise_filtering (
			float	reading,
			const	CPose3D	&sensorPose,
			const	mrpt::system::TTimeStamp timestamp );

		/** Save the WIFI_MAP generated into a log file for offline representation
		  */
		void save_log_map(
			const mrpt::system::TTimeStamp	timestamp,
			const float						reading,
			const float						estimation,
			const float						k,
			const double					yaw,
			const float						speed);

		/** Computes the average cell concentration, or the overall average value if it has never been observed  */
		double computeMeanCellValue_DM_DMV (const TWirelessPowerCell *cell ) const;

		/** Computes the estimated variance of the cell concentration, or the overall average variance if it has never been observed  */
		double computeVarCellValue_DM_DMV (const TWirelessPowerCell *cell ) const;


		/** In the KF2 implementation, takes the auxiliary matrices and from them update the cells' mean and std values.
		  * \sa m_hasToRecoverMeanAndCov
		  */
		void  recoverMeanAndCov() const;

		 /** Erase all the contents of the map
		  */
		 virtual void  internal_clear();

		 /** Insert the observation information into this map. This method must be implemented
		  *    in derived classes.
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  * \sa CObservation::insertObservationInto
		  */
		 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );


	};


	} // End of namespace


	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::CWirelessPowerGridMap2D::TMapRepresentation>
		{
			typedef slam::CWirelessPowerGridMap2D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CWirelessPowerGridMap2D::mrKernelDM,          "mrKernelDM");
				m_map.insert(slam::CWirelessPowerGridMap2D::mrKalmanFilter,      "mrKalmanFilter");
				m_map.insert(slam::CWirelessPowerGridMap2D::mrKalmanApproximate, "mrKalmanApproximate");
				m_map.insert(slam::CWirelessPowerGridMap2D::mrKernelDMV,         "mrKernelDMV");
			}
		};
	} // End of namespace
} // End of namespace

#endif
