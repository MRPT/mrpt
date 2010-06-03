/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef CGasConcentrationGridMap2D_H
#define CGasConcentrationGridMap2D_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/slam/CObservationGasSensors.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace poses { class CPose2D; }

namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;
	class CObservation;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGasConcentrationGridMap2D , CMetricMap, MAPS_IMPEXP )

	/** The contents of each cell in a CGasConcentrationGridMap2D map.
	 **/
	struct MAPS_IMPEXP TGasConcentrationCell
	{
		/** Constructor
		  */
		TGasConcentrationCell(
			float Mean = 0.5f,
			float Std = 0,
			float W = 1e-20f,
			float Wr = 0 ) : mean(Mean), std(Std),w(W),wr(Wr)
		{
		}

		/** The mean and std values of the gas concentration for this cell.
		  */
		float	mean, std;

		/** The accumulated weight and weighted-readings (See Achim's references)
		  */
		float	w,wr;
	};

	/** The content of each m_lastObservations in KF3_deconv estimation
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

	/** CGasConcentrationGridMap2D represents a PDF of gas concentrations over a 2D area.
	 **/
	class MAPS_IMPEXP CGasConcentrationGridMap2D : public CMetricMap, public utils::CDynamicGrid<TGasConcentrationCell>
	{
		typedef utils::CDynamicGrid<TGasConcentrationCell> BASE;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CGasConcentrationGridMap2D )
	public:

		/** Calls the base CMetricMap::clear
		  * Declared here to avoid ambiguity between the two clear() in both base classes.
		  */
		inline void clear() { CMetricMap::clear(); }

		float cell2float(const TGasConcentrationCell& c) const
		{
			return c.mean;
		}


		/** The type of map representation to be used.
		  */
		enum TMapRepresentation
		{
			mrAchim = 0,
			mrKalmanFilter,
			mrKalmanApproximate,
			mrKalmanApproximate_deconv
		};


		/** Constructor
		  */
		CGasConcentrationGridMap2D(
			TMapRepresentation	mapType = mrAchim,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1

			);

		/** Destructor */
		virtual ~CGasConcentrationGridMap2D();

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
			/** Default values loader:
			  */
			TInsertionOptions();

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

			/** The sigma of the "Parzen"-kernel Gaussian
			  */
			float	sigma;

			/** The cutoff radius for updating cells.
			  */
			float	cutoffRadius;

			/** Limits for normalization of sensor readings.
			  */
            float	R_min,R_max;

			/** [KF model] The "sigma" for the initial covariance value between cells (in meters).
			  */
			float	KF_covSigma;

			/** [KF model] The initial standard deviation of each cell's concentration (will be stored both at each cell's structure and in the covariance matrix as variances in the diagonal) (in normalized concentration units).
			  */
			float	KF_initialCellStd;

			/** [KF model] The sensor model noise (in normalized concentration units).
			  */
			float	KF_observationModelNoise;

			/** [KF model] The default value for the mean of cells' concentration.
			  */
			float	KF_defaultCellMeanValue;

			/** [KF2 algorithm] The size of the window of neighbor cells. */
			uint16_t	KF_W_size;

			/** [KF3 algortihm] The sensor type for the gas concentration map (0x0000 ->mean of all installed sensors, 0x2600, 0x6810, ...)
			  */
			uint16_t	KF_sensorType;

			/** Tau values for the rise (tauR) and decay (tauD) sensor's phases.
			*/
			float	tauR;
			float	tauD;

			/** [KF3 algortihm] The last N GasObservations, used for the Kalman Filter with deconvolution estimation.
			  */
			TdataMap new_Obs, new_ANS;
			std::vector<TdataMap> m_lastObservations;
			std::vector<TdataMap> antiNoise_window;

			/** [KF3 algortihm] The number of observations to keep in m_lastObservations
			  */
			uint16_t lastObservations_size;

			/** [KF3 algortihm] The number of observations used to reduce noise on signal.
			  */
			uint16_t	winNoise_size;
			
			/** [KF3 algortihm] The decimate frecuency applied after noise filtering
			  */
			uint16_t	decimate_value;			

			/** [KF3 algortihm] Measured values of K= 1/tauD for different volatile concentrations
			  */
			vector_float tauD_concentration;
			vector_float tauD_value;

			/** [KF3 algortihm] Measured values of the delay (memory effect) for different robot speeds
			  */
			vector_float memory_speed;
			vector_float memory_delay;

			/** [KF3 algortihm] id for the enose used to generate this map (must be < gasGrid_count)
			  */
			uint16_t enose_id;

			/** [KF3 algortihm] true / 1= Save generated gas map as a log file
			  */
			bool save_maplog;


		} insertionOptions;

		/** Changes the size of the grid, maintaining previous contents.
		  * \sa setSize
		  */
		void  resize(		float	new_x_min,
									float	new_x_max,
									float	new_y_min,
									float	new_y_max,
									const TGasConcentrationCell& defaultValueNewCells,
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

		/** This method is called at the end of each "prediction-update-map insertion" cycle within "mrpt::slam::CMetricMapBuilderRBPF::processActionObservation".
		  *  This method should normally do nothing, but in some cases can be used to free auxiliary cached variables.
		  */
		void  auxParticleFilterCleanUp();

		/** Return the type of the gas distribution map, according to parameters passed on construction.
		  */
		TMapRepresentation	 getMapType();

		/** Returns the prediction of the measurement at some (x,y) coordinates, and its certainty (in the form of the expected variance).
		  *  This methods is implemented differently for the different gas map types.
		  */
		void predictMeasurement(
			const double	&x,
			const double	&y,
			double			&out_predict_response,
			double			&out_predict_response_variance );

		/** Return the mean and covariance vector of the full Kalman filter estimate (works for all KF-based methods). */
		void getMeanAndCov( vector_double &out_means, CMatrixDouble &out_cov) const;

	protected:

		/** [KF3] Ofstream to save to file option "save_maplog"
		  */
		std::ofstream			*m_debug_dump;
		
		/** [KF3] Decimate value for oversampled enose readings
		  */
		uint16_t				decimate_count;

		/** [KF3] To force e-nose samples to have fixed time increments
		  */
		double					fixed_incT;
		bool					first_incT;

		/** The map representation type of this map.
		  */
		TMapRepresentation		m_mapType;

		/** The whole covariance matrix, used for the Kalman Filter map representation.
		  */
		CMatrixD				m_cov;


		/** The implementation of "insertObservation" for the Achim's map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_Achim(
			float			normReading,
			const CPose3D	&sensorPose );

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

		/** The implementation of "insertObservation" for the Efficient Kalman Filter map model with estimation of real gas concentration using sensor model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_KF3(
			float			normReading,
			CPose3D	&sensorPose,
			mrpt::system::TTimeStamp	timestamp);

		/** Estimates the gas concentration based on readings and sensor model
		  */
		void CGasConcentration_estimation (
			float							reading,
			const CPose3D					&sensorPose,
			const mrpt::system::TTimeStamp	timestamp);

		/** Reduce noise by averaging with a mobile window
		  */
		void noise_filtering (
			float	reading,
			const	CPose3D	&sensorPose,
			const	mrpt::system::TTimeStamp timestamp );

		/** Save the GAS_MAP generated into a log file for offline representation
		  */
		void save_log_map(
			const mrpt::system::TTimeStamp	timestamp,
			const float						reading,
			const float						estimation,
			const float						k,
			const double					yaw,
			const float						speed);			

		/** Computes the average cell concentration, or 0 if it has never been observed:
		  */
		static float computeAchimCellValue (const TGasConcentrationCell *cell );


		/** The compressed band diagonal matrix for the KF2 implementation.
		  *   The format is a Nx(W^2+2W+1) matrix, one row per cell in the grid map with the
		  *    cross-covariances between each cell and half of the window around it in the grid.
		  */
		CMatrixD		m_stackedCov;

		mutable bool	m_hasToRecoverMeanAndCov;       //!< Only for the KF2 implementation.

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
} // End of namespace

#endif
