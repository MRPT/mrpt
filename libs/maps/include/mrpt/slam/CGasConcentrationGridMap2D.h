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

#ifndef CGasConcentrationGridMap2D_H
#define CGasConcentrationGridMap2D_H

#include <mrpt/slam/CRandomFieldGridMap2D.h>
#include <mrpt/slam/CObservationGasSensors.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CGasConcentrationGridMap2D , CRandomFieldGridMap2D, MAPS_IMPEXP )

	typedef TRandomFieldCell TGasConcentrationCell;  //!< Defined for backward compatibility only (mrpt <0.9.5)

	/** CGasConcentrationGridMap2D represents a PDF of gas concentrations over a 2D area.
	  *
	  *  There are a number of methods available to build the gas grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor (see base class mrpt::slam::CRandomFieldGridMap2D).
	  *
	  * \sa mrpt::slam::CRandomFieldGridMap2D, mrpt::slam::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::slam::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class MAPS_IMPEXP CGasConcentrationGridMap2D : public CRandomFieldGridMap2D
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CGasConcentrationGridMap2D )
	public:

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

		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		 virtual double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );


		/** Parameters related with inserting observations into the map:
		  */
		struct MAPS_IMPEXP TInsertionOptions :
			public utils::CLoadableOptions,
			public TInsertionOptionsCommon
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
			bool save_maplog;	//!< If true save generated gas map as a log file
			/** @} */ // end: Parameters of the "MOS model"

		} insertionOptions;


		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		virtual void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  */
		virtual void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Returns a 3D object representing the map.
		  */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

	protected:

		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon * getCommonInsertOptions() {
			return &insertionOptions;
		}

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


		 /** Erase all the contents of the map */
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
