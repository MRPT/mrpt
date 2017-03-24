/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationGasSensors_H
#define CObservationGasSensors_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationGasSensors , CObservation, OBS_IMPEXP)

	/** Declares a class derived from "CObservation" that represents a set of readings from gas sensors.
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationGasSensors : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationGasSensors )

	 public:
		/** Constructor.
		 */
		CObservationGasSensors(  );

		 /** The structure for each e-nose
		  */
		struct OBS_IMPEXP TObservationENose
		{
			TObservationENose() :
				eNosePoseOnTheRobot(),
				readingsVoltage(),
				sensorTypes(),
				hasTemperature(false),
				temperature()
			{}

			math::TPose3D      eNosePoseOnTheRobot;//!< The pose of the sensors on the robot
			std::vector<float> readingsVoltage;//!< The set of readings (in volts) from the array of sensors (size of "sensorTypes" is the same that the size of "readingsVoltage")

			/** The kind of sensors in the array (size of "sensorTypes" is the same that the size of "readingsVoltage")
			  *  The meaning of values for types of sensors is as follows:
			  *	 0x0000 : No sensor installed in this slot
			  *  0x2600 : Figaro TGS 2600
			  *  0x2602 : Figaro TGS 2602
			  *  0x2620 : Figaro TGS 2620
			  *  0x4161 : Figaro TGS 4161
			  */
			vector_int sensorTypes;
			bool       hasTemperature;//!< Must be true for "temperature" to contain a valid measurement
			float      temperature;//!< Sensed temperature in Celcius (valid if hasTemperature=true only)
			bool       isActive;//!< True if the input to this chamber/enose is poluted air, False if clean air
		};

		/** One entry per e-nose on the robot */
		std::vector<TObservationENose>		m_readings;

		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE;
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE;
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

					/** Declares a class within "CObservationGasSensors" that represents a set of gas concentration readings from the modelation of a MOS gas sensor readings.
					 * This class provides the parameters and functions to simulate the inverse model of a MOS gas sensor.
					 *
					 * \sa CObservationGasSensors
					 */
					class OBS_IMPEXP CMOSmodel
					{
					public:
						/** Constructor  */
						CMOSmodel();
						~CMOSmodel();

						/** @name MOS-model parameters
						  *  @{  */
						size_t        winNoise_size;              //!< The size of the mobile average window used to reduce noise on sensor reagings.
						int           decimate_value;             //!< [useMOSmodel] The decimate frecuency applied after noise filtering
						
						float         a_rise;                     //!< tau = a*AMPLITUDE +b (linear relationship)
						float         b_rise;                     //!< tau = a*AMPLITUDE +b (linear relationship)
						float         a_decay;                    //!< tau = a*AMPLITUDE +b (linear relationship)
						float         b_decay;                    //!< tau = a*AMPLITUDE +b (linear relationship)
												
						bool          save_maplog;                //!< If true save generated gas map as a log file

						/** @} */

						/** Obtain an estimation of the gas distribution based on raw sensor readings  */
						bool get_GasDistribution_estimation(
							float							&reading,							
							mrpt::system::TTimeStamp		&timestamp );

					protected:

						/** The content of each m_lastObservations in the estimation when using the option : MOS_MODEl (useMOSmodel =1)
							*/
						struct OBS_IMPEXP TdataMap
						{
								float						reading;			//!< Sensore reading
								mrpt::system::TTimeStamp	timestamp;			//!< Timestamp of the observation
								float						tau;				//!< tau value applied								
								float						estimation;			//!< The value estimated according to the MOXmodel
								float						reading_filtered;	//!< Reading after smooth (noise averaging)								
						};

						TdataMap              last_Obs, temporal_Obs; //!< The content of each m_lastObservations in the estimation when using the option : MOS_MODEl (useMOSmodel =1)
						std::vector<TdataMap> m_antiNoise_window;  //!< Vector to temporally store and averge readings to reduce noise
						std::ofstream        *m_debug_dump;     //!< Ofstream to save to file option "save_maplog"
						uint16_t              decimate_count; //!< Decimate value for oversampled enose readings
						double                fixed_incT;  //!< To force e-nose samples to have fixed time increments
						bool                  first_incT;  //!< To force e-nose samples to have fixed time increments
						float                 min_reading;	//!< Minimum reading value till the moment, used as approximation to baeline level
						bool                  first_iteration;  //!< To avoid the model estimation on first iteration

						/** Estimates the gas concentration based on readings and sensor model
						  */
						void inverse_MOSmodeling (
							const float						&reading,							
							const mrpt::system::TTimeStamp	&timestamp);

						/** Reduce noise by averaging with a mobile window of specific size (winNoise_size)
						  */
						void noise_filtering (
							const float						&reading,							
							const mrpt::system::TTimeStamp	&timestamp );

						/** Save the gas distribution estiamtion into a log file for offline representation
						*/
						void save_log_map(
							const mrpt::system::TTimeStamp	&timestamp,
							const float						&reading,
							const float						&estimation,
							const float						&tau);

					}; //End of CMOSmodel class def.

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationGasSensors , CObservation, OBS_IMPEXP)

	} // End of namespace
} // End of namespace

#endif
