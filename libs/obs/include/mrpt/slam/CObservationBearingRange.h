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
#ifndef CObservationBearingRange_H
#define CObservationBearingRange_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationBearingRange , CObservation, OBS_IMPEXP)

	/** This observation represents a number of range-bearing value pairs, each one for a detected landmark, which optionally can have identification IDs.
	 *  This class can manage sensors that detect landmarks in a 2D plane (e.g. a laser scanner) or in the 3D space (e.g. a camera). There are
	 *  two direction angles: yaw (azimuth) and pitch (negative elevation). For 2D sensors, the pitch must be always set to 0.
	 * See CObservationBearingRange::validCovariances for the instructions to fill the uncertainty covariances.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationBearingRange : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationBearingRange )

	 public:
		/** Default constructor.
		 */
		CObservationBearingRange( );

		float minSensorDistance, maxSensorDistance;  //! Information about the sensor: Ranges, in meters (0: there is no limits)
		float fieldOfView_yaw;		//!< Information about the sensor: The "field-of-view" of the sensor, in radians (for yaw ).
		float fieldOfView_pitch;	//!< Information about the sensor: The "field-of-view" of the sensor, in radians (for pitch ).

		/** The position of the sensor on the robot.
		  */
		CPose3D		sensorLocationOnRobot;

		/** Each one of the measurements:
			*/
		struct OBS_IMPEXP TMeasurement
		{
			/** The sensed landmark distance, in meters. */
			float	range;

			/** The sensed landmark direction, in radians, measured as the yaw (azimuth) and pitch (negative elevation).
			  *  Set pitch to zero for 2D sensors.
			  * See mrpt::poses::CPose3D for a definition of the 3D angles.
			  */
			float	yaw,pitch;

			/** The ID of the sensed beacon, or INVALID_LANDMARK_ID (-1) if the sensor does not identify the landmark. */
			int32_t	landmarkID;

			/** The covariance matrix of the landmark, with variable indices [0,1,2] being [range,yaw,pitch]. */
			CMatrixDouble33 covariance;
		};

		typedef std::vector<TMeasurement> TMeasurementList;

		/** The list of observed ranges:  */
		TMeasurementList sensedData;

		/** True: The individual 3x3 covariance matrices must be taken into account, false (default): All the measurements have identical, diagonal 3x3 covariance matrices given by the values sensor_std_range,sensor_std_yaw,sensor_std_pitch.
		  */
		bool validCovariances;

		/** Taken into account only if validCovariances=false: the standard deviation of the sensor noise model for range,yaw and pitch (in meters and radians).
		  *  If validCovariances=true, these 3 values are ignored and the individual 3x3 covariance matrices contain the actual uncertainties for each of the detected landmarks.
		  */
		float sensor_std_range,sensor_std_yaw,sensor_std_pitch;

		 /** Prints out the contents of the object.
		   */
		 void  debugPrintOut();

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = sensorLocationOnRobot; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { sensorLocationOnRobot = newSensorPose; }


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
