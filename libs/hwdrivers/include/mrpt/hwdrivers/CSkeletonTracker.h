/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CSkeletonTracker_H
#define CSkeletonTracker_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CObservationSkeleton.h>

namespace mrpt
{
	namespace hwdrivers
	{

		/** A class for interfacing Intersense Inertial Measuring Units (IMUs).
		  *  It uses a serial port or USB-to-serial adapter to communicate to the device, so no special drivers are needed.
		  *  For the more recent 4th generation devices, see the class mrpt::hwdrivers::CIMUXSens_MT4
		  *
		  *  See also the application "rawlog-grabber" for a ready-to-use application to gather data from the scanner.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    pose_x=0	    ; Sensor 3D position relative to the robot (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	; Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *	   sensorLabel = <label> ; Label of the sensor
		  *	   sensitivity	= 10 ; Sensor sensitivity (see dll documentation)
		  *	   enhancement	= 2  ; Enhancement mode (see dll documentation)
		  *	   prediction = 0    ; Prediction mode (see dll documentation)
		  *						                  
		  *  \endcode
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CSkeletonTracker : public hwdrivers::CGenericSensor
		{

enum JOINT {HEAD = 0, NECK, TORSO, 
		LEFT_SHOULDER, LEFT_ELBOW, LEFT_HAND, LEFT_HIP, LEFT_KNEE, LEFT_FOOT, 
		RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_HAND, RIGHT_HIP, RIGHT_KNEE, RIGHT_FOOT, NONE};

#define NUM_JOINTS 15	// number of joints
#define NUM_LINES 14	// number of lines joining joints

			DEFINE_GENERIC_SENSOR(CSkeletonTracker)
		protected:

			void * /* nite::SkeletonState* */	m_skeletons_ptr;
			void * /* nite::userTracker* */		m_userTracker_ptr;

			/** This serial port will be attempted to be opened automatically when this class is first used to request data from the device.
			  * \sa hwdrivers::CSerialPort
			  */
			uint32_t					m_timeStartUI;
			mrpt::system::TTimeStamp	m_timeStartTT;
			mrpt::poses::CPose3D		m_sensorPose;
			int							m_nUsers;

			bool						m_showPreview;
			
			mrpt::gui::CDisplayWindow3DPtr m_win;

			// config

			/** Search the port where the sensor is located and connect to it
			  */
			// bool	searchPortAndConnect();

			unsigned int				m_toutCounter;				//!< Timeout counter (for internal use only)

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase	& configSource,
				const std::string					& iniSection );

		public:
			/** Constructor
			  */
			CSkeletonTracker( );

			/** Destructor
			  */
			virtual ~CSkeletonTracker();

			/** This method will be invoked at a minimum rate of "process_rate" (Hz)
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			void doProcess();

			/** Turns on the xSens device and configure it for getting orientation data */
			void initialize();

			/** Displays real-time info for the captured skeleton */
			void processPreview(const mrpt::obs::CObservationSkeletonPtr & obs);
			void processPreviewNone();

		private:
			std::vector< std::pair<JOINT,JOINT> > m_linesToPlot;
			std::vector<double> m_joint_theta;

		}; // end of class

	} // end of namespace
} // end of namespace

#endif


