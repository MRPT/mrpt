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

		/** A class for grabbing mrpt::obs::CObservationSkeleton from a PrimeSense camera.
		  *  It connects to a PrimeSense camera and tries to detect users while recording the positions of their skeletons' joints along time.
		  *
		  *  See also the application "rawlog-grabber" for a ready-to-use application to gather data from this sensor.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *	   driver			= CSkeletonTracker
		  *	   sensorLabel		= <label>			; Label of the sensor
		  *	   grab_decimation	= 1					; [int] Grab skeletons in 1 out of 'grab_decimation' frames.
		  *    show_preview		= 1					; [bool] {0,1} Opens a display window to show the recorded skeleton.
		  *    pose_x			= 0					; [double] Sensor 3D position relative to the robot (meters)
		  *    pose_y			= 0
		  *    pose_z			= 0
		  *    pose_yaw			= 0					; [double] Angles in degrees
		  *    pose_pitch		= 0
		  *    pose_roll		= 0
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

			/** Opaque pointers to specific NITE data */
			void * /* nite::SkeletonState* */	m_skeletons_ptr;
			void * /* nite::userTracker* */		m_userTracker_ptr;

			/** Timestamp management */
			uint32_t							m_timeStartUI;
			mrpt::system::TTimeStamp			m_timeStartTT;
			
			mrpt::poses::CPose3D				m_sensorPose;				//!< Sensor pose
			int									m_nUsers;					//!< Number of detected users

			/** Preview window management */
			bool									m_showPreview;
			mrpt::gui::CDisplayWindow3DPtr			m_win;
			std::vector< std::pair<JOINT,JOINT> >	m_linesToPlot;			//!< Lines between joints
			std::vector<double>						m_joint_theta;			//!< Joint angles when no skeleton has been detected

			unsigned int						m_toutCounter;				//!< Timeout counter (for internal use only)

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase	& configSource,
				const std::string					& iniSection );

			/** Displays real-time info for the captured skeleton */
			void processPreview(const mrpt::obs::CObservationSkeletonPtr & obs);
			void processPreviewNone();

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

			/** Connects to the PrimeSense camera and prepares it to get skeleton data */
			void initialize();

			/** Set/unset preview */
			inline void setPreview( const bool setPreview = true ) { m_showPreview = setPreview; }
		}; // end of class

	} // end of namespace
} // end of namespace

#endif


