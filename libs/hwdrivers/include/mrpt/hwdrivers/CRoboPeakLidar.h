/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRoboPeakLidar_H
#define CRoboPeakLidar_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/utils/circular_buffer.h>
#include <mrpt/gui/CDisplayWindow3D.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Interfaces a Robo Peak LIDAR laser scanner. 
		  *
		  *  See the example "samples/RoboPeakLidar_laser_test" and the application "rawlog-grabber" for a ready-to-use application to gather data from the scanner.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    COM_port_WIN = COM3
		  *    COM_port_LIN = ttyS0
		  *    pose_x=0	// Laser range scaner 3D position in the robot (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	// Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *
		  *    // Optional: Exclusion zones to avoid the robot seeing itself:
		  *    //exclusionZone1_x = 0.20 0.30 0.30 0.20
		  *    //exclusionZone1_y = 0.20 0.30 0.30 0.20
		  *
		  *    // Optional: Exclusion zones to avoid the robot seeing itself:
		  *    //exclusionAngles1_ini = 20  // Deg
		  *    //exclusionAngles1_end = 25  // Deg
		  *
		  *  \endcode
		  * \note Class introduced in MRPT 1.2.2
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CRoboPeakLidar : public C2DRangeFinderAbstract
		{
			DEFINE_GENERIC_SENSOR(CRoboPeakLidar)
		public:
			CRoboPeakLidar(); //!< Constructor
			virtual ~CRoboPeakLidar();  //!< Destructor: turns the laser off.

			virtual void initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

			// See base class docs
			virtual void  doProcessSimple(
				bool							&outThereIsObservation,
				CObservation2DRangeScan	&outObservation,
				bool							&hardwareError );

			/** If set to non-empty, the serial port will be attempted to be opened automatically when this class is first used to request data from the laser.  */
			void setSerialPort(const std::string &port_name);
			const std::string getSerialPort() { return m_com_port; }  //!< Returns the currently set serial port \sa setSerialPort

			void showPreview(bool enable=true) { m_showPreview=enable; }

			struct HWDRIVERS_IMPEXP TSensorInfo
			{
				TSensorInfo();
			};

			/** Returns info about the device.
			  * \return false on communication error or initialize() not called yet. true on success.
			  */
			bool getDeviceInfo(TSensorInfo &out_info) const;

			virtual bool  turnOn();  //!< See base class docs
			virtual bool  turnOff(); //!< See base class docs

			/** Returns true if the device is connected & operative */
			bool getDeviceHealth() const;

			void disconnect(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user

		protected:
			poses::CPose3D  m_sensorPose;       //!< The sensor 6D pose:
			std::string     m_com_port;
			int             m_com_port_baudrate;
			void          * m_rplidar_drv;  // Opaque "RPlidarDriver*"
			TSensorInfo     m_sensor_info;  //!< The information gathered when the laser is first open

			bool            m_showPreview; //!< If true, shows a 3D window with a preview of the grabber data
			mrpt::gui::CDisplayWindow3DPtr m_win;

			/** Returns true if communication has been established with the device. If it's not, 
			  *  try to create a comms channel. 
			  * \return false on error.
			  */
			bool  checkCOMMs(); 
			

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		};	// End of class

	} // End of namespace

} // End of namespace

#endif
