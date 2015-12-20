/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CVelodyneScanner_H
#define CVelodyneScanner_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A C++ interface to Velodyne laser scanners (HDL-32E, VLP-16), working on Windows and Linux.
		  * 
		  *  <h2>Configuration and usage:</h2> <hr>
		  * Data is returned as observations of type: 
		  *  - mrpt::obs::CObservationVelodyneScan for one or more "data packets" (refer to Velodyne usage manual) 
		  *  - mrpt::obs::CObservationGPS for GPS (GPRMC) packets, if available via the synchronization interface of the device.
		  *  See those classes for documentation on their fields.
		  *
		  * Configuration includes setting the device IP, model and RPM rotation speed (for calculating how many packets form a scan).
		  * These parameters can be set programatically (see methods of this class), or via a configuration file with CGenericSensor::loadConfig()
		  *
		  * <h2>Grabbing live data (as a user)</h2> <hr>
		  *  - Use the application [velodyne-view](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)) to visualize the LIDAR output in real-time.
		  *  - Use [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/) to record a dataset in MRPT's format. See example config file: [MRPT\share\mrpt\config_files\rawlog-grabber\velodyne.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)
		  *
		  * <h2>Grabbing live data (programmatically)</h2> <hr>
		  *  - See CGenericSensor for a general overview of the sequence of methods to be called: loadConfig(), initialize(), doProcess(). 
		  *  - Or use this class inside the application [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/). See example config files:  [MRPT\share\mrpt\config_files\rawlog-grabber\velodyne.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)
		  *
		  * See the source code of the example application `[MRPT]/apps/velodyne-view` ([velodyne-view web page](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)) for more details.
		  *
		  * <h2>Playing back a PCAP file:</h2><hr>
		  *  It is common to save Velodyne datasets as Wireshark's PCAP files. 
		  *  These files can be played back with tools like [bittwist](http://bittwist.sourceforge.net/), which emit all UDP packets in the PCAP log. 
		  *  Then, use this class to receive the packets as if they come from the real sensor.
		  *
		  * <h2>About timestamps:</h2><hr>
		  *  Generated observations timestamp are, by default, set from the computer clock as UDP packets are received.
		  *  *TODO* Set from sensor timestamp.
		  *
		  * <h2>Format of parameters for loading from a .ini file</h2><hr>
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *
		  *   model      = VLP16          // Can be any of: 64E_S2, 64E_S2.1, 64E, 32E, VLP16
		  *   #device_ip  =  // IP address of the device. UDP packets from other IPs will be ignored. Left commented or blank if only one scanner is present (no IP filtering)
		  *   rpm        = 600            // Device spinning speed (this is NOT sent to the device, you must configure it in advance via the LIDAR web interface)
		  *
		  *    # 3D position of the sensor on the vehicle:
		  *   pose_x     = 0      // 3D position (meters)
		  *   pose_y     = 0
		  *   pose_z     = 0
		  *   pose_yaw   = 0    // 3D orientation (degrees)
		  *   pose_pitch = 0
		  *   pose_roll  = 0
		  *
		  *  \endcode
		  *
		  *
		  * <h2>Copyright notice</h2><hr>
		  * Portions of this class are based on code from velodyne ROS node in https://github.com/ros-drivers/velodyne
		  *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
		  *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
		  *  License: Modified BSD Software License Agreement
		  *
		  * \note New in MRPT 1.3.3
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CVelodyneScanner : public mrpt::hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CVelodyneScanner)
		public:
			static short int VELODYNE_DATA_UDP_PORT;  //!< Default: 2368. Change it if required.
			static short int VELODYNE_POSITION_UDP_PORT;  //!< Default: 8308. Change it if required.

		protected:
			std::string   m_model;      //!< Default: "VLP16"
			std::string   m_device_ip;  //!< Default: "" (no IP-based filtering)
			int           m_rpm;        //!< Default: 600
			mrpt::poses::CPose3D m_sensorPose;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

		public:
			CVelodyneScanner( );
			virtual ~CVelodyneScanner();

			/** @name Change configuration parameters; to be called BEFORE initialize(); see above for the list of parameters and their meaning
			  * @{ */
			void setModelName(const std::string & model) { m_model = model; }
			const std::string &getModelName() const { return m_model; }

			void setDeviceIP(const std::string & ip) { m_device_ip = ip; }
			const std::string &getDeviceIP() const { return m_device_ip; }

			void setDeviceRPM(const int rpm) { m_rpm = rpm; }
			int getDeviceRPM() const { return m_rpm; }
			/** @} */

			/** Polls the UDP port for incoming data packets. The user *must* call this method in a timely fashion to grab data as it it generated by the device. 
			  *  The minimum call rate should be the expected number of data packets/second (!=scans/second). Checkout Velodyne user manual if in doubt.
			  *
			  * \param[out] outScan Upon return, an empty smart pointer will be found here if no new data was available. Otherwise, a valid scan.
			  * \param[out] outGPS  Upon return, an empty smart pointer will be found here if no new GPS data was available. Otherwise, a valid GPS reading.
			  * \return true if no error ocurred (even if there was no new observation). false if any communication error occurred.
			  */
			bool getNextObservation(
				mrpt::obs::CObservationVelodyneScanPtr & outScan,
				mrpt::obs::CObservationGPSPtr          & outGPS
				);

			// See docs in parent class
			void  doProcess();

			/** Tries to initialize the sensor driver, after setting all the parameters with a call to loadConfig.
			  * Velodyne specifics: this method sets up the UDP listening sockets, so all relevant params MUST BE SET BEFORE calling this.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

			/** Close the UDP sockets set-up in \a initialize(). This is called automatically upon destruction */
			void close();

			/** Users normally prefer calling \a getNextObservation() instead. 
			  * This method polls the UDP data port and returns one Velodyne packet (1206 bytes). Refer to Velodyne users manual. 
			  * \return The approximate timestamp (based on this computer clock) of the scan reception, or INVALID_TIMESTAMP if timeout ocurred waiting for a packet.
			  */
			mrpt::system::TTimeStamp receiveDataPacket( mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket &out_pkt );

		private:
		/** Handles for the UDP sockets, or INVALID_SOCKET (-1) */
			typedef 
#ifdef MRPT_OS_WINDOWS
#	if MRPT_WORD_SIZE==64
			uint64_t
#	else
			uint32_t
#	endif
#else
			int
#endif
			platform_socket_t;

		platform_socket_t m_hDataSock, m_hPositionSock;

		static mrpt::system::TTimeStamp internal_receive_UDP_packet(platform_socket_t hSocket, uint8_t *out_buffer, const size_t expected_packet_size,const std::string &filter_only_from_IP);

		}; // end of class
	} // end of namespace
} // end of namespace


#endif


