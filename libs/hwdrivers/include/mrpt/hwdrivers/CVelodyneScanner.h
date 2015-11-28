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

#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** An interface to Velodyne laser scanners (HDL-32E, VLP-16)
		  *
		  *  <h2>Configuration and usage:</h2> <hr>
		  * Data is returned as observations of type: 
		  *  - mrpt::obs::CObservationVelodyneScan for one or more "data packets" (refer to Velodyne usage manual) 
		  *  - mrpt::obs::CObservationGPS for GPS (GPRMC) packets, if available via the synchronization interface of the device.
		  *  See those classes for documentation on their fields.
		  *
		  * <h2>Format of parameters for loading from a .ini file</h2><hr>
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
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
		  * \note New in MRPT 1.3.3
		  * \ingroup mrpt_hwdrivers_grp
 		  */
		class HWDRIVERS_IMPEXP CVelodyneScanner : public mrpt::hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CVelodyneScanner)

		protected:
			std::string		m_device_ip;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

		public:
			CVelodyneScanner( );
			virtual ~CVelodyneScanner();

			/**  */
			bool getObservation( mrpt::obs::CObservationVelodyneScan &outObservation );

			// See docs in parent class
			void  doProcess();

			/** Tries to initialize the sensor, after setting all the parameters with a call to loadConfig.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

		}; // end of class
	} // end of namespace
} // end of namespace


#endif


