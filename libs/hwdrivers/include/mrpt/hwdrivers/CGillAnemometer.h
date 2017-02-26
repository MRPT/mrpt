/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CGillAnemometer_H
#define  CGillAnemometer_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements a driver for the Gill Windsonic Option 1 Anemometer
		  *   The sensor is accessed via a standard serial port.
		  *
		  *   Refer to the manufacturer website for details on this sensor: http://gillinstruments.com/data/manuals/WindSonic-Web-Manual.pdf
		  *	  Configure for single <CR> return, at 2Hz
		  *  \sa mrpt::obs::CObservationWindSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CGillAnemometer : public mrpt::hwdrivers::CGenericSensor
		{
				DEFINE_GENERIC_SENSOR(CGillAnemometer)

		private:
			/** COM port name
			 */
			std::string		com_port;
			int				com_bauds;

			/** COM port
			 */
			CSerialPort COM;

			/** Poses
			 */
			float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;

			/** Returns true if the COM port is already open, or try to open it in other case.
			  * \return true if everything goes OK, or false if there are problems opening the port.
			  */
			bool  tryToOpenTheCOM();

		public:
			/** Default constructor.
			 */
			CGillAnemometer();
			/** Default destructor.
			 */
			virtual ~CGillAnemometer(){COM.close();};
			
			void doProcess();

			void  loadConfig_sensorSpecific(
				 const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section);
		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
