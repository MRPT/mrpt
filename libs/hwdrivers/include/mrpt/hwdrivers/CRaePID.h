/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CRaePID_H
#define  CRaePID_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements a driver for the RAE Systems gas PhotoIonization Detector (PID) (Tested on a MiniRAE Lite)
		  *   The sensor is accessed via a standard (or USB) serial port.
		  *
		  *   Refer to the manufacturer website for details on this sensor: http://www.raesystems.com/products/minirae-lite
		  *
		  *  \sa mrpt::obs::CObservationGasSensors
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CRaePID : public mrpt::hwdrivers::CGenericSensor
		{
				DEFINE_GENERIC_SENSOR(CRaePID)

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
			CRaePID();
			/** Default destructor.
			 */
			virtual ~CRaePID(){COM.close();};
			
			void doProcess();

			void  loadConfig_sensorSpecific(
				 const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section);

			 /** Get firmware version string.
			  */
			 std::string getFirmware();

			 /** Get model string.
			  */
			 std::string getModel();

			 /** Get serial number as a string.
			  */
			 std::string getSerialNumber();

			 /** Get name string.
			  */
			 std::string getName();

			 /** Switch power on or off (returns true if turned on).
			  */
			 bool switchPower();

			 /** Get full reading (see PID documentation). In the returned observation, each reding is saved as a separate e-nose
			  */
			 mrpt::obs::CObservationGasSensors getFullInfo();

			 /** Get error status (true if an error was found). errorString shows the error code (see PID documentation)
			  */
			 bool errorStatus(std::string &errorString);

			 /** Get alarm limits
			  */
			 void getLimits(float &min, float &max);


		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
