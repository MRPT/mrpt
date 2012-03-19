/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Emil Khatib  <emilkhatib@uma.es>		                       |
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
#ifndef  CRaePID_H
#define  CRaePID_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/slam/CObservationGasSensors.h>
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
		  *  \sa mrpt::slam::CObservationGasSensors
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CRaePID : public mrpt::hwdrivers::CGenericSensor
		{
				DEFINE_GENERIC_SENSOR(CRaePID)

		private:
			/** COM port name
			 */
			std::string com_port;

			/** COM port
			 */
			CSerialPort COM;

			/** Poses
			 */
			float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;

		public:
			/** Default constructor.
			 */
			CRaePID();
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
			 CObservationGasSensors getFullInfo();

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
