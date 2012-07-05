/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
