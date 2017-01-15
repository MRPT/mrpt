/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CRoboticHeadInterface_H
#define CRoboticHeadInterface_H

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <mrpt/utils/CMessage.h>
#include <mrpt/math/CMatrixTemplate.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/obs/CObservationRange.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/**   This "software driver" implements the communication protocol for interfacing a Robotic Head Board through a custom
		  *   USB RS-422 interface board.
		  *   In this class the "bind" is ignored since it is designed for USB connections only, thus it internally generate the required object for simplicity of use.
		  *   The default serial number is "OREJA001"
		  *
		  * Warning: Avoid defining an object of this class in a global scope if you want to catch all potential
		  *      exceptions during the constructors (like USB interface DLL not found, etc...)
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *   HEAD_serialNumber=OREJA001
		  *   HEAD_gain=127,127,127
		  *   HEAD_yaw=0		// initial yaw value
		  *   HEAD_pitch=0		// initial tilt 
		  *  \endcode
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CRoboticHeadInterface : public mrpt::utils::COutputLogger
		{
			private:
				CInterfaceFTDIMessages	m_usbConnection;
				utils::CMessage		msg;
				std::string				m_serialNumber;
				std::vector<int32_t>	gain;
				int						head_yaw, head_pitch;

				bool 		checkControllerIsConnected();

			protected:
				/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file,
				  *  loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
				  *  See hwdrivers::CSonarSRF10 for the possible parameters
				  */
				void  loadConfig_sensorSpecific(	const mrpt::utils::CConfigFileBase *configSource,
									const std::string	  &iniSection );

			public:
				/** Constructor
				  */
				CRoboticHeadInterface();

				/** Destructor
				  */
				~CRoboticHeadInterface(){}

				/** Changes the serial number of the device to open
				  */
				void  setDeviceSerialNumber(const std::string &deviceSerialNumber)
				{
					m_serialNumber = deviceSerialNumber;
				}

				/** Read the gain for the amplifier of the ear "channel", where channel is 0, 1 or 2.
				  */
				void GetGain(int &_gain,int &channel);

				/** Set the gain for the amplifier each ear. The value range is [0x00(min) .. 0x7F(max)]. The value 0x80 set the resistor
				  * in high impedance state, DON'T USE IT!!!
				  */
				bool SetGain(int &new_gain,int &channel);

				/** This function return the angle where last sound where detected. This angle is related to the robot pose, NOT head pose.
				  * \code
				  * angle > 0deg --> Sound detected in the left
				  * angle = 0deg --> Sound detected in front of the head
				  * angle < 0deg --> Sound detected in the right
				  * \endcode
				  */
				void GetSoundLocation(int &ang);

				/** Debug only!!! This function return the last 500 acquired samples for each sound channel.
				  *
				  */
				void Get3SoundBuffer(mrpt::math::CMatrixTemplate<int>	&buf);

				/** Move the head in:
				   \code
				  * elevation = 'yaw' degrees
				  * orientation = 'pitch' degrees
				  * \endcode
				  */
				void SetHeadPose(int &yaw, int &pitch);

		};	// End of class

	} // End of namespace
} // end of namespace

#endif

