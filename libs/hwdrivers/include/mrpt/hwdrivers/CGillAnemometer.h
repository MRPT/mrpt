/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/config/CConfigFileBase.h>

namespace mrpt::hwdrivers
{
/** This class implements a driver for the Gill Windsonic Option 1 Anemometer
 *   The sensor is accessed via a standard serial port.
 *
 *   Refer to the manufacturer website for details on this sensor:
 *http://gillinstruments.com/data/manuals/WindSonic-Web-Manual.pdf
 *	  Configure for single <CR> return, at 2Hz
 *  \sa mrpt::obs::CObservationWindSensor
 * \ingroup mrpt_hwdrivers_grp
 */
class CGillAnemometer : public mrpt::hwdrivers::CGenericSensor
{
	DEFINE_GENERIC_SENSOR(CGillAnemometer)

   private:
	/** COM port name
	 */
	std::string com_port;
	int com_bauds;

	/** COM port
	 */
	mrpt::comms::CSerialPort COM;

	/** Poses
	 */
	float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;

	/** Returns true if the COM port is already open, or try to open it in other
	 * case.
	 * \return true if everything goes OK, or false if there are problems
	 * opening the port.
	 */
	bool tryToOpenTheCOM();

   public:
	/** Default constructor.
	 */
	CGillAnemometer();
	/** Default destructor.
	 */
	~CGillAnemometer() override { COM.close(); };
	void doProcess() override;

	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section) override;
};  // End of class def.

}  // namespace mrpt::hwdrivers
