/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CIbeoLuxETH_H
#define CIbeoLuxETH_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CClientTCPSocket.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This "software driver" implements the communication protocol for interfacing a Ibeo Lux laser scanners through an ethernet controller.
			*   This class does not need to be bind, i.e. you do not need to call C2DRangeFinderAbstract::bindIO.
			*   Connection is established when user call the turnOn() method. You can pass to the class's constructor the Lux's ip address and port.
			*   Device will NOT be configured. Configuration has to be done seperately.
			*
			* To get a laser scan you must proceed like that :
			* \code
			*     CIbeoLuxETH laser(string("192.168.0.10"), 1234);
			*     laser.turnOn();
			*     bool isOutObs, hardwareError;
			*     CObservation2DRangeScan outObs;
			*     laser.doProcessSimple(isOutObs, outObs, hardwareError);
			* \endcode
			*
			* \note This class was contributed by Adrien Barral - Robopec (France)
			* \note And modified by Jan Girlich - University of Hamburg
		* \ingroup mrpt_hwdrivers_grp
			*/
		class HWDRIVERS_IMPEXP CIbeoLuxETH : public mrpt::hwdrivers::CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CIbeoLuxETH)

			public:
				/** Constructor.
					* Note that there is default arguments, here you can customize IP Adress and TCP Port of your device.
					*/
				CIbeoLuxETH(std::string _ip=std::string("10.152.36.93"), unsigned int _port=12002);
				/** Destructor.
					* Close communcation with the device, and free memory.
					*/
				virtual ~CIbeoLuxETH();
				/** This function acquire a laser scan from the device. If an error occured, hardwareError will be set to true.
					* The new laser scan will be stored in the outObservation argument.
					*
					* \exception This method throw exception if the frame received from the LMS 100 contain the following bad parameters :
					*  * Status is not OK
					*  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).
					*/
				void doProcess();
				void initialize();
				void start();
				void makeCommandHeader(unsigned char* buffer);
				void makeStartCommand(unsigned char* buffer);
				void makeStopCommand(unsigned char* buffer);
				void makeTypeCommand(unsigned char* buffer);

		private :
			std::string                  m_ip;
			unsigned int            m_port;
			mrpt::utils::CClientTCPSocket        m_client;
			unsigned int            m_scanFrequency;    // in hertz
			double                  m_angleResolution;  // in degrees
			double                  m_startAngle;       // degrees
			double                  m_stopAngle;        // degrees
			mrpt::poses::CPose3D                 m_sensorPose;
			double                  m_maxRange;
			double                  m_beamApperture;
			bool					m_run;
			void					dataCollection();
			mrpt::system::TThreadHandle	dataCollectionThread;
			double					convertLayerToRad(int scanlayer);
			double					convertTicksToHRad(int hticks, int hticksPerRotation);
			mrpt::poses::CPoint3D	convertToCartesian(float vrad, float hrad, float distance);
			float					vwinkel;
			std::vector<mrpt::obs::CObservation3DRangeScan>	m_observations;
			bool					m_newObs;
			float					m_vAngle;
			unsigned int			lastScanNumber;
			unsigned int			curScanNumber;

		protected:
			/** Load sensor pose on the robot, or keep the default sensor pose.
				*/
			void  loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,const std::string	  &iniSection );

		};
	}
}

#endif
