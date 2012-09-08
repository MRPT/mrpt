/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef CIbeoLuxETH_H
#define CIbeoLuxETH_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/utils/CClientTCPSocket.h>

namespace mrpt
{
    namespace hwdrivers
    {
		using namespace std;
		using namespace mrpt::hwdrivers;
		using namespace mrpt::utils;
		using namespace mrpt::slam;

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
                CIbeoLuxETH(string _ip=string("10.152.36.93"), unsigned int _port=12002);
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
            string                  m_ip;
            unsigned int            m_port;
            CClientTCPSocket        m_client;
            unsigned int            m_scanFrequency;    // in hertz
            double                  m_angleResolution;  // in degrees
            double                  m_startAngle;       // degrees
            double                  m_stopAngle;        // degrees
            CPose3D                 m_sensorPose;
            double                  m_maxRange;
            double                  m_beamApperture;
			bool					m_run;
			void					dataCollection();
			mrpt::system::TThreadHandle	dataCollectionThread;
			float					convertLayerToRad(int scanlayer);
			float					convertTicksToHRad(int hticks, int hticksPerRotation);
			CPoint3D				convertToCartesian(float vrad, float hrad, float distance);
			float					vwinkel;
			vector<CObservation3DRangeScan>	m_observations;
			bool					m_newObs;
			float					m_vAngle;
			unsigned int			lastScanNumber;
			unsigned int			curScanNumber;

		protected:
            /** Load sensor pose on the robot, or keep the default sensor pose.
             */
            void  loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,
                             const std::string	  &iniSection );

        };
    }
}

#endif
