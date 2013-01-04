/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef CLMS100ETH_H
#define CLMS100ETH_H

#include <mrpt/utils.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>

namespace mrpt
{
    namespace hwdrivers
    {
		using namespace std;
		using namespace mrpt::hwdrivers;
		using namespace mrpt::utils;
		using namespace mrpt::slam;

        /** This "software driver" implements the communication protocol for interfacing a SICK LMS100 laser scanners through an ethernet controller.
          *   This class does not need to be bind, i.e. you do not need to call C2DRangeFinderAbstract::bindIO.
          *   Connection is established when user call the turnOn() method. You can pass to the class's constructor the LMS100 's ip address and port.
          *   Device will be configured with the following parameters :
          * - Start Angle : -45 deg (imposed by hardware)
          * - Stop Angle : +225 deg (imposed by hardware)
          * - Apperture : 270 deg (imposed by hardware)
          * - Angular resolution : 0.25 deg
          * - Scan frequency : 25 Hz
          * - Max Range : 20m (imposed by hardware).
          *
          * <b>Important note:</b> SICK LMS 1xx devices have two levels of configuration. In its present implementation, this class only handles one of them, so
          *    <b>before using this class</b>, you must "pre-configure" your scanner with the SICK's software "SOAP" (this software ships with the device),
          *    and set the framerate with this software. Of course, you have to pre-configure the device just once, then save that configuration in its flash memory.
          *
          * To get a laser scan you must proceed like that :
          * \code
          *     CLMS200Eth laser(string("192.168.0.10"), 1234);
          *     laser.turnOn();
          *     bool isOutObs, hardwareError;
          *     CObservation2DRangeScan outObs;
          *     laser.doProcessSimple(isOutObs, outObs, hardwareError);
          * \endcode
          *
          * The sensor pose on the vehicle could be loaded from an ini configuration file with :
          *  \code
          *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
          * -------------------------------------------------------
          *   [supplied_section_name]
		  *	  ip_address = 192.168.0.50 ;a string wich is the SICK's ip adress (default is 192.168.0.1)
		  *   TCP_port = 1234			; an integer value : the tcp ip port on wich the sick is listening (default is 2111).
          *   pose_x=0.21	; Laser range scaner 3D position in the robot (meters)
          *   pose_y=0
          *   pose_z=0.34
          *   pose_yaw=0	; Angles in degrees
          *   pose_pitch=0
          *   pose_roll=0
          *  \endcode
          * This class doesn't configure the SICK LMS sensor, it is recomended to configure the sensor via the
	  * the SICK software : SOPAS.
          * \note This class was contributed by Adrien Barral - Robopec (France)
	  * \ingroup mrpt_hwdrivers_grp
          */
        class HWDRIVERS_IMPEXP CLMS100Eth : public C2DRangeFinderAbstract
        {
            DEFINE_GENERIC_SENSOR(CLMS100Eth)
            public:
                /** Constructor.
                 * Note that there is default arguments, here you can customize IP Adress and TCP Port of your device.
                 */
                CLMS100Eth(string _ip=string("192.168.0.1"), unsigned int _port=2111);
                /** Destructor.
                 * Close communcation with the device, and free memory.
                 */
                virtual ~CLMS100Eth();
                /** This function acquire a laser scan from the device. If an error occured, hardwareError will be set to true.
                 * The new laser scan will be stored in the outObservation argument.
                 *
                 * \exception This method throw exception if the frame received from the LMS 100 contain the following bad parameters :
                 *  * Status is not OK
                 *  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).
                 */
                void doProcessSimple(bool &outThereIsObservation, CObservation2DRangeScan &outObservation, bool &hardwareError);

                /** This method must be called before trying to get a laser scan.
                 */
                bool turnOn();
                /** This method could be called manually to stop communication with the device. Method is also called by destructor.
                 */
                bool turnOff();

                /** A method to set the sensor pose on the robot.
                  * Equivalent to setting the sensor pose via loading it from a config file.
                  */
                void setSensorPose(const CPose3D& _pose);

				/** This method should be called periodically. Period depend on the process_rate in the configuration file.
			 	 */
				void  doProcess();

				/** Initialize the sensor according to the parameters previously read in the configuration file.
				 */
				void initialize();
        private :
            string                  m_ip;
            unsigned int            m_port;
            CClientTCPSocket        m_client;
            bool                    m_turnedOn;
            string					m_cmd;
            bool                    m_connected;
            unsigned int            m_scanFrequency;    // en hertz
            double                  m_angleResolution;  // en degrés
            double                  m_startAngle;       // degrés
            double                  m_stopAngle;        // degrés
            CPose3D                 m_sensorPose;
            double                  m_maxRange;
            double                  m_beamApperture;

            void generateCmd(const char *cmd);
            bool checkIsConnected();
            bool decodeLogIn(char *msg);
            bool decodeScanCfg(istringstream& stream);
            bool decodeScanDataCfg(istringstream& stream);
            bool decodeScan(char *buf, CObservation2DRangeScan& outObservation);
            void sendCommand(const char *cmd);
            void roughPrint( char *msg );


		protected:
            /** Load sensor pose on the robot, or keep the default sensor pose.
             */
            void  loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,
                             const std::string	  &iniSection );

        };
    }
}
#endif // CLMS100ETH_H
