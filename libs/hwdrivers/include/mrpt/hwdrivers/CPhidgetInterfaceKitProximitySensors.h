/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CPhidgetInterfaceKitProximitySensors_H
#define CPhidgetInterfaceKitProximitySensors_H

#include <mrpt/obs/CObservationRange.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** \brief : An interface for the phidget Interface kit board (1018).
		 *  \class CPhidgetInterfaceKitProximitySensors
		 *  \author Adrien BARRAL - Robopec (aba@robopec.com).
		 *
		 * An interface for the Phidgets Interface kit board (part number 1018) on wich it could be plugged either an Sharp IR adaptater board
	     * (phidget's part number : 1101),or a MaxBotix EZ-1 sonar (phidget's part number : 1118).
		 * The configuration file describe what is plugged to this board, and the geometry of the sensors on the robots. See the exemple below.
		 * \code
		 * [PhidgetInterfaceKitProximitySensors]
		 * sensorLabel = FrontProximitySensors
		 * process_rate = 100 						// Integer value in Hz (common to all sensors), default value is 50Hz.
		 * displayRecapitulativeInformations = true	// default value = false.
		 * serialNumber = 12345						// The interface kit serial number (Integer value), default value is -1.
		 * sensor1 = SHARP-30cm						// sharp InfraRed sensor 30cm range (string value). capital to convert raw data to range data (in meters).
	  	 * pose1_x = 0								// position on the robot (float value in meters)
		 * pose1_y = 0
		 * pose1_z = 0.5
		 * pose1_yaw = -45.0     					// Angles in degrees (float value).
		 * pose1_pitch = 0
		 * pose1_roll = 0
		 *  //...
		 * sensorn = EZ1							// Maxbotix Ultrasound sonar
		 * posen_x = 0
		 * // ...
		 * \endcode
		 *
		 * The maximum number of sensors on this board is 8. Sensor 1 is the first sensor. If you haven't plugged any sensor on an entry of the board, you haven't to specify
		 * anyithing about this sensor in the configuration file.
		 * The following table enumerate the different sensors supported by this class.
		 * \latexonly
		 * \begin{tabular}{|c|c|c}
	     * 		\hline
		 * 		Part Number & Config file indentifiant & IR or US
		 * 		\hline
		 * 		MaxBotix EZ-1 Sonar Sensor & EZ1 & US \\
		 * 		GP2D12 & SHARP-30cm & IR \\
		 * 		GP2Y0A21** & SHARP-80cm & IR \\
		 * 		\hline
		 * \end{tabular}
		 *
		 * This isn't an event based implementation of the phidget library. That means that when an instanciation of a CPhidgetInterfaceKitProximitySensors is done, the constructor will block during
		 * in the worst case 200ms, if the board isn't found, an exception will be thrown.
		 * mrpt::obs::CObservation returned by this class is a CObservationRange. CObservationrange::minSensorDistance will be the minimum of the minimum of the sensor distances, e.g if you plug to the interface
		 * kit a GP2D12 (min range 4 cm) and a GP2Y0A21 (min range 8 cm), then CObservationrange::minSensorDistance = min(0.04,0.08) = 0.04. Respectively for the maximal range.
		 * \endlatexonly
		 * \warning{The Phidget library use udev. By default, udev require to be root to be launched, if you want to be able to run a program wich use a phidget board without be root, you must modify files in /etc/udev/rules.d .}
		  * \ingroup mrpt_hwdrivers_grp
		 */
		enum SensorType{SHARP_30cm, SHARP_80cm, EZ1, UNPLUGGED};

		class HWDRIVERS_IMPEXP CPhidgetInterfaceKitProximitySensors : public mrpt::utils::COutputLogger, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CPhidgetInterfaceKitProximitySensors)

		public:
			/** Constructor
			  * \param serialNumber The board's serial number. Set -1 to choose the first available board
			  */
			CPhidgetInterfaceKitProximitySensors();

			/** Destructor
			  */
			virtual ~CPhidgetInterfaceKitProximitySensors();

			/** This method tries to get a set of range measurements from the IR sensors.
			  * \param outThereIsObservation Will be true if an observation was sucessfully received.
			  */
			void  getObservation(mrpt::obs::CObservationRange	&outObservation);
			/** Initialize the sensor according to the parameters previously read in the configuration file.
			 * \exception throw an exception if the board could not be found.
			 * \exception throw an exception if the process rate can't be set on one of the board channel.
			 */
			void initialize();

			/** This method should be called periodically. Period depend on the process_rate in the configuration file.
			 */
			void  doProcess();

		private:
			/** An 8 dimension vector of boolean value wich store the presence or abscence of a sensor on the phidget interface kit board.
			  */
			std::vector<bool>			m_sensorIsPlugged;
			/** The minimum range in meters, this field is automaticaly filled according to the sensor part number read in the configuration file.
			  * Size of this vector depend on the number of sensors described in the configuration file.
			  */
			std::vector<float>			m_minRange;

			/** The maximum range in meters, this field is automaticaly filled according to the sensor part number read in the configuration file.
			  * Size of this vector depend on the number of sensors described in the configuration file.
			  */
			std::vector<float>			m_maxRange;

			/** The sensor type.
			  */
			std::vector<SensorType>		m_sensorType;
			/** The poses of the 8 sensors x[m] y[m] z[m] yaw[deg] pitch[deg] roll[deg]. This field is automaticaly filled according to the sensor
			  * described in the configuration file.
			  */
			std::vector<mrpt::poses::CPose3D>	m_sensorPoses;

			/** The board serial number read in the configuration file. -1 for any  board.
			 */
			int m_serialNumber;
			float m_minOfMinRanges;
			float m_maxOfMaxRanges;

			void* m_carteInterfaceKit; //CPhidgetInterfaceKitHandle

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase &configSource,
											 const std::string	  &iniSection );
		}; // end class

	} // end namespace
} // end namespace

#endif
