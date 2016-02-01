/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CPtuHokuyo_H
#define CPtuHokuyo_H

#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/utils/CFileOutputStream.h>
#include "CPtuDPerception.h"

namespace mrpt
{
	namespace hwdrivers
	{
		/** The objetive of this class is to coordinate PTU movements and
		 *	Hokuyo scans, adding the posibility of save the points earned
		 *	in several different formats, limit valids points and view
		 * 	them on a grahic.
		 * \ingroup mrpt_hwdrivers_grp
		 */

		class HWDRIVERS_IMPEXP CPtuHokuyo;

		struct ThreadParams
		{
			char axis;
			bool start_capture;
			double scan_vel;  // Velocity of continuous scan
			CPtuHokuyo* ptu_hokuyo;
		};

		class HWDRIVERS_IMPEXP CPtuHokuyo : public CGenericSensor
		{

		DEFINE_GENERIC_SENSOR(CPtuHokuyo)

		protected:

			std::string		m_ptu_port;
			char m_axis;
			double m_velocity, m_initial, m_final, m_hokuyo_frec;

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

		public:

			CHokuyoURG			laser;
			CPtuBase			*ptu;

			/** Specify type of ptu. Current options are:
			*	m_ptu_type = 0 => CPtuDPerception
			*	m_ptu_type = 1 => CPtuMicos
			*/
			int					m_ptu_type;

			std::vector<mrpt::obs::CObservation2DRangeScan> vObs;

			// High between ptu tilt axis and hokuyo laser scan
			double high;

			struct my_pos
			{
				mrpt::system::TTimeStamp timeStamp;
				double pos;
			};

			std::vector<mrpt::hwdrivers::CPtuHokuyo::my_pos> v_my_pos;
			std::vector<double> v_ptu_pos, v_ptu_time;


			/** Default constructor */

			CPtuHokuyo();

			/** Destructor, delete observations of the vector */

			~CPtuHokuyo();

			/** Initialization of laser and ptu */

			bool init(const std::string &portPtu, const std::string &portHokuyo);

			/** Performs a complete scan
			 *	\param <axis> Pan or Till
			 *	\param <tWait> Wait time betwen commands
			 *	\param <initial> initial position
			 *	\param <final> final position
			 *	\param <radPre> radians precision for the scan
			 *	\param <interlaced> if interlaced==true performs a double sweep
			 */

			bool scan(char &axis, const int &tWait, double &initial, double &final, const double &radPre, const int &mean, const bool &interlaced=false);

			/** Performs a continuous scan */

			bool continuousScan(char &axis, const double &velocity, double &initial, double &final);

			/** Show a graphic with the points obtained from the scan or a map*/
			//bool showGraphic(mrpt::maps::CSimplePointsMap	*theMap=0);

			/** Save a simple points map into a simple file (if colours==true save points with a color) */
			//bool saveMap2File(mrpt::maps::CSimplePointsMap	&theMap, char* fname="Data.pts", const bool &colours=false);

			/** Save vector of observations in a CFileOutputStream file */

			bool saveVObs2File(const char *fname="Data.rawlog");

			/** Save vector points of observations into a simple file */

			bool saveVObsPoints2File(const char *fname="Data.pts",const bool &colours=false);

			/** Save pitchs and raw distances of all scans */

			bool savePitchAndDistances2File();

			/** Method for limit map points obtained from a scan */
			//void limit(mrpt::maps::CSimplePointsMap &theMap);

			/** Set high between ptu tilt axis and hokuyo laser scan */

			void setHigh(const double &newHigh) { high = newHigh; }

			/** Obtain a observation from the laser */

			bool obtainObs( mrpt::obs::CObservation2DRangeScan & obs );

			/** This method can or cannot be implemented in the derived class, depending on the need for it.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			void initialize();

			/** This method will be invoked at a minimum rate of "process_rate" (Hz)
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			void doProcess();


		private:

			/** Save a observation from the laser into a vector of
			 *		observations, calculating sensor position
			 */

			double saveObservation(const char &axis, const int &mean);

			/** Performs a simple scan
			 *	\param <axis> Pan or Till
			 *	\param <tWait> Wait time betwen commands
			 *	\param <movements> number total of movements
			 *	\param <radPre> radians precision for the scan
			 *	\param <vObs> reference to obsevations vector for save the observation
			 */

			bool singleScan(const char &axis, const int &tWait, const int &movements, const double &radPre, const int &mean);

			/** Calculate minimum lenght of scan vectors */

			int minLengthVectors(mrpt::obs::CObservation2DRangeScan &obs, std::vector<mrpt::obs::CObservation2DRangeScan> &vObsAux);

			/** Calculate minimum lenght of 2 scan vectors */

			int minLengthVectors(mrpt::obs::CObservation2DRangeScan &obs1, mrpt::obs::CObservation2DRangeScan &obs2, const int &mode);

			/** Load observations in a points map */
			//void loadObs2PointsMap(mrpt::maps::CSimplePointsMap	&theMap);

			/** Limit the valid position of scan points */
			//bool limitScan(const char &axis, double &low, double &high, mrpt::maps::CSimplePointsMap		&theMap);

			/** Refine the observations obtains from a continuous scan */
			void refineVObs(const char &axis);

			/** Calculate the sensor pose depending teh axis of movements and the ptu position */

			void calculateSensorPose(const char &axis, const double &pos, mrpt::obs::CObservation2DRangeScan &obs);

			/** Obtain position of observations between first and second position in m_my_pos map */

			int obsPosition();


		};	// End of class

	} // End of namespace

} // End of namespace

#endif
