/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
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

#include <mrpt/obs.h>   // Precompiled headers



#include <mrpt/slam/CObservationGasSensors.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGasSensors, CObservation,mrpt::slam)

/** Constructor
 */
CObservationGasSensors::CObservationGasSensors( ) :
	m_readings()
{

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGasSensors::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 5;
	else
	{
		uint32_t	i,n = m_readings.size();
		out << n;

		for (i=0;i<n;i++)
		{
			out << CPose3D(m_readings[i].eNosePoseOnTheRobot);
			out << m_readings[i].readingsVoltage;
			out << m_readings[i].sensorTypes;
			out << m_readings[i].hasTemperature;
			if (m_readings[i].hasTemperature)
				out << m_readings[i].temperature;
		}

		out << sensorLabel
		    << timestamp;

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGasSensors::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 2:
	case 3:
	case 4:
	case 5:
		{
		// A general set of e-nose descriptors:
		uint32_t	i,n;

		in >> n;
		m_readings.resize(n);

		CPose3D  aux;

		for (i=0;i<n;i++)
		{

			in >> aux; m_readings[i].eNosePoseOnTheRobot = aux;
			in >> m_readings[i].readingsVoltage;
			in >> m_readings[i].sensorTypes;
			if (version>=3)
			{
				in >> m_readings[i].hasTemperature;
				if (m_readings[i].hasTemperature)
					in >> m_readings[i].temperature;
			}
			else
			{
				m_readings[i].hasTemperature=false;
				m_readings[i].temperature=0;
			}
		}

		if (version>=4)
				in >> sensorLabel;
		else	sensorLabel = "";

		if (version>=5)
				in >> timestamp;
		else 	timestamp = INVALID_TIMESTAMP;


		}
		break;
	case 0:
	case 1:
		{
			TObservationENose		eNose;

			m_readings.clear();

			// There was a single set of 16 values from "Sancho" (DEC-2006)
			vector_float		readings;
			in >> readings;

			ASSERT_(readings.size()==16);

			// There was TWO e-noses:
			// (1)
			eNose.eNosePoseOnTheRobot = CPose3D( 0.20f,-0.15f, 0.10f ); // (x,y,z) only
			eNose.readingsVoltage.resize(4);
			eNose.readingsVoltage[0] = readings[2];
			eNose.readingsVoltage[1] = readings[4];
			eNose.readingsVoltage[2] = readings[5];
			eNose.readingsVoltage[3] = readings[6];

			eNose.sensorTypes.clear();
			eNose.sensorTypes.resize(4,0);
			m_readings.push_back(eNose);

			// (2)
			eNose.eNosePoseOnTheRobot = CPose3D( 0.20f, 0.15f, 0.10f ); // (x,y,z) only
			eNose.readingsVoltage.resize(4);
			eNose.readingsVoltage[0] = readings[8];
			eNose.readingsVoltage[1] = readings[10];
			eNose.readingsVoltage[2] = readings[12];
			eNose.readingsVoltage[3] = readings[14];

			eNose.sensorTypes.clear();
			eNose.sensorTypes.resize(4,0);
			m_readings.push_back(eNose);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


/*---------------------------------------------------------------
                     getSensorPose
 ---------------------------------------------------------------*/
void CObservationGasSensors::getSensorPose( CPose3D &out_sensorPose ) const
{
	if (m_readings.size())
		out_sensorPose = CPose3D(m_readings[0].eNosePoseOnTheRobot);
	else 	out_sensorPose = CPose3D(0,0,0);
}

/*---------------------------------------------------------------
                     setSensorPose
 ---------------------------------------------------------------*/
void CObservationGasSensors::setSensorPose( const CPose3D &newSensorPose )
{
	size_t		i, n = m_readings.size();
	if (n)
		for (i=0;i<n;i++)
			m_readings[i].eNosePoseOnTheRobot= mrpt::math::TPose3D(newSensorPose);
}






/*---------------------------------------------------------------
                     CMOSmodel
 ---------------------------------------------------------------*/
/** Constructor
 */

CObservationGasSensors::CMOSmodel::CMOSmodel():
	winNoise_size(30),
	decimate_value(6),
	tauR(),
	lastObservations_size(10),
	save_maplog(false),
	last_Obs(),
	temporal_Obs(),
	m_debug_dump(NULL),
	decimate_count(1),
	fixed_incT(0),
	first_incT(true),
	min_reading(10)
{
}


CObservationGasSensors::CMOSmodel::~CMOSmodel()
{
}

/*---------------------------------------------------------------
                 get_GasDistribution_estimation
 ---------------------------------------------------------------*/
bool CObservationGasSensors::CMOSmodel::get_GasDistribution_estimation(float &reading, CPose3D &sensorPose, const mrpt::system::TTimeStamp	&timestamp )
{
	try{
		//Noise filtering
		noise_filtering(reading, sensorPose, timestamp);

		//Decimate
		if ( decimate_count != decimate_value ){
			decimate_count++;
			return false;
		}

		//Gas concentration estimation based on FIRST ORDER + NONLINEAR COMPENSATIONS DYNAMICS
		inverse_MOSmodeling( m_antiNoise_window[winNoise_size/2].reading_filtered, m_antiNoise_window[winNoise_size/2].sensorPose, m_antiNoise_window[winNoise_size/2].timestamp);
		decimate_count = 1;

		//update
		std::vector<TdataMap>::iterator iter = m_lastObservations.begin();
		reading = iter->estimation;
		sensorPose = CPose2D(iter->sensorPose);

		//Save data map in log file for Matlab visualization
		if (save_maplog)
			save_log_map(iter->timestamp, iter->reading, iter->estimation, iter->k, iter->sensorPose.yaw(), iter->speed);

		return true;

	}catch(...){
		cout << "Error when decimating \n" ;
		mrpt::system::pause();
		return false;
	}
}

/*---------------------------------------------------------------
                     noise_filtering
 ---------------------------------------------------------------*/
void CObservationGasSensors::CMOSmodel::noise_filtering(const float &reading, const CPose3D &sensorPose, const	mrpt::system::TTimeStamp &timestamp )
{
	try{
		//Store values in the temporal Observation
		temporal_Obs.reading = reading;
		temporal_Obs.timestamp = timestamp;
		temporal_Obs.sensorPose = sensorPose;

		// If first reading from E-nose
		if ( m_antiNoise_window.empty() )
		{
			// use default values
			temporal_Obs.reading_filtered = reading;

			// Populate the noise window
			m_antiNoise_window.assign( winNoise_size, temporal_Obs );

		}else{
			//Erase the first element (the oldest), and add the new one
			m_antiNoise_window.erase( m_antiNoise_window.begin() );
			m_antiNoise_window.push_back(temporal_Obs);
		}

		//Average data to reduce noise (Noise Filtering)
		float partial_sum = 0;
		for (size_t i=0; i<m_antiNoise_window.size(); i++)
			partial_sum += m_antiNoise_window.at(i).reading;

		m_antiNoise_window.at(winNoise_size/2).reading_filtered = partial_sum / winNoise_size;

	}catch(...){
		cout << "Error when filtering noise from readings \n" ;
		mrpt::system::pause();
	}
}



/*---------------------------------------------------------------
                inverse_MOSmodeling
 ---------------------------------------------------------------*/
void CObservationGasSensors::CMOSmodel::inverse_MOSmodeling ( const float &reading, const CPose3D &sensorPose, const mrpt::system::TTimeStamp &timestamp)
{
	try{
		unsigned int N;	//Number of samples to delay

		//Keep the minimum reading value
		if (reading < min_reading)
			min_reading = reading;

		// Check if estimation posible (at least one previous reading)
		if ( !m_lastObservations.empty() )
		{
			//Enose movement speed
			double speed_x = sensorPose.x() - last_Obs.sensorPose.x();
			double speed_y = sensorPose.y() - last_Obs.sensorPose.y();
			double incT = mrpt::system::timeDifference(last_Obs.timestamp,timestamp);

			//Assure the samples are provided at constant rate (important for the correct gas distribution estimation)
			if ( (incT >0) & (!first_incT) ){	//not the same sample (initialization of buffers)
				if (fixed_incT == 0)
					fixed_incT = incT;
				else
					ASSERT_(fabs(incT - fixed_incT) < (double)(0.05));
				last_Obs.speed = (sqrt (speed_x*speed_x + speed_y*speed_y)) / incT;
			}
			else
			{
				last_Obs.speed = 0;
				if (incT > 0)
					first_incT = false;
			}


			//slope>=0 -->Rise
			if ( reading >= last_Obs.reading )
			{
				last_Obs.k = 1.0/tauR;
				N = 1;	//Delay effect compensation
			}
			else //slope<0 -->decay
			{
				//start decaying
				if (last_Obs.k == (float) (1.0/tauR) ){
					//Use the amplitude of the reading to calculate the tauD time constant for the next decaying phase
					last_Obs.k = 1.0 / mrpt::math::leastSquareLinearFit((reading - min_reading),calibrated_tauD_voltages,calibrated_tauD_values,false);
				}else{
					//Do Nothing, keep the same tauD as last observation (we are in the same decaying phase)
				}

				//Delay effect compensation
				N = mrpt::math::leastSquareLinearFit(last_Obs.speed ,calibrated_delay_RobotSpeeds, calibrated_delay_values,false);
				N = round(N);

				if (N >lastObservations_size -1)
				{
					N = lastObservations_size-1;
				}
			}//end-if

			//New estimation values -- Ziegler-Nichols model --
			if( incT >0)
				//Initially there may come repetetive values till m_antiNoise_window is full populated.
				last_Obs.estimation = ( ((reading -last_Obs.reading)/(incT* last_Obs.k)) )+ reading;
			else
				last_Obs.estimation = reading;


			//Prepare the New observation
			last_Obs.timestamp = timestamp ;
			last_Obs.reading = reading;
			last_Obs.sensorPose = sensorPose;

		}else{
			// First filtered reading (use default values)
			last_Obs.k = 1.0/tauR;
			last_Obs.reading = reading;
			last_Obs.timestamp = timestamp;
			last_Obs.speed = 0.0;
			last_Obs.sensorPose = sensorPose;
			last_Obs.estimation = reading;		//No estimation possible at this step
			N = 1;								//Default delay (number of samples)

			//populate the vector m_lastObservations
			for (unsigned int i=0; i<lastObservations_size; i++)
				m_lastObservations.push_back(last_Obs);

		}//end-if estimation values



		/* Update m_lastObservations
		   due to the delay in the sensor response, the gas estimation of the last_Obs may be introduced in any position of m_lastObservations
		   In order o maintain the time flow, only the last_Obs.estimation is not introduced in order.
		 */

		//-20 is a dummy value, just to detect a non-valid esimation for this Observation
		if (m_lastObservations[1].estimation == -20.0){
			//Copy right
			m_lastObservations[1].estimation = m_lastObservations[0].estimation;
		}

		m_lastObservations.erase( m_lastObservations.begin() );		//Erase the first element (the oldest)
		m_lastObservations.push_back(last_Obs);						//Add last_Obs in temporal order
		m_lastObservations.rbegin()->estimation = -20.0;			//Add Dummy value to estimation

		//Modify queue estimation in the -Nth position
		m_lastObservations.at(lastObservations_size - N-1).estimation = last_Obs.estimation;

	}catch(exception e){
		cerr << "**ERROR** " << e.what() << endl;
	}
}

/*---------------------------------------------------------------
						save_log_map
  ---------------------------------------------------------------*/
void CObservationGasSensors::CMOSmodel::save_log_map(
	const mrpt::system::TTimeStamp	&timestamp,
	const float						&reading,
	const float						&estimation,
	const float						&k,
	const double					&yaw,
	const float						&speed
	)
{

	//function to save in a log file the information of the generated gas distribution estimation

	double time = mrpt::system::timestampTotime_t(timestamp);
	char buffer [50];
	sprintf (buffer, "./log_MOSmodel_GasDistribution.txt");

	if (!m_debug_dump)
		m_debug_dump = new ofstream(buffer);

	if (m_debug_dump->is_open())
	{
		*m_debug_dump << format("%f \t", time );
		*m_debug_dump << format("%f \t", reading );
		*m_debug_dump << format("%f \t", estimation );
		*m_debug_dump << format("%f \t", k );
		*m_debug_dump << format("%f \t", yaw );
		*m_debug_dump << format("%f \t", speed );
		*m_debug_dump << "\n";
	}
	else cout << "Unable to open file";
}
