/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGasSensors, CObservation,mrpt::obs)

/** Constructor
 */
CObservationGasSensors::CObservationGasSensors( ) :
	m_readings()
{

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGasSensors::writeToStream(mrpt::utils::CStream &out, int *version) const
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
void  CObservationGasSensors::readFromStream(mrpt::utils::CStream &in, int version)
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
			CVectorFloat		readings;
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
	if (!m_readings.empty())
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
	a_rise(0),
	b_rise(0),
	a_decay(0),
	b_decay(0),
	save_maplog(false),
	last_Obs(),
	temporal_Obs(),
	m_debug_dump(NULL),
	decimate_count(1),
	fixed_incT(0),
	first_incT(true),
	min_reading(10),
	first_iteration(true)
{
}


CObservationGasSensors::CMOSmodel::~CMOSmodel()
{
}

/*---------------------------------------------------------------
                 get_GasDistribution_estimation
 ---------------------------------------------------------------*/
bool CObservationGasSensors::CMOSmodel::get_GasDistribution_estimation(float &reading, mrpt::system::TTimeStamp &timestamp )
{
	try{
		//Noise filtering
		noise_filtering(reading, timestamp);

		//Decimate
		if ( decimate_count != decimate_value ){
			decimate_count++;
			return false;
		}

		//Gas concentration estimation based on FIRST ORDER + NONLINEAR COMPENSATIONS DYNAMICS
		inverse_MOSmodeling( m_antiNoise_window[winNoise_size/2].reading_filtered, m_antiNoise_window[winNoise_size/2].timestamp);
		decimate_count = 1;

		//update (output)
		reading = last_Obs.estimation;
		timestamp = last_Obs.timestamp;

		//Save data map in log file for Matlab visualization
		if (save_maplog)
			save_log_map(last_Obs.timestamp, last_Obs.reading, last_Obs.estimation, last_Obs.tau );

		return true;

	}catch(...){
		cout << "Error when decimating \n" ;
		mrpt::system::pause();
		return false;
	}
}

/*---------------------------------------------------------------
                     noise_filtering (smooth)
 ---------------------------------------------------------------*/
void CObservationGasSensors::CMOSmodel::noise_filtering(const float &reading, const	mrpt::system::TTimeStamp &timestamp )
{
	try{
		//Store values in the temporal Observation
		temporal_Obs.reading = reading;
		temporal_Obs.timestamp = timestamp;

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
void CObservationGasSensors::CMOSmodel::inverse_MOSmodeling ( const float &reading, const mrpt::system::TTimeStamp &timestamp)
{
	try{

		//Keep the minimum reading value as an approximation to the basline level
		if (reading < min_reading)
			min_reading = reading;

		// Check if estimation posible (not possible in the first iteration)
		if ( !first_iteration)
		{
			//Assure the samples are provided at constant rate (important for the correct gas distribution estimation)
			double incT = mrpt::system::timeDifference(last_Obs.timestamp,timestamp);

			if ( (incT >0) & (!first_incT) ){	//not the same sample due to initialization of buffers
				if (fixed_incT == 0)
					fixed_incT = incT;
				else
					//ASSERT_(fabs(incT - fixed_incT) < (double)(0.05));
					if (fabs(incT - fixed_incT) > (double)(0.05))
						cout << "IncT is not constant by HW." << endl;
			}
			else
			{
				if (incT > 0)
					first_incT = false;
			}


			//slope<0 -->Decay
			if ( reading < last_Obs.reading )
			{
				last_Obs.tau = a_decay * abs(reading-min_reading) + b_decay;
			}
			else //slope>=0 -->rise
			{
				last_Obs.tau = a_rise * abs(reading-min_reading) + b_rise;
			}//end-if

			//New estimation values -- Ziegler-Nichols model --
			if( incT >0)
				//Initially there may come repetetive values till m_antiNoise_window is full populated.
				last_Obs.estimation = ( (reading -last_Obs.reading)*last_Obs.tau/incT) + reading;
			else
				last_Obs.estimation = reading;


			//Prepare the New observation
			last_Obs.timestamp = timestamp;
			last_Obs.reading = reading;

		}else{
			// First filtered reading (use default values)
			last_Obs.tau = b_rise;
			last_Obs.reading = reading;
			last_Obs.timestamp = timestamp;
			last_Obs.estimation = reading;		//No estimation possible at this step
			first_iteration = false;
		}//end-if estimation values


	}catch(exception e){
		cerr << "**Exception in CObservationGasSensors::CMOSmodel::inverse_MOSmodeling** " << e.what() << endl;
	}
}

/*---------------------------------------------------------------
						save_log_map
  ---------------------------------------------------------------*/
void CObservationGasSensors::CMOSmodel::save_log_map(
	const mrpt::system::TTimeStamp	&timestamp,
	const float						&reading,
	const float						&estimation,
	const float						&tau
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
		*m_debug_dump << format("%f \t", tau );
		*m_debug_dump << "\n";
	}
	else cout << "Unable to open file";
}

void CObservationGasSensors::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	for (size_t j=0;j<m_readings.size();j++)
	{
		o << format("e-nose #%u:\n",(unsigned)j);

		vector<float>::const_iterator it;
		vector_int::const_iterator   itKind;

		ASSERT_( m_readings[j].readingsVoltage.size() == m_readings[j].sensorTypes.size());

		for (it=m_readings[j].readingsVoltage.begin(),itKind=m_readings[j].sensorTypes.begin();it!=m_readings[j].readingsVoltage.end();it++,itKind++)
			o << format( "%04X: %.03f ", *itKind, *it);

		o << endl;

		o << format("  Sensor pose on robot: (x,y,z)=(%.02f,%.02f,%.02f)\n",
			m_readings[j].eNosePoseOnTheRobot.x,
			m_readings[j].eNosePoseOnTheRobot.y,
			m_readings[j].eNosePoseOnTheRobot.z );

		o << "Measured temperature: ";
		if (m_readings[j].hasTemperature)
			o << format("%.03f degC\n", m_readings[j].temperature );
		else
			o << "NOT AVAILABLE\n";
	}
}
