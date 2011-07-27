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

#include <mrpt/maps.h>  // Precompiled header



#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/system/os.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>

#include <mrpt/opengl.h>
#include <numeric>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGasConcentrationGridMap2D, CRandomFieldGridMap2D,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CGasConcentrationGridMap2D::CGasConcentrationGridMap2D(
	TMapRepresentation	mapType,
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution ) :
		CRandomFieldGridMap2D(mapType, x_min,x_max,y_min,y_max,resolution ),
		insertionOptions(),
		m_debug_dump(NULL),
		decimate_count(1),
		fixed_incT(0),
		first_incT(true)
{
	// Set the grid to initial values (and adjusts the KF covariance matrix!)
	//  Also, calling clear() is mandatory to end initialization of our base class (read note in CRandomFieldGridMap2D::CRandomFieldGridMap2D)
	CMetricMap::clear();
}

CGasConcentrationGridMap2D::~CGasConcentrationGridMap2D()
{
	mrpt::utils::delete_safe(m_debug_dump);
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::internal_clear()
{
	// Just do the generic clear:
	CRandomFieldGridMap2D::internal_clear();

	// Anything else special for this derived class?
	// ...
}

/*---------------------------------------------------------------
						save_log_map
  ---------------------------------------------------------------*/
void CGasConcentrationGridMap2D::save_log_map(
	const mrpt::system::TTimeStamp	timestamp,
	const float						reading,
	const float						estimation,
	const float						k,
	const double					yaw,
	const float						speed
	)
{

	//function to save in a log file the information of the generated map

	double time = mrpt::system::timestampTotime_t(timestamp);
	char buffer [50];
	sprintf (buffer, "./log_MOSmodel_MAP_%X.txt",insertionOptions.sensorType);

	if (!m_debug_dump)
		m_debug_dump=new ofstream(buffer);

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

/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CGasConcentrationGridMap2D::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D			*robotPose )
{
	MRPT_START

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( IS_CLASS(obs, CObservationGasSensors ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationGasSensors
		********************************************************************/
		const CObservationGasSensors	*o = static_cast<const CObservationGasSensors*>( obs );
		float						sensorReading;

		//Cover all robot e-noses (m_readings)

		// Get index to differentiate between enoses --> insertionoptions.enose_id
		for (std::vector<CObservationGasSensors::TObservationENose>::const_iterator it = o->m_readings.begin(); it!=o->m_readings.end();it+=1)

//		ASSERT_(o->m_readings.size() > insertionOptions.enose_id);
//		const CObservationGasSensors::TObservationENose *it = &o->m_readings[insertionOptions.enose_id];
		{
			// Compute the 3D sensor pose in world coordinates:
			CPose2D		sensorPose( CPose3D(robotPose2D) + it->eNosePoseOnTheRobot );

			// Compute the sensor reading value (Volts):
			if (insertionOptions.sensorType==0x0000){	//compute the mean
				sensorReading = math::mean( it->readingsVoltage );
			}
			else
			{
				// Look for the correct sensor type
				size_t i;
				for (i=0; i<it->sensorTypes.size(); i++)
				{
					if (it->sensorTypes.at(i) == int(insertionOptions.sensorType) )
						break;
				}

				if (i<it->sensorTypes.size()){
					sensorReading = it->readingsVoltage[i];
				}
				else
				{
					//Sensor especified not found, compute default mean value
					sensorReading = math::mean( it->readingsVoltage );
				}
			}

			// Conversion Voltage(V)-->1/Resistance(1/Ohms)
			//sensorReading = 1/ (5 * insertionOptions.VoltageDivider_Res /sensorReading - insertionOptions.VoltageDivider_Res);

			// Normalization:
			sensorReading = (sensorReading - insertionOptions.R_min) /( insertionOptions.R_max - insertionOptions.R_min );


			// MOS model
			if(insertionOptions.useMOSmodel)
			{
				// Anti-Noise filtering
				noise_filtering(sensorReading, sensorPose, o->timestamp);

				//Decimate
				if ( decimate_count != insertionOptions.decimate_value ){
					decimate_count++;
					return true;
				}

				//Gas concentration estimation based on FIRST ORDER + NONLINEAR COMPENSATIONS DYNAMICS
				CGasConcentration_estimation(m_antiNoise_window[insertionOptions.winNoise_size/2].reading_filtered, m_antiNoise_window[insertionOptions.winNoise_size/2].sensorPose, m_antiNoise_window[insertionOptions.winNoise_size/2].timestamp);
				decimate_count = 1;

				//Use estimation to generate the map
				std::vector<TdataMap>::iterator iter = m_lastObservations.begin();
				sensorReading = iter->estimation;
				sensorPose = CPose2D(iter->sensorPose);

				//Save data map in log file for Matlab visualization
				if (insertionOptions.save_maplog)
					save_log_map(iter->timestamp, iter->reading, iter->estimation, iter->k, iter->sensorPose.yaw(), iter->speed);
			}

			// Update the gross estimates of mean/vars for the whole reading history (see IROS2009 paper):
			m_average_normreadings_mean = (sensorReading + m_average_normreadings_count*m_average_normreadings_mean)/(1+m_average_normreadings_count);
			m_average_normreadings_var  = (square(sensorReading - m_average_normreadings_mean) + m_average_normreadings_count*m_average_normreadings_var) /(1+m_average_normreadings_count);
			m_average_normreadings_count++;

#if 0
			cout << "[DEBUG] m_average_normreadings_count: " << m_average_normreadings_count << " -> mean: " << m_average_normreadings_mean << " var: " <<  m_average_normreadings_var  << endl;
#endif

			// Finally, do the actual map update with that value:
			switch (m_mapType)
			{
				case mrKernelDM:           insertObservation_KernelDM_DMV(sensorReading,sensorPose, false); break;
				case mrKernelDMV:          insertObservation_KernelDM_DMV(sensorReading,sensorPose, true); break;
				case mrKalmanFilter:       insertObservation_KF(sensorReading,sensorPose); break;
				case mrKalmanApproximate:  insertObservation_KF2(sensorReading,sensorPose);
			};

		} // for each e-nose obs.

		return true;	// Done!

	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}

/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CGasConcentrationGridMap2D::computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	n;

		// Save the dimensions of the grid:
		out << m_x_min << m_x_max << m_y_min << m_y_max;
		out << m_resolution;
		out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y);

		// To assure compatibility: The size of each cell:
		n = static_cast<uint32_t>(sizeof( TGasConcentrationCell ));
		out << n;

		// Save the map contents:
		n = static_cast<uint32_t>(m_map.size());
		out << n;

		// Save the "m_map": This requires special handling for big endian systems:
#if MRPT_IS_BIG_ENDIAN
		for (uint32_t i=0;i<n;i++)
		{
			out << m_map[i].kf_mean << m_map[i].dm_mean << m_map[i].dmv_var_mean;
		}
#else
		// Little endian: just write all at once:
		out.WriteBuffer( &m_map[0], sizeof(m_map[0])*m_map.size() );  // TODO: Do this endianness safe!!
#endif


		// Version 1: Save the insertion options:
		out << uint8_t(m_mapType)
			<< m_cov
			<< m_stackedCov;

		out << insertionOptions.sigma
			<< insertionOptions.cutoffRadius
			<< insertionOptions.R_min
			<< insertionOptions.R_max
			<< insertionOptions.KF_covSigma
			<< insertionOptions.KF_initialCellStd
			<< insertionOptions.KF_observationModelNoise
			<< insertionOptions.KF_defaultCellMeanValue
			<< insertionOptions.KF_W_size;

		// New in v3:
		out << m_average_normreadings_mean << m_average_normreadings_var << uint64_t(m_average_normreadings_count);

	}
}

// Aux struct used below (must be at global scope for STL):
struct TOldCellTypeInVersion1
{
	float	mean, std;
	float	w,wr;
};

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t	n,i,j;

			// Load the dimensions of the grid:
			in >> m_x_min >> m_x_max >> m_y_min >> m_y_max;
			in >> m_resolution;
			in >> i >> j;
			m_size_x = i;
			m_size_y = j;

			// To assure compatibility: The size of each cell:
			in >> n;

			if (version<2)
			{	// Converter from old versions <=1
				ASSERT_( n == static_cast<uint32_t>( sizeof( TOldCellTypeInVersion1 ) ));
				// Load the map contents in an aux struct:
				in >> n;
				vector<TOldCellTypeInVersion1>  old_map(n);
				in.ReadBuffer( &old_map[0], sizeof(old_map[0])*old_map.size() );

				// Convert to newer format:
				m_map.resize(n);
				for (size_t k=0;k<n;k++)
				{
					m_map[k].kf_mean = (old_map[k].w!=0) ? old_map[k].wr : old_map[k].mean;
					m_map[k].kf_std  = (old_map[k].w!=0) ? old_map[k].w  : old_map[k].std;
				}
			}
			else
			{
				ASSERT_EQUAL_( n , static_cast<uint32_t>( sizeof( TGasConcentrationCell ) ));
				// Load the map contents:
				in >> n;
				m_map.resize(n);

				// Read the note in writeToStream()
#if MRPT_IS_BIG_ENDIAN
				for (uint32_t i=0;i<n;i++)
					in >> m_map[i].kf_mean >> m_map[i].dm_mean >> m_map[i].dmv_var_mean;
#else
				// Little endian: just read all at once:
				in.ReadBuffer( &m_map[0], sizeof(m_map[0])*m_map.size() );
#endif
			}

			// Version 1: Insertion options:
			if (version>=1)
			{
				uint8_t	i;
				in  >> i;
				m_mapType = TMapRepresentation(i);

				in	>> m_cov
					>> m_stackedCov;

				in  >> insertionOptions.sigma
					>> insertionOptions.cutoffRadius
					>> insertionOptions.R_min
					>> insertionOptions.R_max
					>> insertionOptions.KF_covSigma
					>> insertionOptions.KF_initialCellStd
					>> insertionOptions.KF_observationModelNoise
					>> insertionOptions.KF_defaultCellMeanValue
					>> insertionOptions.KF_W_size;
			}

			if (version>=3)
			{
				uint64_t N;
				in >> m_average_normreadings_mean >> m_average_normreadings_var >> N;
				m_average_normreadings_count = N;
			}

			m_hasToRecoverMeanAndCov = true;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CGasConcentrationGridMap2D::TInsertionOptions::TInsertionOptions() :
	sensorType				( 0x0000 ),		//By default use the mean between all e-nose sensors

	//MOSmodel parameters (only if useMOSmodel = true)
	useMOSmodel					( false ),
	tauR						( 4 ),			//Time constant for the rise phase
	tauD						( 4 ),
	lastObservations_size		( 5 ),
	winNoise_size				( 30 ),
	decimate_value				( 2 ),
	calibrated_tauD_voltages	( 0 ),			//Voltages values of the sensorID at the calibration points.
	calibrated_tauD_values		( 0 ),			//Tau_d
	calibrated_delay_RobotSpeeds ( 0 ),			//[m/s]
	calibrated_delay_values		( 0 ),			//Number of delayed samples before decimation
	enose_id					( 0 ),			//By default use the first enose
	save_maplog					( false )
	//VoltageDivider_Res			( 10000 ),
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CGasConcentrationGridMap2D::TInsertionOptions] ------------ \n\n");
	internal_dumpToTextStream_common(out);  // Common params to all random fields maps:

	out.printf("sensorType                           = %u\n", (unsigned)sensorType);
	out.printf("enose_id								= %u\n", (unsigned)enose_id);
	out.printf("useMOSmodel								= %c\n", useMOSmodel ? 'Y':'N' );
	//out.printf("VoltageDivider_Res						= %f\n", VoltageDivider_Res);
	//MOSmodel parameters
	out.printf("tauR		                            = %f\n", tauR);
	out.printf("winNoise_size		                    = %u\n", (unsigned)winNoise_size);
	out.printf("decimate_value		                    = %u\n", (unsigned)decimate_value);
	out.printf("lastObservations_size                   = %u\n", (unsigned)lastObservations_size);

	//Need to dump the vector parameters (ToDo)
	out.printf("save_maplog		                        = %c\n", save_maplog ? 'Y':'N' );

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile,section);

	{
		int tmpSensorType = iniFile.read_int(section.c_str(),"sensorType",-1);
		if (tmpSensorType>=0)
		{
			// Valid number found:
			sensorType = tmpSensorType;
		}
		else
		{ // fall back to old name, or default to current value:
			sensorType = iniFile.read_int(section.c_str(),"KF_sensorType",sensorType);
		}
	}

	enose_id				= iniFile.read_int(section.c_str(),"enose_id",enose_id);
	//VoltageDivider_Res		= iniFile.read_float(section.c_str(),"VoltageDivider_Res",VoltageDivider_Res);
	useMOSmodel				= iniFile.read_bool(section.c_str(),"useMOSmodel",useMOSmodel);

	//MOSmodel parameters

	tauR					= iniFile.read_float(section.c_str(),"tauR",tauR);
	winNoise_size			= iniFile.read_int(section.c_str(),"winNoise_size",winNoise_size);
	decimate_value			= iniFile.read_int(section.c_str(),"decimate_value",decimate_value);
	lastObservations_size	= iniFile.read_int(section.c_str(),"lastObservations_size",lastObservations_size);
	iniFile.read_vector(section.c_str(),"calibrated_tauD_voltages",calibrated_tauD_voltages,calibrated_tauD_voltages);
	iniFile.read_vector(section.c_str(),"calibrated_tauD_values",calibrated_tauD_values,calibrated_tauD_values);
	iniFile.read_vector(section.c_str(),"calibrated_delay_RobotSpeeds",calibrated_delay_RobotSpeeds,calibrated_delay_RobotSpeeds);
	iniFile.read_vector(section.c_str(),"calibrated_delay_values",calibrated_delay_values,calibrated_delay_values);
	save_maplog				= iniFile.read_bool(section.c_str(),"save_maplog",save_maplog);

	if (useMOSmodel)
	{
		//Update the values of delay according to decimation
		for (int i = 0; i<calibrated_delay_values.size(); i++){
			calibrated_delay_values[i] = round( calibrated_delay_values[i]/decimate_value );
		}
		//Get the lastObservations_size (Must be higher than max delay_value/decimate_value)
		if ( lastObservations_size < *max_element(calibrated_delay_values.begin(), calibrated_delay_values.end()) )
			lastObservations_size = *max_element( calibrated_delay_values.begin(), calibrated_delay_values.end() )+1 ;
	}
}


/*---------------------------------------------------------------
					getAsBitmapFile
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAsBitmapFile(mrpt::utils::CImage &out_img) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::getAsBitmapFile(out_img);

	MRPT_END
}


/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::saveMetricMapRepresentationToFile(filNamePrefix);

	MRPT_END
}

/*---------------------------------------------------------------
					saveAsMatlab3DGraph
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::saveAsMatlab3DGraph(const std::string  &filName) const
{
	MRPT_START

	// Nothing special in this derived class:
	CRandomFieldGridMap2D::saveAsMatlab3DGraph(filName);

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START
	if (m_disableSaveAs3DObject)
		return;

	CRandomFieldGridMap2D::getAs3DObject(outObj);

	MRPT_END
}


/*---------------------------------------------------------------
				CGasConcentration_estimation
  ---------------------------------------------------------------*/
	// First order estimator model with non-linear dynamics compensation : x(i-N) =  ((y(i) -y(i-1))/(incT* k)) + y(i);
void CGasConcentrationGridMap2D::CGasConcentration_estimation (
	float	reading,
	const	CPose3D	&sensorPose,
	const	mrpt::system::TTimeStamp timestamp )
{
	int N;	//Memory efect delay

	// Check if estimation posible (at least one previous reading)
	if ( !m_lastObservations.empty() )
	{

		//Enose movement speed
		double speed_x = sensorPose.x() - m_new_Obs.sensorPose.x();
		double speed_y = sensorPose.y() - m_new_Obs.sensorPose.y();
		double incT = mrpt::system::timeDifference(m_new_Obs.timestamp,timestamp);

		if ( (incT >0) & (!first_incT) ){	//not the same sample (initialization of buffers)
			if (fixed_incT == 0)
				fixed_incT = incT;
			else
				ASSERT_(fabs(incT - fixed_incT) < (double)(0.05));
			m_new_Obs.speed = (sqrt (speed_x*speed_x + speed_y*speed_y)) / incT;
		}
		else
		{
			m_new_Obs.speed = 0;
			if (incT > 0)
				first_incT = false;
		}


		//slope>=0 -->Rise
		if ( reading >= m_new_Obs.reading )
		{
			m_new_Obs.k = 1.0/insertionOptions.tauR;
            N = 1;	//Memory effect compensation
		}
		//slope<0 -->decay
		else
		{
			//start decaying
			if (m_new_Obs.k == (float) (1.0/insertionOptions.tauR) ){
				//Use amplitude or just value?
				// Non-Linear compensation = f(sensor, amplitude, speed)
				m_new_Obs.k = 1.0/mrpt::math::leastSquareLinearFit((reading - insertionOptions.R_min),insertionOptions.calibrated_tauD_voltages,insertionOptions.calibrated_tauD_values,false);
			}else{
				//Do Nothing, keep the same tauD as last observation

			}// end-if(start decaying)

			//Dealy effect compensation
			N = mrpt::math::leastSquareLinearFit(m_new_Obs.speed ,insertionOptions.calibrated_delay_RobotSpeeds, insertionOptions.calibrated_delay_values,false);
			N = round(N);

			if (N >insertionOptions.lastObservations_size -1)
			{
				N = insertionOptions.lastObservations_size-1;
			}
		}//end-if  tau = f(slope)


		//New estimation values -- Ziegler-Nichols model --
		if( incT >0)
			//Initially there may come repetetive values till m_antiNoise_window is completed.
			m_new_Obs.estimation = ( ((reading -m_new_Obs.reading)/(incT* m_new_Obs.k)) )+ reading;
		else
			m_new_Obs.estimation = reading;


        //Prepare the New observation
		m_new_Obs.timestamp = timestamp ;
		m_new_Obs.reading = reading;
		m_new_Obs.sensorPose = sensorPose;

	}else{
		// First reading (use default values)
		m_new_Obs.k = 1.0/insertionOptions.tauR;
		m_new_Obs.reading = reading;
		m_new_Obs.timestamp = timestamp;
		m_new_Obs.speed = 0;
		m_new_Obs.sensorPose = sensorPose;
		m_new_Obs.estimation = reading;
		N = 1;

		//initialize the queue
		for (int i=0; i<insertionOptions.lastObservations_size; i++)
			m_lastObservations.push_back(m_new_Obs);

	}//end-if estimation values



	//Update m_lastObservations (due to memory efect)
	if (m_lastObservations[1].estimation == -20.0){
		//Copy right
		m_lastObservations[1].estimation = m_lastObservations[0].estimation;
	}

	m_lastObservations.erase( m_lastObservations.begin() );	//Erase the first element (the oldest)
	m_lastObservations.push_back(m_new_Obs);										//Add NULL obs as actual Obss
	m_lastObservations.rbegin()->estimation = -20.0;	//Non valid value to decect non valid observation

	//Modify queue estimation in the -Nth position
	try{
		m_lastObservations.at(insertionOptions.lastObservations_size - N-1).estimation = m_new_Obs.estimation;

	}catch(...){
		cout << "Error al acceder al array de m_readings \n" ;
		mrpt::system::pause();
	}

}//end-CGasConcentration_estimation



/*---------------------------------------------------------------
			noise_filtering
---------------------------------------------------------------*/
void CGasConcentrationGridMap2D::noise_filtering (
	float	reading,
	const	CPose3D	&sensorPose,
	const	mrpt::system::TTimeStamp timestamp )
{
	float partial_sum;

	m_new_ANS.reading = reading;
	m_new_ANS.timestamp = timestamp;
	m_new_ANS.sensorPose = sensorPose;

	if ( m_antiNoise_window.empty() )
	{
		// First reading (use default values)
		m_new_ANS.reading_filtered = reading;

		//initialize the queue
		m_antiNoise_window.assign( insertionOptions.winNoise_size, m_new_ANS );

	}else{
		m_antiNoise_window.erase( m_antiNoise_window.begin() );	//Erase the first element (the oldest)
		m_antiNoise_window.push_back(m_new_ANS);
	}

	//Average data to reduce noise
	partial_sum = 0;
	for (size_t i=0; i<m_antiNoise_window.size(); i++)
		partial_sum += m_antiNoise_window.at(i).reading;

	m_antiNoise_window.at(insertionOptions.winNoise_size/2).reading_filtered = partial_sum / insertionOptions.winNoise_size;

}
