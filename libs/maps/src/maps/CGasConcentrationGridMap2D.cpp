/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/utils/round.h>  // round()
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CGasConcentrationGridMap2D,gasGrid", mrpt::maps::CGasConcentrationGridMap2D)

CGasConcentrationGridMap2D::TMapDefinition::TMapDefinition() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CGasConcentrationGridMap2D::mrKernelDM)
{
}

void CGasConcentrationGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, float,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, float,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, float,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, float,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, float,   source,sSectCreation);
	mapType = source.read_enum<CGasConcentrationGridMap2D::TMapRepresentation>(sSectCreation,"mapType",mapType);

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
}

void CGasConcentrationGridMap2D::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CGasConcentrationGridMap2D::TMapRepresentation>::value2name(mapType).c_str() );
	LOADABLEOPTS_DUMP_VAR(min_x         , float);
	LOADABLEOPTS_DUMP_VAR(max_x         , float);
	LOADABLEOPTS_DUMP_VAR(min_y         , float);
	LOADABLEOPTS_DUMP_VAR(max_y         , float);
	LOADABLEOPTS_DUMP_VAR(resolution         , float);

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CGasConcentrationGridMap2D::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CGasConcentrationGridMap2D::TMapDefinition &def = *dynamic_cast<const CGasConcentrationGridMap2D::TMapDefinition*>(&_def);
	CGasConcentrationGridMap2D *obj = new CGasConcentrationGridMap2D(def.mapType,def.min_x,def.max_x,def.min_y,def.max_y,def.resolution );
	obj->insertionOptions  = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CGasConcentrationGridMap2D, CRandomFieldGridMap2D,mrpt::maps)

// Short-cut:
#define LUT_TABLE (*(LUT.table))

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
		insertionOptions()
{
	// Override defaults:
	insertionOptions.GMRF_saturate_min = 0;
	insertionOptions.GMRF_saturate_max = 1;
	insertionOptions.GMRF_lambdaObsLoss = 1.0;

	// Set the grid to initial values (and adjust covariance matrices,...)
	//  Also, calling clear() is mandatory to end initialization of our base class (read note in CRandomFieldGridMap2D::CRandomFieldGridMap2D)
	CMetricMap::clear();

	// Create WindGrids with same dimensions that the original map
	windGrid_module.setSize( x_min,x_max,y_min,y_max,resolution );
	windGrid_direction.setSize( x_min,x_max,y_min,y_max,resolution );

	//initialize counter for advection simulation
	timeLastSimulated = mrpt::system::now();
}

CGasConcentrationGridMap2D::~CGasConcentrationGridMap2D()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::internal_clear()
{
	// Just do the generic clear:
	CRandomFieldGridMap2D::internal_clear();

	// Anything else special for this derived class?

	if (insertionOptions.useWindInformation)
	{
		//Set default values of the wind grid
		windGrid_module.fill(insertionOptions.default_wind_speed);
		windGrid_direction.fill(insertionOptions.default_wind_direction);

		/*float S = windGrid_direction.getSizeX() * windGrid_direction.getSizeY();

		for( unsigned int y=windGrid_direction.getSizeY()/2; y<windGrid_direction.getSizeY(); y++ )
		{
			for( unsigned int x=0; x<windGrid_direction.getSizeX(); x++ )
			{
				double * wind_cell = windGrid_direction.cellByIndex(x,y);
				*wind_cell =  3*3.141516/2;
}
		}*/

		// Generate Look-Up Table of the Gaussian weights due to wind advection.
		if( !build_Gaussian_Wind_Grid())
		{
			//mrpt::system::pause();
			THROW_EXCEPTION("Problem with LUT wind table");

		}

	}
}


/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CGasConcentrationGridMap2D::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D		*robotPose )
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

		if ( o->sensorLabel.compare(insertionOptions.gasSensorLabel) == 0 )
		{
			float		sensorReading;
			CPose2D		sensorPose;

			if ( o->sensorLabel.compare("MCEnose")==0 || o->sensorLabel.compare("Full_MCEnose")==0 )
			{
				ASSERT_(o->m_readings.size() > insertionOptions.enose_id);
				const CObservationGasSensors::TObservationENose *it = &o->m_readings[insertionOptions.enose_id];

				// Compute the 3D sensor pose in world coordinates:
				sensorPose = CPose2D( CPose3D(robotPose2D) + it->eNosePoseOnTheRobot );

				// Compute the sensor reading value (Volts):
				if (insertionOptions.gasSensorType==0x0000){	//compute the mean
					sensorReading = math::mean( it->readingsVoltage );
				}
				else
				{
					// Look for the correct sensor type
					size_t i;
					for (i=0; i<it->sensorTypes.size(); i++)
					{
						if (it->sensorTypes.at(i) == int(insertionOptions.gasSensorType) )
							break;
					}

					if (i<it->sensorTypes.size()){
						sensorReading = it->readingsVoltage[i];
					}
					else
					{
						cout << "Sensor especified not found, compute default mean value" << endl;
						sensorReading = math::mean( it->readingsVoltage );
					}
				}
			}
			else //"GDM, RAE_PID, ENOSE_SIMUL
			{
				const CObservationGasSensors::TObservationENose *it = &o->m_readings[0];
				// Compute the 3D sensor pose in world coordinates:
				sensorPose = CPose2D( CPose3D(robotPose2D) + it->eNosePoseOnTheRobot );
				sensorReading = it->readingsVoltage[0];
			}


			// Normalization:
			sensorReading = (sensorReading - insertionOptions.R_min) /( insertionOptions.R_max - insertionOptions.R_min );


			// Update the gross estimates of mean/vars for the whole reading history (see IROS2009 paper):
			m_average_normreadings_mean = (sensorReading + m_average_normreadings_count*m_average_normreadings_mean)/(1+m_average_normreadings_count);
			m_average_normreadings_var  = (square(sensorReading - m_average_normreadings_mean) + m_average_normreadings_count*m_average_normreadings_var) /(1+m_average_normreadings_count);
			m_average_normreadings_count++;

#if 0
			cout << "[DEBUG] m_average_normreadings_count: " << m_average_normreadings_count << " -> mean: " << m_average_normreadings_mean << " var: " <<  m_average_normreadings_var  << endl;
#endif

			// Finally, do the actual map update with that value:
			this->insertIndividualReading(sensorReading, mrpt::math::TPoint2D(sensorPose.x(),sensorPose.y()) );
			return true;	// Done!

		} //endif correct "gasSensorLabel"

	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}

/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CGasConcentrationGridMap2D::internal_computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 5;
	else
	{
		dyngridcommon_writeToStream(out);

		// To assure compatibility: The size of each cell:
		uint32_t n = static_cast<uint32_t>(sizeof( TRandomFieldCell ));
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

		out << genericMapParams; // v4
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
void  CGasConcentrationGridMap2D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		{
			dyngridcommon_readFromStream(in, version<5);

			// To assure compatibility: The size of each cell:
			uint32_t	n;
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
				ASSERT_EQUAL_( n , static_cast<uint32_t>( sizeof( TRandomFieldCell ) ));
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

			if (version>=4) 
				in >> genericMapParams;

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

	gasSensorLabel				( "MCEnose" ),
	enose_id				( 0 ),			//By default use the first enose
	gasSensorType				( 0x0000 ),		//By default use the mean between all e-nose sensors
	windSensorLabel				( "windSensor" ),
	useWindInformation			( false ),		//By default dont use wind
	std_windNoise_phi			( 0.2f ),
	std_windNoise_mod			( 0.2f ),
	default_wind_direction		( 0.0f ),
	default_wind_speed			( 1.0f )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CGasConcentrationGridMap2D::TInsertionOptions] ------------ \n\n");
	out.printf("[TInsertionOptions.Common] ------------ \n\n");
	internal_dumpToTextStream_common(out);  // Common params to all random fields maps:

	out.printf("[TInsertionOptions.GasSpecific] ------------ \n\n");
	out.printf("gasSensorLabel							= %s\n", gasSensorLabel.c_str() );
	out.printf("enose_id								= %u\n", (unsigned)enose_id);
	out.printf("gasSensorType							= %u\n", (unsigned)gasSensorType);
	out.printf("windSensorLabel							= %s\n", windSensorLabel.c_str() );
	out.printf("useWindInformation						= %u\n", useWindInformation);

	out.printf("advectionFreq							= %f\n", advectionFreq);
	out.printf("default_wind_direction					= %f\n", default_wind_direction);
	out.printf("default_wind_speed						= %f\n", default_wind_speed);
	out.printf("std_windNoise_phi						= %f\n", std_windNoise_phi);
	out.printf("std_windNoise_mod						= %f\n", std_windNoise_mod);

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

	// Specific data fields for gasGridMaps
	gasSensorLabel	= iniFile.read_string(section.c_str(),"gasSensorLabel","Full_MCEnose",true);
	enose_id	= iniFile.read_int(section.c_str(),"enoseID",enose_id);
	//Read sensor type in hexadecimal
	{
		std::string sensorType_str = iniFile.read_string(section.c_str(),"gasSensorType","-1",true);
		int tmpSensorType;
		stringstream convert ( sensorType_str );
		convert>> std::hex >> tmpSensorType;

		if (tmpSensorType>=0)
		{
			// Valid number found:
			gasSensorType = tmpSensorType;
		}
		else
		{ // fall back to old name, or default to current value:
			gasSensorType = iniFile.read_int(section.c_str(),"KF_sensorType",gasSensorType,true);
		}
	}
	windSensorLabel	= iniFile.read_string(section.c_str(),"windSensorLabel","Full_MCEnose",true);

	//Indicates if wind information must be used for Advection Simulation
	useWindInformation				 = iniFile.read_bool(section.c_str(),"useWindInformation","false",true);

	//(rad) The initial/default value of the wind direction
	default_wind_direction			 = iniFile.read_float(section.c_str(),"default_wind_direction",0,false);
	//(m/s) The initial/default value of the wind speed
	default_wind_speed				 = iniFile.read_float(section.c_str(),"default_wind_speed",0,false);

	//(rad) The noise in the wind direction
	std_windNoise_phi				 = iniFile.read_float(section.c_str(),"std_windNoise_phi",0,false);
	//(m/s) The noise in the wind strenght
	std_windNoise_mod				 = iniFile.read_float(section.c_str(),"std_windNoise_mod",0,false);

	//(m/s) The noise in the wind strenght
	advectionFreq				 = iniFile.read_float(section.c_str(),"advectionFreq",1,true);
}


/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(outObj);
	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getAs3DObject(
	mrpt::opengl::CSetOfObjectsPtr	&meanObj,
	mrpt::opengl::CSetOfObjectsPtr	&varObj ) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(meanObj,varObj);
	MRPT_END
}

/*---------------------------------------------------------------
						getWindAs3DObject
---------------------------------------------------------------*/
void  CGasConcentrationGridMap2D::getWindAs3DObject( mrpt::opengl::CSetOfObjectsPtr &windObj) const
{
	//Return an arrow map of the wind state (module(color) and direction).
	float scale = 0.2f;
	size_t arrow_separation = 5;	//distance between arrows, expresed as times the cell resolution


	//map limits
	float x_min = getXMin();
	float x_max = getXMax();
	float y_min = getYMin();
	float y_max = getYMax();
	float resol = getResolution();


	//Ensure map dimensions match with wind map
	unsigned int wind_map_size = windGrid_direction.getSizeX() * windGrid_direction.getSizeY();
	ASSERT_( wind_map_size == windGrid_module.getSizeX() * windGrid_module.getSizeY() );
	if( m_map.size() != wind_map_size )
	{
		cout << " GAS MAP DIMENSIONS DO NOT MATCH WIND MAP " << endl;
		//mrpt::system::pause();
	}


	unsigned int	cx,cy;
	vector<float>	xs,ys;

	// xs: array of X-axis values
	xs.resize( floor((x_max-x_min)/(arrow_separation*resol)) );
	for (cx=0;cx<xs.size();cx++)
		xs[cx] = x_min + arrow_separation*resol * cx;

	// ys: array of X-axis values
	ys.resize( floor((y_max-y_min)/(arrow_separation*resol)) );
	for (cy=0;cy<ys.size();cy++)
		ys[cy] = y_min + arrow_separation*resol * cy;

	for (cy=0;cy<ys.size();cy++)
	{
		for (cx=0;cx<xs.size();cx++)
		{
			// Cell values [0,inf]:
			double dir_xy = *windGrid_direction.cellByPos( xs[cx],ys[cy] );
			double mod_xy = *windGrid_module.cellByPos( xs[cx],ys[cy] );


			mrpt::opengl::CArrowPtr obj = mrpt::opengl::CArrow::Create(
				xs[cx],ys[cy],0,
				xs[cx]+scale*cos(dir_xy),ys[cy]+scale*sin(dir_xy),0,
				1.15f*scale,0.3f*scale,0.35f*scale);

			float r,g,b;
			jet2rgb( mod_xy,r,g,b );
			obj->setColor(r,g,b);

			windObj->insert( obj );
		}
	}
}

/*---------------------------------------------------------------
						increaseUncertainty
---------------------------------------------------------------*/
void CGasConcentrationGridMap2D::increaseUncertainty(const double STD_increase_value)
{
	//Increase cell variance
//	unsigned int cx,cy;
//	double memory_retention;

	m_hasToRecoverMeanAndCov = true;
	for( size_t it=0; it<m_map.size(); it++ )
        {
		m_stackedCov(it,0) = m_stackedCov(it,0)+STD_increase_value;
	}

	//Update m_map.kf_std
	recoverMeanAndCov();

	//for (cy=0; cy<m_size_y; cy++)
 //   {
 //       for (cx=0; cx<m_size_x; cx++)
 //       {
	//		// Forgetting_curve --> memory_retention = exp(-time/memory_relative_strenght)
	//		memory_retention = exp(- mrpt::system::timeDifference(m_map[cx + cy*m_size_x].last_updated, now()) / memory_relative_strenght);
	//		//Update Uncertainty (STD)
	//		m_map[cx + cy*m_size_x].kf_std = 1 - ( (1-m_map[cx + cy*m_size_x].updated_std) * memory_retention );
 //       }
 //   }
}


/*---------------------------------------------------------------
						simulateAdvection
---------------------------------------------------------------*/
bool CGasConcentrationGridMap2D::simulateAdvection( const double &STD_increase_value )
{

	/* 1- Ensure we can use Wind Information
	-------------------------------------------------*/
	if(!insertionOptions.useWindInformation)
		return false;

	//Get time since last simulation
	double At = mrpt::system::timeDifference(timeLastSimulated, mrpt::system::now());
	cout << endl << " - At since last simulation = " << At << "seconds" << endl;
	//update time of last updated.
	timeLastSimulated = mrpt::system::now();


	/* 3- Build Transition Matrix (SA)
	  This Matrix contains the probabilities of each cell
	  to "be displaced" to other cells by the wind effect.
	------------------------------------------------------*/
	mrpt::utils::CTicTac tictac;
	size_t i=0,c=0;
	int cell_i_cx,cell_i_cy;
	float mu_phi, mu_r, mu_modwind;
	const size_t N = m_map.size();
	mrpt::math::CMatrix A(N,N);
	A.fill(0.0);
	//std::vector<double> row_sum(N,0.0);
	double *row_sum = (double*)calloc(N, sizeof(double));

	try
		{
		//Ensure map dimensions match with wind map
		unsigned int wind_map_size = windGrid_direction.getSizeX() * windGrid_direction.getSizeY();
		ASSERT_( wind_map_size == windGrid_module.getSizeX() * windGrid_module.getSizeY() );
		if( N != wind_map_size )
		{
			cout << " GAS MAP DIMENSIONS DO NOT MATCH WIND INFORMATION " << endl;
			//mrpt::system::pause();
		}

		tictac.Tic();

		//Generate Sparse Matrix of the wind advection SA
		for(i=0; i<N; i++)
		{
			//Cell_i indx and coordinates
			idx2cxcy(i,cell_i_cx,cell_i_cy);

			//Read dirwind value of cell i
			mu_phi = *windGrid_direction.cellByIndex(cell_i_cx,cell_i_cy);		//[0,2*pi]
			unsigned int phi_indx = round(mu_phi/LUT.phi_inc);

			//Read modwind value of cell i
			mu_modwind = *windGrid_module.cellByIndex(cell_i_cx,cell_i_cy);		//[0,inf)
			mu_r =  mu_modwind * At;
			if( mu_r > LUT.max_r)
				mu_r = LUT.max_r;
			unsigned int r_indx = round(mu_r/LUT.r_inc);

			//Evaluate LUT
			ASSERT_(phi_indx < LUT.phi_count);
			ASSERT_(r_indx < LUT.r_count);

			//define label
			vector<TGaussianCell> &cells_to_update = LUT_TABLE[phi_indx][r_indx];

			//Generate Sparse Matrix with the wind weights "SA"
			for( unsigned int ci=0; ci<cells_to_update.size(); ci++)
			{
				int final_cx = cell_i_cx + cells_to_update[ci].cx;
				int final_cy = cell_i_cy + cells_to_update[ci].cy;
				//Check if affected cells is within the map
				if( (final_cx>=0) && (final_cx<(int)getSizeX()) && (final_cy>=0) && (final_cy<(int)getSizeY()) )
				{
					int final_idx = final_cx + final_cy*getSizeX();

					// Add Value to SA Matrix
					if( cells_to_update[ci].value != 0.0 )
					{
						A(final_idx,i) = cells_to_update[ci].value;
						row_sum[final_idx] += cells_to_update[ci].value;
					}
				}
			}//end-for ci
		}//end-for cell i

		cout << " - SA matrix computed in " << tictac.Tac() << "s" << endl << endl;
	}
	catch( std::exception &e)
	{
		cout << " #########  EXCEPTION computing Transition Matrix (A) ##########\n: " << e.what() << endl;
		cout << "on cell i= " << i << "  c=" << c << endl << endl;
		return false;
	}


	/* Update Mean + Variance as a Gaussian Mixture
	------------------------------------------------*/
	try
	{
		tictac.Tic();
		//std::vector<double> new_means(N,0.0);
		double *new_means = (double*)calloc(N, sizeof(double));
		//std::vector<double> new_variances(N,0.0);
		double *new_variances = (double*)calloc(N, sizeof(double));

		for( size_t it_i=0; it_i<N; it_i++)
		{
			//--------
			// mean
			//--------
			for( size_t it_j=0; it_j<N; it_j++)
			{
				if (m_map[it_j].kf_mean!=0 && A(it_i,it_j)!=0)
				{
					if (row_sum[it_i] >= 1)
						new_means[it_i] += ( A(it_i,it_j)/row_sum[it_i] ) * m_map[it_j].kf_mean;
					else
						new_means[it_i] +=  A(it_i,it_j) * m_map[it_j].kf_mean;
				}
			}


			//----------
			// variance
			//----------
			//Consider special case (borders cells)
			if (row_sum[it_i] < 1)
				new_variances[it_i] = (1-row_sum[it_i]) * square(insertionOptions.KF_initialCellStd);

			for( size_t it_j=0; it_j<N; it_j++)
			{
				if (A(it_i,it_j)!=0)
				{
					if (row_sum[it_i] >= 1)
						new_variances[it_i] += ( A(it_i,it_j)/row_sum[it_i] ) * ( m_stackedCov(it_j,0) + square( m_map[it_j].kf_mean - new_means[it_i]) );
					else
						new_variances[it_i] += A(it_i,it_j) * ( m_stackedCov(it_j,0) + square( m_map[it_j].kf_mean - new_means[it_i]) );
				}
			}
		}

		//Update means and Cov of the Kalman filter state
		for( size_t it_i=0; it_i<N; it_i++)
		{
			m_map[it_i].kf_mean = new_means[it_i];		//means

			//Variances
			// Scale the Current Covariances with the new variances
			for (size_t it_j=0; it_j<N; it_j++)
			{
				m_stackedCov(it_i,it_j) = (m_stackedCov(it_i,it_j)/m_stackedCov(it_i,it_i) )* new_variances[it_i];	//variances
				m_stackedCov(it_j,it_i) = m_stackedCov(it_i,it_j);
			}
		}
		m_hasToRecoverMeanAndCov = true;
		recoverMeanAndCov();

		cout << " - Mean&Var updated in " << tictac.Tac() << "s" << endl;


		//Free Memory
		free(row_sum);
		free(new_means);
		free(new_variances);

	}
	catch( std::exception &e)
				{
		cout << " #########  EXCEPTION Updating Covariances ##########\n: " << e.what() << endl;
		cout << "on row i= " << i << "  column c=" << c << endl << endl;
		return false;
	}


	//cout << " Increasing general STD..." << endl;
	increaseUncertainty(STD_increase_value);

	return true;

}




/*---------------------------------------------------------------
						build_Gaussian_Wind_Grid
---------------------------------------------------------------*/

bool CGasConcentrationGridMap2D::build_Gaussian_Wind_Grid()
/** Builds a LookUp table with the values of the Gaussian Weights result of the wind advection
*   for a specific condition.
*
*	The LUT contains the values of the Gaussian Weigths and the references to the cell indexes to be applied.
*	Since the LUT is independent of the wind direction and angle, it generates the Gaussian Weights for different configurations
*	of wind angle and module values.
*
*	To increase precission, each cell of the grid is sub-divided in subcells of smaller size.

*	cell_i --> Cell origin (We consider our reference system in the bottom left corner of cell_i ).
			   Is the cell that contains the gas measurement which will be propagated by the wind.
			   The wind propagates in the shape of a 2D Gaussian with center in the target cell (cell_j)
*	cell_j --> Target cell. Is the cell where falls the center of the Gaussian that models the propagation of the gas comming from cell_i.
*/

					{

	cout << endl << "---------------------------------" << endl;
	cout << " BUILDING GAUSSIAN WIND WEIGHTS " << endl;
	cout << "---------------------------------" << endl << endl;


	//-----------------------------
	//          PARAMS
	//-----------------------------
	LUT.resolution = getResolution();														//resolution of the grid-cells (m)
	LUT.std_phi = insertionOptions.std_windNoise_phi;										//Standard Deviation in wind Angle (cte)
	LUT.std_r = insertionOptions.std_windNoise_mod / insertionOptions.advectionFreq;		//Standard Deviation in wind module (cte)
	std::string filename = format("Gaussian_Wind_Weights_res(%f)_stdPhi(%f)_stdR(%f).gz",LUT.resolution,LUT.std_phi,LUT.std_r);

	// Fixed Params:
	LUT.phi_inc = M_PIf/8;									//Increment in the wind Angle. (rad)
	LUT.phi_count = round(2*M_PI/LUT.phi_inc)+1;			//Number of angles to generate
	LUT.r_inc = 0.1f;										//Increment in the wind Module. (m)
	LUT.max_r = 2;											//maximum distance (m) to simulate
	LUT.r_count = round(LUT.max_r/LUT.r_inc)+1;				//Number of wind modules to simulate

	LUT.table = new vector<vector<vector<TGaussianCell> > >(LUT.phi_count, vector<vector<TGaussianCell> >(LUT.r_count,vector<TGaussianCell>()) );

	//LUT.table = new vector<vector<vector<vector<vector<TGaussianCell>>>>>(LUT.subcell_count, vector<vector<vector<vector<TGaussianCell>>>>(LUT.subcell_count, vector<vector<vector<TGaussianCell>>>(LUT.phi_count, vector<vector<TGaussianCell>>(LUT.r_count,vector<TGaussianCell>()) ) ) );

	//-----------------------------
	//    Check if file exists
	//-----------------------------

	cout << "Looking for file: " << filename.c_str() << endl;

	if( mrpt::system::fileExists(filename.c_str()) )
	{
		// file exists. Load lookUptable from file
		cout << "LookUp table found for this configuration. Loading..." << endl;
		return load_Gaussian_Wind_Grid_From_File();

	}
	else
	{
		// file does not exists. Generate LookUp table.
		cout << "LookUp table NOT found. Generating table..." << endl;

		bool debug = true;
		FILE *debug_file;

		if (debug)
		{
			debug_file = fopen ("simple_LUT.txt","w");
			fprintf(debug_file," phi_inc = %.4f \n r_inc = %.4f \n",LUT.phi_inc,LUT.r_inc);
			fprintf(debug_file," std_phi = %.4f \n std_r = %.4f \n",LUT.std_phi,LUT.std_r);
			fprintf(debug_file,"[ phi ] [ r ] ---> (cx,cy)=Value\n");
			fprintf(debug_file,"----------------------------------\n");
						}

				// For the different possible angles (phi)
				for (size_t phi_indx=0; phi_indx<LUT.phi_count; phi_indx++ )
				{
					// mean of the phi value
					float phi = phi_indx * LUT.phi_inc;

					//For the different and possibe wind modules (r)
					for (size_t r_indx=0; r_indx<LUT.r_count; r_indx++ )
					{
						//mean of the radius value
						float r = r_indx * LUT.r_inc;

						if (debug)
						{
							fprintf(debug_file, "\n[%.2f] [%.2f] ---> ",phi, r);
						}

						//Estimates Cell_i_position
						//unsigned int cell_i_cx = 0;
						//unsigned int cell_i_cy = 0;
						float cell_i_x = LUT.resolution/2.0;
						float cell_i_y = LUT.resolution/2.0;

						//Estimate target position according to the mean value of wind.
						//float x_final = cell_i_x + r*cos(phi);
						//float y_final = cell_i_y + r*sin(phi);

						//Determine cell_j coordinates respect to origin_cell
						//int cell_j_cx = static_cast<int>(floor( (x_final)/LUT.resolution ));
						//int cell_j_cy = static_cast<int>(floor( (y_final)/LUT.resolution ));
						//Center of cell_j
						//float cell_j_x = (cell_j_cx+0.5f)*LUT.resolution;
						//float cell_j_y = (cell_j_cy+0.5f)*LUT.resolution;
						//left bottom corner of cell_j
						//float cell_j_xmin = cell_j_x - LUT.resolution/2.0;
						//float cell_j_ymin = cell_j_y - LUT.resolution/2.0;



						/* ---------------------------------------------------------------------------------
							Generate bounding-box  (+/- 3std) to determine which cells to update
						---------------------------------------------------------------------------------*/
						std::vector<double> vertex_x, vertex_y;
						vertex_x.resize(14);
						vertex_y.resize(14);
						//Bounding-Box initialization
						double minBBox_x = 1000;
						double maxBBox_x = -1000;
						double minBBox_y = 1000;
						double maxBBox_y = -1000;

						//Consider special case for high uncertainty in PHI. The shape of the polygon is a donut.
						double std_phi_BBox = LUT.std_phi;
						if( std_phi_BBox > M_PI/3)
				{
							std_phi_BBox = M_PI/3;	//To avoid problems generating the bounding box. For std>pi/3 the shape is always a donut.
				}

						// Calculate bounding box limits
						size_t indx = 0;
						int sr=3;
						for( int sd=(-3); sd<=(3); sd++ )
						{
							vertex_x[indx] = cell_i_x + (r+sr*LUT.std_r)*cos(phi+sd*std_phi_BBox);
							if (vertex_x[indx] < minBBox_x)
								minBBox_x = vertex_x[indx];
							if (vertex_x[indx] > maxBBox_x)
								maxBBox_x = vertex_x[indx];

							vertex_y[indx] = cell_i_y + (r+sr*LUT.std_r)*sin(phi+sd*std_phi_BBox);
							if (vertex_y[indx] < minBBox_y)
								minBBox_y = vertex_y[indx];
							if (vertex_y[indx] > maxBBox_y)
								maxBBox_y = vertex_y[indx];

							indx++;
						}
						sr=-3;
						for( int sd=(3); sd>=(-3); sd-- )
			{
							vertex_x[indx] = cell_i_x + (r+sr*LUT.std_r)*cos(phi+sd*std_phi_BBox);
							if (vertex_x[indx] < minBBox_x)
								minBBox_x = vertex_x[indx];
							if (vertex_x[indx] > maxBBox_x)
								maxBBox_x = vertex_x[indx];

							vertex_y[indx] = cell_i_y + (r+sr*LUT.std_r)*sin(phi+sd*std_phi_BBox);
							if (vertex_y[indx] < minBBox_y)
								minBBox_y = vertex_y[indx];
							if (vertex_y[indx] > maxBBox_y)
								maxBBox_y = vertex_y[indx];

							indx++;
						}

						/* ------------------------------------------------------------------------
						   Determine range of cells to update according to the Bounding-Box limits.
						   Origin cell is cx=cy= 0   x[0,res), y[0,res)
						---------------------------------------------------------------------------*/
						int min_cx = static_cast<int>( floor(minBBox_x/LUT.resolution) );
						int max_cx = static_cast<int>( floor(maxBBox_x/LUT.resolution) );
						int min_cy = static_cast<int>( floor(minBBox_y/LUT.resolution) );
						int max_cy = static_cast<int>( floor(maxBBox_y/LUT.resolution) );

						int num_cells_affected = (max_cx-min_cx+1) * (max_cy-min_cy+1);

						if( num_cells_affected == 1 )
				{
							//Concentration of cell_i moves to cell_a (cx,cy)
							TGaussianCell gauss_info;
							gauss_info.cx = min_cx;		//since max_cx == min_cx
							gauss_info.cy = min_cy;
							gauss_info.value = 1;		//prob = 1

							//Add cell volume to LookUp Table
							LUT_TABLE[phi_indx][r_indx].push_back(gauss_info);

							if (debug)
							{
								//Save to file (debug)
								fprintf(debug_file, "(%d,%d)=%.4f",gauss_info.cx, gauss_info.cy, gauss_info.value);
							}
						}
						else
						{
							// Estimate volume of the Gaussian under each affected cell

							float subcell_pres = LUT.resolution/10;
							//Determine the number of subcells inside the Bounding-Box
							const int BB_x_subcells = (int) (floor( (maxBBox_x - minBBox_x)/subcell_pres ) +1 );
							const int BB_y_subcells = (int) (floor( (maxBBox_y - minBBox_y)/subcell_pres ) +1 );

							double subcell_pres_x = (maxBBox_x - minBBox_x)/BB_x_subcells;
							double subcell_pres_y = (maxBBox_y - minBBox_y)/BB_y_subcells;

							//Save the W value of each cell using a map
							std::map<std::pair<int,int>, float> w_values;
							std::map<std::pair<int,int>, float>::iterator it;
							float sum_w = 0;

							for(int scy=0; scy<BB_y_subcells; scy++)
					{
								for(int scx=0; scx<BB_x_subcells; scx++)
						{
									//P-Subcell coordinates (center of the p-subcell)
									float subcell_a_x = minBBox_x + (scx+0.5f)*subcell_pres_x;
									float subcell_a_y = minBBox_y + (scy+0.5f)*subcell_pres_y;

									//distance and angle between cell_i and subcell_a
									float r_ia = sqrt( square(subcell_a_x-cell_i_x) + square(subcell_a_y-cell_i_y) );
									float phi_ia = atan2(subcell_a_y-cell_i_y, subcell_a_x-cell_i_x);

									//Volume Approximation of subcell_a (Gaussian Bivariate)
									float w = (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia-phi)/square(LUT.std_phi) ) );
									w += (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia+2*M_PI-phi)/square(LUT.std_phi) ) );
									w += (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia-2*M_PI-phi)/square(LUT.std_phi) ) );

									//Since we work with a cell grid, approximate the weight of the gaussian by the volume of the subcell_a
									if (r_ia != 0.0)
										w = (w * (subcell_pres_x*subcell_pres_y)/r_ia);

									//Determine cell index of the current subcell
									int cell_cx = static_cast<int>( floor(subcell_a_x/LUT.resolution) );
									int cell_cy = static_cast<int>( floor(subcell_a_y/LUT.resolution) );

									//Save w value
									it = w_values.find(std::make_pair(cell_cx,cell_cy));
									if( it!=w_values.end() )	//already exists
										w_values[std::make_pair(cell_cx,cell_cy)] += w;
									else
										w_values[std::make_pair(cell_cx,cell_cy)] = w;

									sum_w = sum_w + w;
								}//end-for scx
							}//end-for scy


							//SAVE to LUT
							for(it= w_values.begin(); it != w_values.end(); it++)
							{
								float w_final = (it->second)/sum_w;	//normalization to 1

								if( w_final >= 0.001 )
								{
									//Save the weight of the gaussian volume for cell_a (cx,cy)
									TGaussianCell gauss_info;
									gauss_info.cx = it->first.first;
									gauss_info.cy = it->first.second;
									gauss_info.value = w_final;

									//Add cell volume to LookUp Table
									LUT_TABLE[phi_indx][r_indx].push_back(gauss_info);

									if (debug)
									{
										//Save to file (debug)
										fprintf(debug_file, "(%d,%d)=%.6f    ",gauss_info.cx, gauss_info.cy, gauss_info.value);
						}
								}
							}

							//OLD WAY

							/* ---------------------------------------------------------
							   Estimate the volume of the Gaussian on each affected cell
							//-----------------------------------------------------------*/
							//for(int cx=min_cx; cx<=max_cx; cx++)
							//{
							//	for(int cy=min_cy; cy<=max_cy; cy++)
							//	{
							//		// Coordinates of affected cell (center of the cell)
							//		float cell_a_x = (cx+0.5f)*LUT.resolution;
							//		float cell_a_y = (cy+0.5f)*LUT.resolution;
							//		float w_cell_a = 0.0;	//initial Gaussian value of cell afected

							//		// Estimate volume of the Gaussian under cell (a)
							//		// Partition each cell into (p x p) subcells and evaluate the gaussian.
							//		int p = 40;
							//		float subcell_pres = LUT.resolution/p;
							//		float cell_a_x_min = cell_a_x - LUT.resolution/2.0;
							//		float cell_a_y_min = cell_a_y - LUT.resolution/2.0;

							//
							//		for(int scy=0; scy<p; scy++)
							//		{
							//			for(int scx=0; scx<p; scx++)
							//			{
							//				//P-Subcell coordinates (center of the p-subcell)
							//				float subcell_a_x = cell_a_x_min + (scx+0.5f)*subcell_pres;
							//				float subcell_a_y = cell_a_y_min + (scy+0.5f)*subcell_pres;

							//				//distance and angle between cell_i and subcell_a
							//				float r_ia = sqrt( square(subcell_a_x-cell_i_x) + square(subcell_a_y-cell_i_y) );
							//				float phi_ia = atan2(subcell_a_y-cell_i_y, subcell_a_x-cell_i_x);

							//				//Volume Approximation of subcell_a (Gaussian Bivariate)
							//				float w = (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia-phi)/square(LUT.std_phi) ) );
							//				w += (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia+2*M_PI-phi)/square(LUT.std_phi) ) );
							//				w += (1/(2*M_PI*LUT.std_r*LUT.std_phi)) * exp(-0.5*( square(r_ia-r)/square(LUT.std_r) + square(phi_ia-2*M_PI-phi)/square(LUT.std_phi) ) );
							//
							//				//Since we work with a cell grid, approximate the weight of the gaussian by the volume of the subcell_a
							//				if (r_ia != 0.0)
							//					w_cell_a = w_cell_a + (w * square(subcell_pres)/r_ia);
							//			}//end-for scx
							//		}//end-for scy

							//		//Save the weight of the gaussian volume for cell_a (cx,cy)
							//		TGaussianCell gauss_info;
							//		gauss_info.cx = cx;
							//		gauss_info.cy = cy;
							//		gauss_info.value = w_cell_a;

							//		//Add cell volume to LookUp Table
							//		LUT_TABLE[phi_indx][r_indx].push_back(gauss_info);

							//		if (debug)
							//		{
							//			//Save to file (debug)
							//			fprintf(debug_file, "(%d,%d)=%.6f    ",gauss_info.cx, gauss_info.cy, gauss_info.value);
							//		}
							//
							//
							//	}//end-for cy
							//}//end-for cx

						}//end-if only one affected cell

					}//end-for r
				}//end-for phi

		if (debug)
			fclose(debug_file);

		//Save LUT to File
		return save_Gaussian_Wind_Grid_To_File();

	}//end-if table not available
					}


bool CGasConcentrationGridMap2D::save_Gaussian_Wind_Grid_To_File()
{
	// Save LUT to file
	cout << "Saving to File ....";

	CFileGZOutputStream	f( format("Gaussian_Wind_Weights_res(%f)_stdPhi(%f)_stdR(%f).gz",LUT.resolution,LUT.std_phi,LUT.std_r) );

	if( !f.fileOpenCorrectly() )
	{
		return false;
		cout << "WARNING: Gaussian_Wind_Weights file NOT SAVED" << endl;
}

	try
{
		//Save params first
		f << LUT.resolution;	//cell resolution used
		f << LUT.std_phi;		//std_phi used
		f << LUT.std_r;


		f << LUT.phi_inc;	//rad
		f << (float)LUT.phi_count;
		f << LUT.r_inc;	//m
		f << LUT.max_r;	//maximum distance (m)
		f << (float)LUT.r_count;

		//Save Multi-table
		//vector< vector< vector<TGaussianCell>>>>> *table;

		for (size_t phi_indx=0; phi_indx<LUT.phi_count; phi_indx++ )
		{
			for (size_t r_indx=0; r_indx<LUT.r_count; r_indx++ )
			{
				//save all cell values.
				size_t N = LUT_TABLE[phi_indx][r_indx].size();
				f << (float)N;

				for (size_t i=0; i<N; i++)
    {
					f << (float)LUT_TABLE[phi_indx][r_indx][i].cx;
					f << (float)LUT_TABLE[phi_indx][r_indx][i].cy;
					f << LUT_TABLE[phi_indx][r_indx][i].value;
        }
    }
}
		cout << "DONE" << endl;
		f.close();
		return true;
	}
	catch(exception e)
	{
		cout << endl << "------------------------------------------------------------" << endl;
		cout << "EXCEPTION WHILE SAVING LUT TO FILE" << endl;
		cout << "Exception = " << e.what() << endl;
		f.close();
		return false;
	}

}

bool CGasConcentrationGridMap2D::load_Gaussian_Wind_Grid_From_File()
{
	// LOAD LUT from file
	cout << "Loading from File ....";

	try
    {
		CFileGZInputStream	f( format("Gaussian_Wind_Weights_res(%f)_stdPhi(%f)_stdR(%f).gz",LUT.resolution,LUT.std_phi,LUT.std_r) );

		if( !f.fileOpenCorrectly() )
        {
			cout << "WARNING WHILE READING FROM: Gaussian_Wind_Weights" << endl;
			return false;
        }

		float t_float;
		unsigned int t_uint;
		// Ensure params from file are correct with the specified in the ini file
		f >> t_float;
		ASSERT_(LUT.resolution == t_float)

		f >> t_float;
		ASSERT_(LUT.std_phi == t_float);

		f >> t_float;
		ASSERT_(LUT.std_r == t_float);

		f >> t_float;
		ASSERT_(LUT.phi_inc == t_float);

		f >> t_float;
		t_uint = (unsigned int)t_float;
		ASSERT_(LUT.phi_count == t_uint);

		f >> t_float;
		ASSERT_(LUT.r_inc == t_float);

		f >> t_float;
		ASSERT_(LUT.max_r == t_float);

		f >> t_float;
		t_uint = (unsigned int)t_float;
		ASSERT_(LUT.r_count == t_uint);

		// Load Multi-table
		// vector< vector< vector<TGaussianCell>>>>> *table;

				for (size_t phi_indx=0; phi_indx<LUT.phi_count; phi_indx++ )
				{
					for (size_t r_indx=0; r_indx<LUT.r_count; r_indx++ )
					{
						//Number of cells to update
						size_t N;
						f >> t_float;
						N = (size_t)t_float;

						for (size_t i=0; i<N; i++)
						{
							TGaussianCell gauss_info;
							f >> t_float;
							gauss_info.cx = (int)t_float;

							f >> t_float;
							gauss_info.cy = (int)t_float;

							f >> gauss_info.value;

							//Add cell volume to LookUp Table
							LUT_TABLE[phi_indx][r_indx].push_back(gauss_info);
    }
}
				}
		cout << "DONE" << endl;
		return true;
	}
	catch (exception e)
	{
		cout << endl << "------------------------------------------------------------" << endl;
		cout << "EXCEPTION WHILE LOADING LUT FROM FILE" << endl;
		cout << "Exception = " << e.what() << endl;
		return false;
	}
}
