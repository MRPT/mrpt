/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CWirelessPowerGridMap2D,wifiGrid", mrpt::maps::CWirelessPowerGridMap2D)

CWirelessPowerGridMap2D::TMapDefinition::TMapDefinition() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CWirelessPowerGridMap2D::mrKernelDM)
{
}

void CWirelessPowerGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, double ,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double ,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double ,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double ,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double ,   source,sSectCreation);
	mapType = source.read_enum<CWirelessPowerGridMap2D::TMapRepresentation>(sSectCreation,"mapType",mapType);

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
}

void CWirelessPowerGridMap2D::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CWirelessPowerGridMap2D::TMapRepresentation>::value2name(mapType).c_str() );
	LOADABLEOPTS_DUMP_VAR(min_x         , double );
	LOADABLEOPTS_DUMP_VAR(max_x         , double );
	LOADABLEOPTS_DUMP_VAR(min_y         , double );
	LOADABLEOPTS_DUMP_VAR(max_y         , double );
	LOADABLEOPTS_DUMP_VAR(resolution         , double );

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CWirelessPowerGridMap2D::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CWirelessPowerGridMap2D::TMapDefinition &def = *dynamic_cast<const CWirelessPowerGridMap2D::TMapDefinition*>(&_def);
	CWirelessPowerGridMap2D *obj = new CWirelessPowerGridMap2D(def.mapType,def.min_x,def.max_x,def.min_y,def.max_y,def.resolution );
	obj->insertionOptions  = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CWirelessPowerGridMap2D, CRandomFieldGridMap2D,mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CWirelessPowerGridMap2D::CWirelessPowerGridMap2D(
	TMapRepresentation	mapType,
	double x_min,
	double x_max,
	double y_min,
	double y_max,
	double resolution ) :
		CRandomFieldGridMap2D(mapType, x_min,x_max,y_min,y_max,resolution ),
		insertionOptions()
{
	// Set the grid to initial values (and adjusts the KF covariance matrix!)
	//  Also, calling clear() is mandatory to end initialization of our base class (read note in CRandomFieldGridMap2D::CRandomFieldGridMap2D)
	CMetricMap::clear();
}

CWirelessPowerGridMap2D::~CWirelessPowerGridMap2D()
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::internal_clear()
{
	// Just do the generic clear:
	CRandomFieldGridMap2D::internal_clear();

	// Anything else special for this derived class?
	// ...
}


/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CWirelessPowerGridMap2D::internal_insertObservation(
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

	if ( IS_CLASS(obs, CObservationWirelessPower ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationWirelessPower
		********************************************************************/
		const CObservationWirelessPower	*o = static_cast<const CObservationWirelessPower*>( obs );
		float						sensorReading;


		// Compute the 3D sensor pose in world coordinates:
		CPose2D		sensorPose = CPose2D(robotPose3D + o->sensorPoseOnRobot );

		sensorReading = o->power;

		// Normalization:
		sensorReading = (sensorReading - insertionOptions.R_min) /( insertionOptions.R_max - insertionOptions.R_min );

		// Update the gross estimates of mean/vars for the whole reading history (see IROS2009 paper):
		m_average_normreadings_mean = (sensorReading + m_average_normreadings_count*m_average_normreadings_mean)/(1+m_average_normreadings_count);
		m_average_normreadings_var  = (square(sensorReading - m_average_normreadings_mean) + m_average_normreadings_count*m_average_normreadings_var) /(1+m_average_normreadings_count);
		m_average_normreadings_count++;

		// Finally, do the actual map update with that value:
		this->insertIndividualReading(sensorReading, mrpt::math::TPoint2D(sensorPose.x(),sensorPose.y()) );

		return true;	// Done!

	} // end if "CObservationWirelessPower"

	return false;

	MRPT_END
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CWirelessPowerGridMap2D::internal_computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 5;
	else
	{
		dyngridcommon_writeToStream(out);

		// To ensure compatibility: The size of each cell:
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
void  CWirelessPowerGridMap2D::readFromStream(mrpt::utils::CStream &in, int version)
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

			// To ensure compatibility: The size of each cell:
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
CWirelessPowerGridMap2D::TInsertionOptions::TInsertionOptions()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CWirelessPowerGridMap2D::TInsertionOptions] ------------ \n\n");
	internal_dumpToTextStream_common(out);  // Common params to all random fields maps:

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	// Common data fields for all random fields maps:
	internal_loadFromConfigFile_common(iniFile,section);
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CWirelessPowerGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START
	if (!genericMapParams.enableSaveAs3DObject) return;
	CRandomFieldGridMap2D::getAs3DObject(outObj);
	MRPT_END
}
