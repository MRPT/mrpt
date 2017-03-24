/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/round.h> // round()
#include <mrpt/system/os.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/utils/CStream.h>


using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CReflectivityGridMap2D,reflectivityMap", mrpt::maps::CReflectivityGridMap2D)

CReflectivityGridMap2D::TMapDefinition::TMapDefinition() :
	min_x(-10.0f),
	max_x(10.0f),
	min_y(-10.0f),
	max_y(10.0f),
	resolution(0.10f)
{
}

void CReflectivityGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double,   source,sSectCreation);

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
}

void CReflectivityGridMap2D::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x         , double);
	LOADABLEOPTS_DUMP_VAR(max_x         , double);
	LOADABLEOPTS_DUMP_VAR(min_y         , double);
	LOADABLEOPTS_DUMP_VAR(max_y         , double);
	LOADABLEOPTS_DUMP_VAR(resolution         , double);

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CReflectivityGridMap2D::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CReflectivityGridMap2D::TMapDefinition &def = *dynamic_cast<const CReflectivityGridMap2D::TMapDefinition*>(&_def);
	CReflectivityGridMap2D *obj = new CReflectivityGridMap2D(def.min_x,def.max_x,def.min_y,def.max_y,def.resolution);
	obj->insertionOptions  = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CReflectivityGridMap2D, CMetricMap,mrpt::maps)


// Lookup tables for log-odds
CLogOddsGridMapLUT<CReflectivityGridMap2D::cell_t>  CReflectivityGridMap2D::m_logodd_lut;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CReflectivityGridMap2D::CReflectivityGridMap2D(
	double x_min, double x_max,
	double y_min, double y_max,
	double resolution ) :
		CDynamicGrid<int8_t>( x_min,x_max,y_min,y_max,resolution ),
		insertionOptions()
{
	internal_clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::internal_clear()
{
	fill( m_logodd_lut.p2l(0.5) );
}

/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool  CReflectivityGridMap2D::isEmpty() const
{
	return false;
}

/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CReflectivityGridMap2D::internal_insertObservation(
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

	if ( IS_CLASS(obs, CObservationReflectivity ) )
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const CObservationReflectivity	*o = static_cast<const CObservationReflectivity*>( obs );

		CPose3D sensor_pose;
		sensor_pose.composeFrom(robotPose3D, o->sensorPose);

		// log-odd increment due to the observation:
		const cell_t logodd_observation = m_logodd_lut.p2l( o->reflectivityLevel );

		// Update cell, with saturation:
		cell_t *cell = cellByPos(sensor_pose.x(),sensor_pose.y());
		if (!cell)
		{
			// We need to resize the grid!
			const double new_x_min = std::min( m_x_min, sensor_pose.x());
			const double new_y_min = std::min( m_y_min, sensor_pose.y());
			const double new_x_max = std::min( m_x_max, sensor_pose.x());
			const double new_y_max = std::min( m_y_max, sensor_pose.y());

			const int8_t default_cell = m_logodd_lut.p2l(0.5);
			resize(new_x_min,new_x_max, new_y_min,new_y_max, default_cell , 2.0 /* addit. margin */ );

			// Now we should get the cell:
			cell = cellByPos(sensor_pose.x(),sensor_pose.y());

			ASSERTMSG_(cell!=NULL,"cell==NULL even after resizing grid!?")
		}

		const int cell_old = static_cast<int>(*cell);
		int cell_new = cell_old + static_cast<int>(logodd_observation);
		keep_min(cell_new, static_cast<int>( CELLTYPE_MAX)-1);
		keep_max(cell_new, static_cast<int>( CELLTYPE_MIN)+1);

		*cell = static_cast<cell_t>(cell_new);

		return true;	// Done!
	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CReflectivityGridMap2D::internal_computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_START

	if ( IS_CLASS(obs, CObservationReflectivity ) )
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const CObservationReflectivity	*o = static_cast<const CObservationReflectivity*>( obs );

		CPose3D sensor_pose;
		sensor_pose.composeFrom(takenFrom, o->sensorPose);

		cell_t *cell = cellByPos(sensor_pose.x(),sensor_pose.y());
		if (!cell)
			return 0; // out of the map..
		else
		{
			ASSERT_ABOVEEQ_(o->reflectivityLevel,0)
			ASSERT_BELOWEQ_(o->reflectivityLevel,1)
			return -0.5*square( ( m_logodd_lut.p2l(*cell) - o->reflectivityLevel )/o->sensorStdNoise);
		}
	}
	else
		return 0;

	MRPT_END
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 2;
	else
	{
		dyngridcommon_writeToStream(out);

		// Map cells:
		const uint32_t n = static_cast<uint32_t>(m_map.size());
		out << n;
		if (n)
			out.WriteBuffer(&m_map[0],n);

		// Save the insertion options:
		// out << insertionOptions.maxOccupancyUpdateCertainty;

		out << genericMapParams; // v1
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			dyngridcommon_readFromStream(in, version<2);

			// Map cells:
			uint32_t n;
			in >> n;
			m_map.resize(n);
			if (n)
				in.ReadBuffer(&m_map[0],n);

			// Load the insertion options:
			// in >> insertionOptions.maxOccupancyUpdateCertainty;

			if (version>=1) 
				in >> genericMapParams;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CReflectivityGridMap2D::TInsertionOptions::TInsertionOptions()
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CReflectivityGridMap2D::TInsertionOptions] ------------ \n\n");

	//LOADABLEOPTS_DUMP_VAR(maxOccupancyUpdateCertainty, float)

	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_UNUSED_PARAM(iniFile);
	MRPT_UNUSED_PARAM(section);
	//MRPT_LOAD_CONFIG_VAR( maxOccupancyUpdateCertainty,	float, iniFile, section )
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	// Text matrix:
	saveToTextFile( filNamePrefix + std::string("_probability.txt") );
}

/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void  CReflectivityGridMap2D::getAsImage(
	utils::CImage	&img,
	bool verticalFlip,
	bool forceRGB ) const
{
	if (!forceRGB)
	{	// 8bit gray-scale
		img.resize(m_size_x,m_size_y,1,true);
		const cell_t	*srcPtr = &m_map[0];
		unsigned char	*destPtr;
		for (unsigned int y=0;y<m_size_y;y++)
		{
			if (!verticalFlip)
					destPtr = img(0,m_size_y-1-y);
			else 	destPtr = img(0,y);
			for (unsigned int x=0;x<m_size_x;x++)
			{
				*destPtr++ = m_logodd_lut.l2p_255(*srcPtr++);
			}
		}
	}
	else
	{	// 24bit RGB:
		img.resize(m_size_x,m_size_y,3,true);
		const cell_t	*srcPtr = &m_map[0];
		unsigned char	*destPtr;
		for (unsigned int y=0;y<m_size_y;y++)
		{
			if (!verticalFlip)
					destPtr = img(0,m_size_y-1-y);
			else 	destPtr = img(0,y);
			for (unsigned int x=0;x<m_size_x;x++)
			{
				uint8_t c = m_logodd_lut.l2p_255(*srcPtr++);
				*destPtr++ = c;
				*destPtr++ = c;
				*destPtr++ = c;
			}
		}
	}
}


/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CReflectivityGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outSetOfObj ) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	MRPT_START

	opengl::CTexturedPlanePtr	outObj = opengl::CTexturedPlane::Create();

	outObj->setPlaneCorners(m_x_min,m_x_max,m_y_min,m_y_max);

	// Create the color & transparecy (alpha) images:
	CImage			imgColor(m_size_x,m_size_y,1);
	CImage			imgTrans(m_size_x,m_size_y,1);

	const cell_t		*srcPtr = &m_map[0];
	unsigned char		*destPtr_color;
	unsigned char		*destPtr_trans;

	for (unsigned int y=0;y<m_size_y;y++)
	{
		destPtr_color = imgColor(0,y);
		destPtr_trans = imgTrans(0,y);
		for (unsigned int x=0;x<m_size_x;x++)
		{
			uint8_t  cell255 = m_logodd_lut.l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t   auxC = (int8_t)((signed short)cell255)-128;
			*destPtr_trans++ = auxC>0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast( imgColor,imgTrans );
	outSetOfObj->insert( outObj );

	MRPT_END
}

float  CReflectivityGridMap2D::compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const
{
	MRPT_UNUSED_PARAM(otherMap); MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(params);
	return 0;
}
