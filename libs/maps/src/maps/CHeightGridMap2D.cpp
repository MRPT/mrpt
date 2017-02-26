/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/utils/CConfigFileBase.h> // MRPT_LOAD_CONFIG_VAR()
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloudColoured.h>

//#include <mrpt/system/os.h>
//#include <mrpt/utils/color_maps.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CHeightGridMap2D,heightMap,dem", mrpt::maps::CHeightGridMap2D)

CHeightGridMap2D::TMapDefinition::TMapDefinition() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CHeightGridMap2D::mrSimpleAverage)
{
}

void CHeightGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix+string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double,   source,sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double,   source,sSectCreation);
	mapType = source.read_enum<CHeightGridMap2D::TMapRepresentation>(sSectCreation,"mapType",mapType);

	insertionOpts.loadFromConfigFile(source, sectionNamePrefix+string("_insertOpts") );
}

void CHeightGridMap2D::TMapDefinition::dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x         , double);
	LOADABLEOPTS_DUMP_VAR(max_x         , double);
	LOADABLEOPTS_DUMP_VAR(min_y         , double);
	LOADABLEOPTS_DUMP_VAR(max_y         , double);
	LOADABLEOPTS_DUMP_VAR(resolution         , double);
	out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CHeightGridMap2D::TMapRepresentation>::value2name(mapType).c_str() );

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CHeightGridMap2D::internal_CreateFromMapDefinition(const mrpt::maps::TMetricMapInitializer &_def)
{
	const CHeightGridMap2D::TMapDefinition &def = *dynamic_cast<const CHeightGridMap2D::TMapDefinition*>(&_def);
	CHeightGridMap2D *obj = new CHeightGridMap2D(def.mapType,def.min_x,def.max_x,def.min_y,def.max_y,def.resolution);
	obj->insertionOptions  = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========


IMPLEMENTS_SERIALIZABLE(CHeightGridMap2D, CMetricMap,mrpt::maps)
	
bool mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH = true;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHeightGridMap2D::CHeightGridMap2D(
	TMapRepresentation	mapType,
	double x_min, double x_max,
	double y_min, double y_max, 
	double resolution ) :
		CDynamicGrid<THeightGridmapCell>( x_min,x_max,y_min,y_max,resolution ),
		insertionOptions(),
		m_mapType(mapType)
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::internal_clear()
{
	fill( THeightGridmapCell() );
}

/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool  CHeightGridMap2D::isEmpty() const
{
	return false;
}

bool CHeightGridMap2D::insertIndividualPoint(const double x,const double y,const double z, const CHeightGridMap2D_Base::TPointInsertParams & params)
{
	THeightGridmapCell *cell = cellByPos(x,y);
	if(!cell) return false; // Out of the map: Ignore if we've not resized before.

	if (!insertionOptions.filterByHeight || (z>=insertionOptions.z_min && z<=insertionOptions.z_max ) )
	{
		cell->u += z;
		cell->v += z*z;
		if (!cell->w)
		{
			cell->h = z;	// First observation
			cell->w = 1;
		}
		else
		{
			float W = cell->w++;	// W = N-1
			cell->h = (cell->h*W + z)/cell->w;
			if (W > 0)
				cell->var = 1/(W) * (cell->v - pow(cell->u,2)/cell->w);
		}
	} // end if really inserted
	return true;
}

bool CHeightGridMap2D::internal_insertObservation(const CObservation *obs, const CPose3D *robotPose )
{
	return dem_internal_insertObservation(obs,robotPose);
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CHeightGridMap2D::internal_computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CHeightGridMap2D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		dyngridcommon_writeToStream(out);

		// To assure compatibility: The size of each cell:
		uint32_t n = static_cast<uint32_t>(sizeof( THeightGridmapCell ));
		out << n;

		// Save the map contents:
		n = static_cast<uint32_t>(m_map.size());
		out << n;
		for (vector<THeightGridmapCell>::const_iterator it=m_map.begin();it!=m_map.end();++it)
			out << it->h << it->w; // This was removed in version 1: << it->history_Zs;

		// Save the insertion options:
		out << uint8_t(m_mapType);

		out << insertionOptions.filterByHeight
			<< insertionOptions.z_min
			<< insertionOptions.z_max;

		out << genericMapParams; // v2
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CHeightGridMap2D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			dyngridcommon_readFromStream(in, version<3);
			// To ensure compatibility: The size of each cell:
			uint32_t	n;
			in >> n;
			ASSERT_( n == static_cast<uint32_t>( sizeof( THeightGridmapCell ) ));

			// Save the map contents:
			in >> n;
			m_map.resize(n);
			for (vector<THeightGridmapCell>::iterator it=m_map.begin();it!=m_map.end();++it)
			{
				in >> it->h >> it->w;
				// Data member in version 0:
				if (version==0)
				{
					std::multimap<mrpt::system::TTimeStamp,float>	history_Zs;
					in >> history_Zs; // Discarded now...
				}
			}

			// Insertion options:
			uint8_t	ty;
			in  >> ty;
			m_mapType = TMapRepresentation(ty);

			in  >> insertionOptions.filterByHeight
				>> insertionOptions.z_min
				>> insertionOptions.z_max;

			if (version>=2) 
				in >> genericMapParams;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CHeightGridMap2D::TInsertionOptions::TInsertionOptions() :
	filterByHeight				( false ),
	z_min						( -0.5  ),
	z_max						(  0.5  ),
	colorMap( cmJET )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CHeightGridMap2D::TInsertionOptions] ------------ \n\n");
	out.printf("filterByHeight                          = %c\n", filterByHeight ? 'y':'n');
	out.printf("z_min                                   = %f\n", z_min);
	out.printf("z_max                                   = %f\n", z_max);
	out.printf("colormap                                = %s\n", colorMap == cmJET ? "jet" : "grayscale");
	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR( filterByHeight,	bool, iniFile, section )
	MRPT_LOAD_CONFIG_VAR( z_min,			float, iniFile, section )
	MRPT_LOAD_CONFIG_VAR( z_max,			float, iniFile, section )
	string aux = iniFile.read_string(section, "colorMap", "jet");

	if(strCmp(aux,"jet") )
		colorMap = cmJET;
	else if(strCmp(aux,"grayscale") )
		colorMap = cmGRAYSCALE;
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	// Text matrix:
	saveToTextFile( filNamePrefix + std::string("_mean.txt") );
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CHeightGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	if (mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH)
	{
		opengl::CMeshPtr	mesh = opengl::CMesh::Create();

		mesh->setGridLimits(m_x_min,m_x_max, m_y_min, m_y_max);

		mesh->setColor(0.4, 0.4, 0.4 );

		mesh->enableWireFrame(true);
		mesh->enableColorFromZ(true, insertionOptions.colorMap /*cmJET*/);

		CMatrixFloat Z,mask;
		/*mesh->getZ(Z);

		mesh->getMask(mask);*/

		Z.setSize( m_size_x, m_size_y );
		mask.setSize( m_size_x, m_size_y );

		for (size_t x=0;x<m_size_x;x++)
		{
			for (size_t y=0;y<m_size_y;y++)
			{
				const THeightGridmapCell *c = cellByIndex(x,y);
				ASSERTDEB_(c);
				Z.set_unsafe( x, y, c->h );
				mask.set_unsafe( x, y, c->w ? 1:0 );
			}
		}
		mesh->setZ(Z);
		mesh->setMask(mask);

		outObj->insert( mesh );
	}
	else
	{
		// As points:
		mrpt::opengl::CPointCloudColouredPtr obj = mrpt::opengl::CPointCloudColoured::Create();
		obj->setPointSize(2);

		// Find min/max:
		float z_min,z_max;
		float K;
		if (this->getMinMaxHeight(z_min,z_max))
		     K = 1.0f/(z_max-z_min);
		else K = 1.0f;

		obj->reserve(m_size_x*m_size_y);
		for (size_t x=0;x<m_size_x;x++)
			for (size_t y=0;y<m_size_y;y++)
			{
				const THeightGridmapCell *c = cellByIndex(x,y);
				ASSERTDEB_(c)
				if (c->w)
				{
					float r,g,b;
					const float col_idx = (c->h-z_min)*K;
					colormap(
						insertionOptions.colorMap, //cmJET, //cmGRAYSCALE,
						col_idx, r,g,b );
					obj->push_back( idx2x(x),idx2y(y), c->h, r,g,b );
				}
			}

		outObj->insert( obj );
	}
}

/** Return the number of cells with at least one height data inserted. */
size_t CHeightGridMap2D::countObservedCells() const
{
	switch (m_mapType)
	{
	case mrSimpleAverage:
		{
			size_t obsCells = 0;
			const size_t N = m_map.size();
			for (size_t i=0;i<N;i++)
				if (m_map[i].w)
					obsCells++;
			return obsCells;
		}
		break;
	default: THROW_EXCEPTION("countObservedCells() not implemented for this mapType (!?)")
	};
}

double CHeightGridMap2D::dem_get_resolution() const {
	return m_resolution;
}
size_t CHeightGridMap2D::dem_get_size_x() const {
	return m_size_x;
}
size_t CHeightGridMap2D::dem_get_size_y() const {
	return m_size_y;
}
bool CHeightGridMap2D::dem_get_z_by_cell(const size_t cx, const size_t cy, double &z_out) const {
	const THeightGridmapCell *cell = cellByIndex(cx,cy);
	if (cell && cell->w) {
		z_out = cell->h;
		return true;
	} else return false;
}
bool CHeightGridMap2D::dem_get_z(const double x, const double y, double &z_out) const
{
	const THeightGridmapCell *cell = cellByPos(x,y);
	if (cell && cell->w) {
		z_out = cell->h;
		return true;
	} else {
		return false;
	}
}
void CHeightGridMap2D::dem_update_map() {
	// Nothing to do in this class: estimate is always up-to-date
}

float  CHeightGridMap2D::compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const
{
	MRPT_UNUSED_PARAM(otherMap); MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(params); 
	return 0;
}


