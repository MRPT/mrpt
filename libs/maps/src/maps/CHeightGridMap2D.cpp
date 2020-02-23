/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR()
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"mrpt::maps::CHeightGridMap2D,heightMap,dem", mrpt::maps::CHeightGridMap2D)

CHeightGridMap2D::TMapDefinition::TMapDefinition()

	= default;

void CHeightGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation =
		sectionNamePrefix + string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(min_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, double, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);
	mapType = source.read_enum<CHeightGridMap2D::TMapRepresentation>(
		sSectCreation, "mapType", mapType);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
}

void CHeightGridMap2D::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x, double);
	LOADABLEOPTS_DUMP_VAR(max_x, double);
	LOADABLEOPTS_DUMP_VAR(min_y, double);
	LOADABLEOPTS_DUMP_VAR(max_y, double);
	LOADABLEOPTS_DUMP_VAR(resolution, double);
	out << mrpt::format(
		"MAP TYPE                                  = %s\n",
		mrpt::typemeta::TEnumType<
			CHeightGridMap2D::TMapRepresentation>::value2name(mapType)
			.c_str());

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CHeightGridMap2D::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CHeightGridMap2D::TMapDefinition& def =
		*dynamic_cast<const CHeightGridMap2D::TMapDefinition*>(&_def);
	auto* obj = new CHeightGridMap2D(
		def.mapType, def.min_x, def.max_x, def.min_y, def.max_y,
		def.resolution);
	obj->insertionOptions = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CHeightGridMap2D, CMetricMap, mrpt::maps)

bool HEIGHTGRIDMAP_EXPORT3D_AS_MESH_value = true;

bool mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH()
{
	return HEIGHTGRIDMAP_EXPORT3D_AS_MESH_value;
}
void mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH(bool value)
{
	HEIGHTGRIDMAP_EXPORT3D_AS_MESH_value = value;
}

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHeightGridMap2D::CHeightGridMap2D(
	TMapRepresentation mapType, double x_min, double x_max, double y_min,
	double y_max, double resolution)
	: CDynamicGrid<THeightGridmapCell>(x_min, x_max, y_min, y_max, resolution),
	  insertionOptions(),
	  m_mapType(mapType)
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CHeightGridMap2D::internal_clear() { fill(THeightGridmapCell()); }
/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool CHeightGridMap2D::isEmpty() const { return false; }
bool CHeightGridMap2D::insertIndividualPoint(
	const double x, const double y, const double z,
	const CHeightGridMap2D_Base::TPointInsertParams& params)
{
	THeightGridmapCell* cell = cellByPos(x, y);
	if (!cell)
		return false;  // Out of the map: Ignore if we've not resized before.

	if (!insertionOptions.filterByHeight ||
		(z >= insertionOptions.z_min && z <= insertionOptions.z_max))
	{
		cell->u += z;
		cell->v += z * z;
		if (!cell->w)
		{
			cell->h = z;  // First observation
			cell->w = 1;
		}
		else
		{
			float W = cell->w++;  // W = N-1
			cell->h = (cell->h * W + z) / cell->w;
			if (W > 0)
				cell->var = 1 / (W) * (cell->v - pow(cell->u, 2) / cell->w);
		}
	}  // end if really inserted
	return true;
}

bool CHeightGridMap2D::internal_insertObservation(
	const CObservation& obs, const CPose3D* robotPose)
{
	return dem_internal_insertObservation(obs, robotPose);
}

/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double CHeightGridMap2D::internal_computeObservationLikelihood(
	const CObservation& obs, const CPose3D& takenFrom)
{
	MRPT_UNUSED_PARAM(obs);
	MRPT_UNUSED_PARAM(takenFrom);

	THROW_EXCEPTION("Not implemented yet!");
}

uint8_t CHeightGridMap2D::serializeGetVersion() const { return 3; }
void CHeightGridMap2D::serializeTo(mrpt::serialization::CArchive& out) const
{
	dyngridcommon_writeToStream(out);

	// To assure compatibility: The size of each cell:
	auto n = static_cast<uint32_t>(sizeof(THeightGridmapCell));
	out << n;

	// Save the map contents:
	n = static_cast<uint32_t>(m_map.size());
	out << n;
	for (const auto& it : m_map)
		out << it.h
			<< it.w;  // This was removed in version 1: << it->history_Zs;

	// Save the insertion options:
	out << uint8_t(m_mapType);

	out << insertionOptions.filterByHeight << insertionOptions.z_min
		<< insertionOptions.z_max;

	out << genericMapParams;  // v2
}

void CHeightGridMap2D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			dyngridcommon_readFromStream(in, version < 3);
			// To ensure compatibility: The size of each cell:
			uint32_t n;
			in >> n;
			ASSERT_(n == static_cast<uint32_t>(sizeof(THeightGridmapCell)));

			// Save the map contents:
			in >> n;
			m_map.resize(n);
			for (auto& it : m_map)
			{
				in >> it.h >> it.w;
				// Data member in version 0:
				if (version == 0)
				{
					std::multimap<mrpt::system::TTimeStamp, float> history_Zs;
					in >> history_Zs;  // Discarded now...
				}
			}

			// Insertion options:
			uint8_t ty;
			in >> ty;
			m_mapType = TMapRepresentation(ty);

			in >> insertionOptions.filterByHeight >> insertionOptions.z_min >>
				insertionOptions.z_max;

			if (version >= 2) in >> genericMapParams;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

CHeightGridMap2D::TInsertionOptions::TInsertionOptions()

	= default;

void CHeightGridMap2D::TInsertionOptions::dumpToTextStream(
	std::ostream& out) const
{
	out << "\n----------- [CHeightGridMap2D::TInsertionOptions] ------------ "
		   "\n\n";
	out << mrpt::format(
		"filterByHeight                          = %c\n",
		filterByHeight ? 'y' : 'n');
	out << mrpt::format(
		"z_min                                   = %f\n", z_min);
	out << mrpt::format(
		"z_max                                   = %f\n", z_max);
	out << mrpt::format(
		"colormap                                = %s\n",
		colorMap == cmJET ? "jet" : "grayscale");
	out << "\n";
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CHeightGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	MRPT_LOAD_CONFIG_VAR(filterByHeight, bool, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(z_min, float, iniFile, section)
	MRPT_LOAD_CONFIG_VAR(z_max, float, iniFile, section)
	string aux = iniFile.read_string(section, "colorMap", "jet");

	if (strCmp(aux, "jet"))
		colorMap = cmJET;
	else if (strCmp(aux, "grayscale"))
		colorMap = cmGRAYSCALE;
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CHeightGridMap2D::saveMetricMapRepresentationToFile(
	const std::string& filNamePrefix) const
{
	// Text matrix:
	saveToTextFile(filNamePrefix + std::string("_mean.txt"));
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void CHeightGridMap2D::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	if (HEIGHTGRIDMAP_EXPORT3D_AS_MESH_value)
	{
		opengl::CMesh::Ptr mesh = std::make_shared<opengl::CMesh>();

		mesh->setGridLimits(m_x_min, m_x_max, m_y_min, m_y_max);

		mesh->setColor(0.4f, 0.4f, 0.4f);

		mesh->enableWireFrame(true);
		mesh->enableColorFromZ(true, insertionOptions.colorMap /*cmJET*/);

		CMatrixFloat Z, mask;
		/*mesh->getZ(Z);

		mesh->getMask(mask);*/

		Z.setSize(m_size_x, m_size_y);
		mask.setSize(m_size_x, m_size_y);

		for (size_t x = 0; x < m_size_x; x++)
		{
			for (size_t y = 0; y < m_size_y; y++)
			{
				const THeightGridmapCell* c = cellByIndex(x, y);
				ASSERTDEB_(c);
				Z(x, y) = c->h;
				mask(x, y) = c->w ? 1 : 0;
			}
		}
		mesh->setZ(Z);
		mesh->setMask(mask);

		outObj->insert(mesh);
	}
	else
	{
		// As points:
		mrpt::opengl::CPointCloudColoured::Ptr obj =
			mrpt::opengl::CPointCloudColoured::Create();
		obj->setPointSize(2);

		// Find min/max:
		float z_min, z_max;
		float K;
		if (this->getMinMaxHeight(z_min, z_max))
			K = 1.0f / (z_max - z_min);
		else
			K = 1.0f;

		obj->reserve(m_size_x * m_size_y);
		for (size_t x = 0; x < m_size_x; x++)
			for (size_t y = 0; y < m_size_y; y++)
			{
				const THeightGridmapCell* c = cellByIndex(x, y);
				ASSERTDEB_(c);
				if (c->w)
				{
					float r, g, b;
					const float col_idx = (c->h - z_min) * K;
					colormap(
						insertionOptions.colorMap,  // cmJET, //cmGRAYSCALE,
						col_idx, r, g, b);
					obj->push_back(idx2x(x), idx2y(y), c->h, r, g, b);
				}
			}

		outObj->insert(obj);
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
			for (size_t i = 0; i < N; i++)
				if (m_map[i].w) obsCells++;
			return obsCells;
		}
		break;
		default:
			THROW_EXCEPTION(
				"countObservedCells() not implemented for this mapType (!?)");
	};
}

double CHeightGridMap2D::dem_get_resolution() const { return m_resolution; }
size_t CHeightGridMap2D::dem_get_size_x() const { return m_size_x; }
size_t CHeightGridMap2D::dem_get_size_y() const { return m_size_y; }
bool CHeightGridMap2D::dem_get_z_by_cell(
	const size_t cx, const size_t cy, double& z_out) const
{
	const THeightGridmapCell* cell = cellByIndex(cx, cy);
	if (cell && cell->w)
	{
		z_out = cell->h;
		return true;
	}
	else
		return false;
}
bool CHeightGridMap2D::dem_get_z(
	const double x, const double y, double& z_out) const
{
	const THeightGridmapCell* cell = cellByPos(x, y);
	if (cell && cell->w)
	{
		z_out = cell->h;
		return true;
	}
	else
	{
		return false;
	}
}
void CHeightGridMap2D::dem_update_map()
{
	// Nothing to do in this class: estimate is always up-to-date
}

float CHeightGridMap2D::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(params);
	return 0;
}
