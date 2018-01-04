/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/config/CConfigFileBase.h>  // MRPT_LOAD_CONFIG_VAR()
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloudColoured.h>

//#include <mrpt/system/os.h>
//#include <mrpt/img/color_maps.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CHeightGridMap2D,heightMap,dem", mrpt::maps::CHeightGridMap2D)

CHeightGridMap2D::TMapDefinition::TMapDefinition()
	: min_x(-2),
	  max_x(2),
	  min_y(-2),
	  max_y(2),
	  resolution(0.10f),
	  mapType(CHeightGridMap2D::mrSimpleAverage)
{
}

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
	mrpt::utils::CStream& out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x, double);
	LOADABLEOPTS_DUMP_VAR(max_x, double);
	LOADABLEOPTS_DUMP_VAR(min_y, double);
	LOADABLEOPTS_DUMP_VAR(max_y, double);
	LOADABLEOPTS_DUMP_VAR(resolution, double);
	out << mrpt::format(
		"MAP TYPE                                  = %s\n",
		mrpt::utils::TEnumType<
			CHeightGridMap2D::TMapRepresentation>::value2name(mapType)
			.c_str());

	this->insertionOpts.dumpToTextStream(std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CHeightGridMap2D::TInsertionOptions] ------------ "
		"\n\n");
	out << mrpt::format(
		"filterByHeight                          = %c\n",
		filterByHeight ? 'y' : 'n');
	out << mrpt::format("z_min                                   = %f\n", z_min);
	out << mrpt::format("z_max                                   = %f\n", z_max);
	out << mrpt::format(
		"colormap                                = %s\n",
		colorMap == cmJET ? "jet" : "grayscale");
	out << mrpt::format("\n");
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
		opengl::CMesh::Ptr mesh = mrpt::make_aligned_shared<opengl::CMesh>();

		mesh->setGridLimits(m_x_min, m_x_max, m_y_min, m_y_max);

		mesh->setColor(0.4, 0.4, 0.4);

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
				Z.set_unsafe(x, y, c->h);
				mask.set_unsafe(x, y, c->w ? 1 : 0);
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
			mrpt::make_aligned_shared<mrpt::opengl::CPointCloudColoured>();
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
				ASSERTDEB_(c)
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
				"countObservedCells() not implemented for this mapType (!?)")
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
