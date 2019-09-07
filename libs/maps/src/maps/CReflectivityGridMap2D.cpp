/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/core/round.h>  // round()
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::img;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CReflectivityGridMap2D,reflectivityMap",
	mrpt::maps::CReflectivityGridMap2D)

CReflectivityGridMap2D::TMapDefinition::TMapDefinition()

	= default;

void CReflectivityGridMap2D::TMapDefinition::loadFromConfigFile_map_specific(
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

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
}

void CReflectivityGridMap2D::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x, double);
	LOADABLEOPTS_DUMP_VAR(max_x, double);
	LOADABLEOPTS_DUMP_VAR(min_y, double);
	LOADABLEOPTS_DUMP_VAR(max_y, double);
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap*
	CReflectivityGridMap2D::internal_CreateFromMapDefinition(
		const mrpt::maps::TMetricMapInitializer& _def)
{
	const CReflectivityGridMap2D::TMapDefinition& def =
		*dynamic_cast<const CReflectivityGridMap2D::TMapDefinition*>(&_def);
	auto* obj = new CReflectivityGridMap2D(
		def.min_x, def.max_x, def.min_y, def.max_y, def.resolution);
	obj->insertionOptions = def.insertionOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CReflectivityGridMap2D, CMetricMap, mrpt::maps)

// Lookup tables for log-odds
CLogOddsGridMapLUT<CReflectivityGridMap2D::cell_t>
	CReflectivityGridMap2D::m_logodd_lut;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CReflectivityGridMap2D::CReflectivityGridMap2D(
	double x_min, double x_max, double y_min, double y_max, double resolution)
	: CDynamicGrid<int8_t>(x_min, x_max, y_min, y_max, resolution),
	  insertionOptions()
{
	internal_clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CReflectivityGridMap2D::internal_clear() { fill(m_logodd_lut.p2l(0.5)); }
/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool CReflectivityGridMap2D::isEmpty() const { return false; }
/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool CReflectivityGridMap2D::internal_insertObservation(
	const CObservation& obs, const CPose3D* robotPose)
{
	MRPT_START

	CPose2D robotPose2D;
	CPose3D robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if (IS_CLASS(obs, CObservationReflectivity))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const auto& o = dynamic_cast<const CObservationReflectivity&>(obs);

		if (o.channel != -1 && insertionOptions.channel != -1 &&
			o.channel != insertionOptions.channel)
		{
			return false;  // Incorrect channel
		}

		CPose3D sensor_pose;
		sensor_pose.composeFrom(robotPose3D, o.sensorPose);

		// log-odd increment due to the observation:
		const cell_t logodd_observation = m_logodd_lut.p2l(o.reflectivityLevel);

		// Update cell, with saturation:
		cell_t* cell = cellByPos(sensor_pose.x(), sensor_pose.y());
		if (!cell)
		{
			// We need to resize the grid!
			const double new_x_min = std::min(m_x_min, sensor_pose.x());
			const double new_y_min = std::min(m_y_min, sensor_pose.y());
			const double new_x_max = std::min(m_x_max, sensor_pose.x());
			const double new_y_max = std::min(m_y_max, sensor_pose.y());

			const int8_t default_cell = m_logodd_lut.p2l(0.5);
			resize(
				new_x_min, new_x_max, new_y_min, new_y_max, default_cell,
				2.0 /* addit. margin */);

			// Now we should get the cell:
			cell = cellByPos(sensor_pose.x(), sensor_pose.y());

			ASSERTMSG_(
				cell != nullptr, "cell==nullptr even after resizing grid!?");
		}

		const int cell_old = static_cast<int>(*cell);
		int cell_new = cell_old + static_cast<int>(logodd_observation);
		keep_min(cell_new, static_cast<int>(CELLTYPE_MAX) - 1);
		keep_max(cell_new, static_cast<int>(CELLTYPE_MIN) + 1);

		*cell = static_cast<cell_t>(cell_new);

		return true;  // Done!
	}  // end if "CObservationGasSensors"

	return false;

	MRPT_END
}

/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double CReflectivityGridMap2D::internal_computeObservationLikelihood(
	const CObservation& obs, const CPose3D& takenFrom)
{
	MRPT_START

	if (IS_CLASS(obs, CObservationReflectivity))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationReflectivity
		********************************************************************/
		const auto& o = dynamic_cast<const CObservationReflectivity&>(obs);

		if (o.channel != -1 && insertionOptions.channel != -1 &&
			o.channel != insertionOptions.channel)
		{
			return 0;  // Incorrect channel
		}

		CPose3D sensor_pose;
		sensor_pose.composeFrom(takenFrom, o.sensorPose);

		cell_t* cell = cellByPos(sensor_pose.x(), sensor_pose.y());
		if (!cell)
			return 0;  // out of the map..
		else
		{
			ASSERT_ABOVEEQ_(o.reflectivityLevel, 0);
			ASSERT_BELOWEQ_(o.reflectivityLevel, 1);
			return -0.5 * square(
							  (m_logodd_lut.l2p(*cell) - o.reflectivityLevel) /
							  o.sensorStdNoise);
		}
	}
	else
		return 0;

	MRPT_END
}

uint8_t CReflectivityGridMap2D::serializeGetVersion() const { return 3; }
void CReflectivityGridMap2D::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	dyngridcommon_writeToStream(out);

	// Map cells:
	const auto n = static_cast<uint32_t>(m_map.size());
	out << n;
	if (n) out.WriteBuffer(&m_map[0], n);

	// Save the insertion options
	out << insertionOptions.channel;  // v3

	out << genericMapParams;  // v1
}

void CReflectivityGridMap2D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			dyngridcommon_readFromStream(in, version < 2);

			// Map cells:
			uint32_t n;
			in >> n;
			m_map.resize(n);
			if (n) in.ReadBuffer(&m_map[0], n);

			// Load the insertion options:
			if (version >= 3) in >> insertionOptions.channel;

			if (version >= 1) in >> genericMapParams;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CReflectivityGridMap2D::TInsertionOptions::TInsertionOptions() = default;
void CReflectivityGridMap2D::TInsertionOptions::dumpToTextStream(
	std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CReflectivityGridMap2D::TInsertionOptions] "
		"------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(channel, int);

	out << mrpt::format("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void CReflectivityGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	MRPT_UNUSED_PARAM(iniFile);
	MRPT_UNUSED_PARAM(section);
	MRPT_LOAD_CONFIG_VAR(channel, int, iniFile, section);
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CReflectivityGridMap2D::saveMetricMapRepresentationToFile(
	const std::string& filNamePrefix) const
{
	// Text matrix:
	saveToTextFile(filNamePrefix + std::string("_probability.txt"));
}

/*---------------------------------------------------------------
					getAsImage
  ---------------------------------------------------------------*/
void CReflectivityGridMap2D::getAsImage(
	CImage& img, bool verticalFlip, bool forceRGB) const
{
	if (!forceRGB)
	{  // 8bit gray-scale
		img.resize(m_size_x, m_size_y, CH_GRAY);
		const cell_t* srcPtr = &m_map[0];
		unsigned char* destPtr;
		for (unsigned int y = 0; y < m_size_y; y++)
		{
			if (!verticalFlip)
				destPtr = img(0, m_size_y - 1 - y);
			else
				destPtr = img(0, y);
			for (unsigned int x = 0; x < m_size_x; x++)
			{
				*destPtr++ = m_logodd_lut.l2p_255(*srcPtr++);
			}
		}
	}
	else
	{  // 24bit RGB:
		img.resize(m_size_x, m_size_y, CH_RGB);
		const cell_t* srcPtr = &m_map[0];
		unsigned char* destPtr;
		for (unsigned int y = 0; y < m_size_y; y++)
		{
			if (!verticalFlip)
				destPtr = img(0, m_size_y - 1 - y);
			else
				destPtr = img(0, y);
			for (unsigned int x = 0; x < m_size_x; x++)
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
void CReflectivityGridMap2D::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outSetOfObj) const
{
	if (!genericMapParams.enableSaveAs3DObject) return;

	MRPT_START

	opengl::CTexturedPlane::Ptr outObj =
		std::make_shared<opengl::CTexturedPlane>();

	outObj->setPlaneCorners(m_x_min, m_x_max, m_y_min, m_y_max);

	// Create the color & transparecy (alpha) images:
	CImage imgColor(m_size_x, m_size_y, CH_GRAY);
	CImage imgTrans(m_size_x, m_size_y, CH_GRAY);

	const cell_t* srcPtr = &m_map[0];
	unsigned char* destPtr_color;
	unsigned char* destPtr_trans;

	for (unsigned int y = 0; y < m_size_y; y++)
	{
		destPtr_color = imgColor(0, y);
		destPtr_trans = imgTrans(0, y);
		for (unsigned int x = 0; x < m_size_x; x++)
		{
			uint8_t cell255 = m_logodd_lut.l2p_255(*srcPtr++);
			*destPtr_color++ = cell255;

			int8_t auxC = (int8_t)((signed short)cell255) - 128;
			*destPtr_trans++ = auxC > 0 ? (auxC << 1) : ((-auxC) << 1);
		}
	}

	outObj->assignImage_fast(imgColor, imgTrans);
	outSetOfObj->insert(outObj);

	MRPT_END
}

float CReflectivityGridMap2D::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(params);
	return 0;
}
