/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CWeightedPointsMap.h>
//#include <mrpt/serialization/CArchive.h>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CWeightedPointsMap,weightedPointsMap", mrpt::maps::CWeightedPointsMap)

CWeightedPointsMap::TMapDefinition::TMapDefinition() {}
void CWeightedPointsMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CWeightedPointsMap::TMapDefinition::dumpToTextStream_map_specific(
	mrpt::utils::CStream& out) const
{
	this->insertionOpts.dumpToTextStreamstd::ostream& out, int* version) const
{
	if (version)
		*version = 2;
	else
	{
		uint32_t n = x.size();

		// First, write the number of points:
		out << n;

		if (n > 0)
		{
			out.WriteBufferFixEndianness(&x[0], n);
			out.WriteBufferFixEndianness(&y[0], n);
			out.WriteBufferFixEndianness(&z[0], n);
			out.WriteBufferFixEndianness(&pointWeight[0], n);
		}

		out << genericMapParams;  // v2
		insertionOptions.writeToStream(
			out);  // version 9: insert options are saved with its own method
		likelihoodOptions.writeToStream(out);  // Added in version 5
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void CWeightedPointsMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			mark_as_modified();

			// Read the number of points:
			uint32_t n;
			in >> n;

			this->resize(n);

			if (n > 0)
			{
				in.ReadBufferFixEndianness(&x[0], n);
				in.ReadBufferFixEndianness(&y[0], n);
				in.ReadBufferFixEndianness(&z[0], n);
				in.ReadBufferFixEndianness(&pointWeight[0], n);
			}

			if (version >= 1)
			{
				if (version >= 2)
					in >> genericMapParams;
				else
				{
					bool disableSaveAs3DObject;
					in >> disableSaveAs3DObject;
					genericMapParams.enableSaveAs3DObject =
						!disableSaveAs3DObject;
				}

				insertionOptions.readFromStream(in);  // version 9: insert
				// options are saved with
				// its own method
			}
			else
			{
				insertionOptions = TInsertionOptions();
				in >> insertionOptions.minDistBetweenLaserPoints >>
					insertionOptions.addToExistingPointsMap >>
					insertionOptions.also_interpolate >>
					insertionOptions.disableDeletion >>
					insertionOptions.fuseWithExisting >>
					insertionOptions.isPlanarMap >>
					insertionOptions.maxDistForInterpolatePoints;
				{
					bool disableSaveAs3DObject;
					in >> disableSaveAs3DObject;
					genericMapParams.enableSaveAs3DObject =
						!disableSaveAs3DObject;
				}
				in >> insertionOptions.horizontalTolerance;
			}

			likelihoodOptions.readFromStream(in);  // Added in version 5
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					Clear
  ---------------------------------------------------------------*/
void CWeightedPointsMap::internal_clear()
{
	// This swap() thing is the only way to really deallocate the memory.
	vector_strong_clear(x);
	vector_strong_clear(y);
	vector_strong_clear(z);
	vector_strong_clear(pointWeight);

	mark_as_modified();
}

namespace mrpt
{
namespace maps
{
namespace detail
{
using mrpt::maps::CWeightedPointsMap;

template <>
struct pointmap_traits<CWeightedPointsMap>
{
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan2D_init(
		CWeightedPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		MRPT_UNUSED_PARAM(lric);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan2D_prepareOneRange(
		CWeightedPointsMap& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		MRPT_UNUSED_PARAM(gz);
		MRPT_UNUSED_PARAM(lric);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan2D_postPushBack(
		CWeightedPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(lric);
		me.pointWeight.push_back(1);
	}

	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called only once before inserting
	 * points - this is the place to reserve memory in lric for extra working
	 * variables. */
	inline static void internal_loadFromRangeScan3D_init(
		CWeightedPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		MRPT_UNUSED_PARAM(lric);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data */
	inline static void internal_loadFromRangeScan3D_prepareOneRange(
		CWeightedPointsMap& me, const float gx, const float gy, const float gz,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		MRPT_UNUSED_PARAM(gx);
		MRPT_UNUSED_PARAM(gy);
		MRPT_UNUSED_PARAM(gz);
		MRPT_UNUSED_PARAM(lric);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called after each
	 * "{x,y,z}.push_back(...);" */
	inline static void internal_loadFromRangeScan3D_postPushBack(
		CWeightedPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(lric);
		me.pointWeight.push_back(1);
	}
	/** Helper method fot the generic implementation of
	 * CPointsMap::loadFromRangeScan(), to be called once per range data, at the
	 * end */
	inline static void internal_loadFromRangeScan3D_postOneRange(
		CWeightedPointsMap& me,
		mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
	{
		MRPT_UNUSED_PARAM(me);
		MRPT_UNUSED_PARAM(lric);
	}
};
}
}
}

/** See CPointsMap::loadFromRangeScan() */
void CWeightedPointsMap::loadFromRangeScan(
	const CObservation2DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CWeightedPointsMap>::
		templ_loadFromRangeScan(*this, rangeScan, robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void CWeightedPointsMap::loadFromRangeScan(
	const CObservation3DRangeScan& rangeScan, const CPose3D* robotPose)
{
	mrpt::maps::detail::loadFromRangeImpl<CWeightedPointsMap>::
		templ_loadFromRangeScan(*this, rangeScan, robotPose);
}

// ================================ PLY files import & export virtual methods
// ================================

/** In a base class, reserve memory to prepare subsequent calls to
 * PLY_import_set_vertex */
void CWeightedPointsMap::PLY_import_set_vertex_count(const size_t N)
{
	this->setSize(N);
}
