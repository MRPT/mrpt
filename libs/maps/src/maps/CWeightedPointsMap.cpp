/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/core/bits_mem.h>

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

CWeightedPointsMap::TMapDefinition::TMapDefinition() = default;
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
	std::ostream& out) const
{
	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CWeightedPointsMap::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CWeightedPointsMap::TMapDefinition& def =
		*dynamic_cast<const CWeightedPointsMap::TMapDefinition*>(&_def);
	auto* obj = new CWeightedPointsMap();
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CWeightedPointsMap, CPointsMap, mrpt::maps)

CWeightedPointsMap::CWeightedPointsMap() { reserve(400); }

void CWeightedPointsMap::reserve(size_t newLength)
{
	m_x.reserve(newLength);
	m_y.reserve(newLength);
	m_z.reserve(newLength);
	pointWeight.reserve(newLength);
}

// Resizes all point buffers so they can hold the given number of points: newly
// created points are set to default values,
//  and old contents are not changed.
void CWeightedPointsMap::resize(size_t newLength)
{
	m_x.resize(newLength, 0);
	m_y.resize(newLength, 0);
	m_z.resize(newLength, 0);
	pointWeight.resize(newLength, 1);
}

// Resizes all point buffers so they can hold the given number of points,
// *erasing* all previous contents
//  and leaving all points to default values.
void CWeightedPointsMap::setSize(size_t newLength)
{
	m_x.assign(newLength, 0);
	m_y.assign(newLength, 0);
	m_z.assign(newLength, 0);
	pointWeight.assign(newLength, 1);
}

void CWeightedPointsMap::insertPointFast(float x, float y, float z)
{
	m_x.push_back(x);
	m_y.push_back(y);
	m_z.push_back(z);
	this->pointWeight.push_back(1);
	// mark_as_modified(); -> Fast
}

void CWeightedPointsMap::impl_copyFrom(const CPointsMap& obj)
{
	// This also does a ::resize(N) of all data fields.
	CPointsMap::base_copyFrom(obj);

	const auto* pW = dynamic_cast<const CWeightedPointsMap*>(&obj);
	if (pW)
	{
		pointWeight = pW->pointWeight;
	}
}

/*---------------------------------------------------------------
						addFrom_classSpecific
 ---------------------------------------------------------------*/
void CWeightedPointsMap::addFrom_classSpecific(
	const CPointsMap& anotherMap, const size_t nPreviousPoints)
{
	const size_t nOther = anotherMap.size();

	// Specific data for this class:
	const auto* anotheMap_w =
		dynamic_cast<const CWeightedPointsMap*>(&anotherMap);

	if (anotheMap_w)
	{
		for (size_t i = 0, j = nPreviousPoints; i < nOther; i++, j++)
			pointWeight[j] = anotheMap_w->pointWeight[i];
	}
}

uint8_t CWeightedPointsMap::serializeGetVersion() const { return 2; }
void CWeightedPointsMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t n = m_x.size();

	// First, write the number of points:
	out << n;

	if (n > 0)
	{
		out.WriteBufferFixEndianness(&m_x[0], n);
		out.WriteBufferFixEndianness(&m_y[0], n);
		out.WriteBufferFixEndianness(&m_z[0], n);
		out.WriteBufferFixEndianness(&pointWeight[0], n);
	}

	out << genericMapParams;  // v2
	insertionOptions.writeToStream(
		out);  // version 9: insert options are saved with its own method
	likelihoodOptions.writeToStream(out);  // Added in version 5
}

void CWeightedPointsMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
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
				in.ReadBufferFixEndianness(&m_x[0], n);
				in.ReadBufferFixEndianness(&m_y[0], n);
				in.ReadBufferFixEndianness(&m_z[0], n);
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
	vector_strong_clear(m_x);
	vector_strong_clear(m_y);
	vector_strong_clear(m_z);
	vector_strong_clear(pointWeight);

	mark_as_modified();
}

namespace mrpt::maps::detail
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
}  // namespace mrpt::maps::detail
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
