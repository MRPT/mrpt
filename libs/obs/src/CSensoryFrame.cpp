/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>
#include <mrpt/system/os.h>
#include <iterator>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSensoryFrame, CSerializable, mrpt::obs)

CSensoryFrame::CSensoryFrame(const CSensoryFrame& o) : m_observations()
{
	*this = o;
}

CSensoryFrame& CSensoryFrame::operator=(const CSensoryFrame& o)
{
	MRPT_START
	clear();
	if (this == &o) return *this;  // It may be used sometimes
	m_observations = o.m_observations;
	m_cachedMap.reset();
	return *this;
	MRPT_END
}

void CSensoryFrame::clear()
{
	m_observations.clear();
	m_cachedMap.reset();
}

uint8_t CSensoryFrame::serializeGetVersion() const { return 2; }
void CSensoryFrame::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_observations.size());
	for (const auto& o : m_observations)
	{
		ASSERT_(o);
		out << *o;
	}
}

void CSensoryFrame::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	MRPT_START
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			uint32_t i, n;
			mrpt::system::TTimeStamp tempTimeStamp = INVALID_TIMESTAMP;

			clear();
			if (version < 2)  // ID was removed in version 2
			{
				uint32_t ID;
				in >> ID;
			}

			if (version == 0) in.ReadBufferFixEndianness(&tempTimeStamp, 1);

			in >> n;
			m_observations.resize(n);
			for_each(
				m_observations.begin(), m_observations.end(),
				mrpt::serialization::metaprogramming::ObjectReadFromStream(
					&in));

			if (version == 0)
				for (i = 0; i < n; i++)
					m_observations[i]->timestamp = tempTimeStamp;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

	m_cachedMap.reset();

	MRPT_END
}

/*---------------------------------------------------------------
						operator +=
  ---------------------------------------------------------------*/
void CSensoryFrame::operator+=(const CSensoryFrame& sf)
{
	MRPT_UNUSED_PARAM(sf);
	m_cachedMap.reset();
	for (auto it = begin(); it != end(); ++it)
	{
		CObservation::Ptr newObs = *it;
		newObs.reset(dynamic_cast<CObservation*>(newObs->clone()));
		m_observations.push_back(
			newObs);  // static_cast<CObservation*>( (*it)->clone()) );
	}
}

/*---------------------------------------------------------------
						operator +=
  ---------------------------------------------------------------*/
void CSensoryFrame::operator+=(const CObservation::Ptr& obs)
{
	m_cachedMap.reset();
	m_observations.push_back(obs);
}

/*---------------------------------------------------------------
					push_back
  ---------------------------------------------------------------*/
void CSensoryFrame::push_back(const CObservation::Ptr& obs)
{
	m_cachedMap.reset();
	m_observations.push_back(obs);
}

/*---------------------------------------------------------------
				insert
  ---------------------------------------------------------------*/
void CSensoryFrame::insert(const CObservation::Ptr& obs)
{
	m_cachedMap.reset();
	m_observations.push_back(obs);
}

/*---------------------------------------------------------------
				eraseByIndex
  ---------------------------------------------------------------*/
void CSensoryFrame::eraseByIndex(const size_t& idx)
{
	MRPT_START
	if (idx >= size())
		THROW_EXCEPTION_FMT(
			"Index %u out of range.", static_cast<unsigned>(idx));

	m_cachedMap.reset();
	auto it = begin() + idx;
	ASSERT_(!*it);
	// delete (*it);
	m_observations.erase(it);
	MRPT_END
}

/*---------------------------------------------------------------
					getObservationByIndex
  ---------------------------------------------------------------*/
CObservation::Ptr CSensoryFrame::getObservationByIndex(const size_t& idx) const
{
	MRPT_START
	if (idx >= size())
		THROW_EXCEPTION_FMT(
			"Index %u out of range.", static_cast<unsigned>(idx));

	auto it = begin() + idx;
	return *it;

	MRPT_END
}

/*---------------------------------------------------------------
					erase
  ---------------------------------------------------------------*/
CSensoryFrame::iterator CSensoryFrame::erase(const iterator& it)
{
	MRPT_START
	ASSERT_(it != end());
	m_cachedMap.reset();

	return m_observations.erase(it);
	MRPT_END
}

/*---------------------------------------------------------------
					getObservationBySensorLabel
  ---------------------------------------------------------------*/
CObservation::Ptr CSensoryFrame::getObservationBySensorLabel(
	const std::string& label, const size_t& idx) const
{
	MRPT_START

	size_t foundCount = 0;
	for (const auto& it : *this)
		if (!os::_strcmpi(it->sensorLabel.c_str(), label.c_str()))
			if (foundCount++ == idx) return it;

	return CObservation::Ptr();

	MRPT_END
}

/*---------------------------------------------------------------
						swap
  ---------------------------------------------------------------*/
void CSensoryFrame::swap(CSensoryFrame& sf)
{
	m_observations.swap(sf.m_observations);
	std::swap(m_cachedMap, sf.m_cachedMap);
}

/*---------------------------------------------------------------
						eraseByLabel
  ---------------------------------------------------------------*/
void CSensoryFrame::eraseByLabel(const std::string& label)
{
	for (auto it = begin(); it != end();)
	{
		if (!os::_strcmpi((*it)->sensorLabel.c_str(), label.c_str()))
		{
			it = erase(it);
		}
		else
			it++;
	}
	m_cachedMap.reset();
}

namespace mrpt::obs
{
// Tricky way to call to a library that depends on us, a sort of "run-time"
// linking: ptr_internal_build_points_map_from_scan2D is a functor in
// "mrpt-obs", set by "mrpt-maps" at its startup.
using scan2pts_functor = void (*)(
	const mrpt::obs::CObservation2DRangeScan& obs,
	mrpt::maps::CMetricMap::Ptr& out_map, const void* insertOps);
extern scan2pts_functor ptr_internal_build_points_map_from_scan2D;  // impl in
// CObservation2DRangeScan.cpp
}  // namespace mrpt::obs
/*---------------------------------------------------------------
						internal_buildAuxPointsMap
  ---------------------------------------------------------------*/
void CSensoryFrame::internal_buildAuxPointsMap(const void* options) const
{
	if (!ptr_internal_build_points_map_from_scan2D)
		throw std::runtime_error(
			"[CSensoryFrame::buildAuxPointsMap] ERROR: This function needs "
			"linking against mrpt-maps.\n");

	for (const auto& it : *this)
		if (IS_CLASS(it, CObservation2DRangeScan))
			(*ptr_internal_build_points_map_from_scan2D)(
				dynamic_cast<CObservation2DRangeScan&>(*it.get()), m_cachedMap,
				options);
}

bool CSensoryFrame::insertObservationsInto(
	mrpt::maps::CMetricMap* theMap, const CPose3D* robotPose) const
{
	bool anyone = false;
	for (const auto& it : *this)
		anyone |= it->insertObservationInto(theMap, robotPose);
	return anyone;
}
