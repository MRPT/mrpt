/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <iterator>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

#include <mrpt/utils/metaprogramming.h>
using namespace mrpt::utils::metaprogramming;

IMPLEMENTS_SERIALIZABLE( CSensoryFrame, CSerializable, mrpt::obs )

/*---------------------------------------------------------------
						Default constructor
  ---------------------------------------------------------------*/
CSensoryFrame::CSensoryFrame() :
	m_cachedMap(),
	m_observations()
{
}

/*---------------------------------------------------------------
						Copy constructor
  ---------------------------------------------------------------*/
CSensoryFrame::CSensoryFrame( const CSensoryFrame &o) :
	m_observations()
{
	*this = o;
}

/*---------------------------------------------------------------
							Copy
  ---------------------------------------------------------------*/
CSensoryFrame & CSensoryFrame::operator =( const CSensoryFrame &o)
{
	MRPT_START

	clear();

	if (this == &o) return *this;		// It may be used sometimes

	m_observations = o.m_observations;

	m_cachedMap.clear(); 

	return *this;

	MRPT_END

}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CSensoryFrame::~CSensoryFrame()
{
	clear();
}


/*---------------------------------------------------------------
							clear
  ---------------------------------------------------------------*/
void  CSensoryFrame::clear()
{
	m_observations.clear();
	m_cachedMap.clear();
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CSensoryFrame::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		uint32_t		i,n;

		n = static_cast<uint32_t>(m_observations.size());
		out << n;
		for (i=0;i<n;i++)
			out << *m_observations[i];
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CSensoryFrame::readFromStream(mrpt::utils::CStream &in,int version)
{
	MRPT_START

	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			uint32_t	i,n;
			mrpt::system::TTimeStamp	tempTimeStamp= INVALID_TIMESTAMP;

			clear();
			if (version<2)		// ID was removed in version 2
			{
				uint32_t ID;
				in >> ID;
			}

			if (version==0)
				in.ReadBufferFixEndianness( &tempTimeStamp, 1);

			in >> n;
			m_observations.resize(n);
			for_each( m_observations.begin(), m_observations.end(), ObjectReadFromStream(&in) );

			if (version==0)
				for (i=0;i<n;i++)
					m_observations[i]->timestamp = tempTimeStamp;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

	m_cachedMap.clear();

	MRPT_END
}


/*---------------------------------------------------------------
						operator +=
  ---------------------------------------------------------------*/
void CSensoryFrame::operator += (const CSensoryFrame &sf)
{
	MRPT_UNUSED_PARAM(sf);
	m_cachedMap.clear();
	for (const_iterator it = begin();it!=end();++it)
	{
		CObservationPtr newObs = *it;
		newObs.make_unique();
		m_observations.push_back( newObs ); //static_cast<CObservation*>( (*it)->duplicate()) );
	}
}

/*---------------------------------------------------------------
						operator +=
  ---------------------------------------------------------------*/
void CSensoryFrame::operator += (const CObservationPtr &obs)
{
	m_cachedMap.clear();
	m_observations.push_back( obs );
}

/*---------------------------------------------------------------
					push_back
  ---------------------------------------------------------------*/
void CSensoryFrame::push_back(const CObservationPtr &obs)
{
	m_cachedMap.clear();
	m_observations.push_back( obs );
}

/*---------------------------------------------------------------
				insert
  ---------------------------------------------------------------*/
void CSensoryFrame::insert(const CObservationPtr &obs)
{
	m_cachedMap.clear();
	m_observations.push_back( obs );
}

/*---------------------------------------------------------------
				eraseByIndex
  ---------------------------------------------------------------*/
void CSensoryFrame::eraseByIndex(const size_t &idx)
{
	MRPT_START
	if (idx>=size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %u out of range.", static_cast<unsigned>(idx) );

	m_cachedMap.clear();
	iterator it = begin()+idx;
	ASSERT_(it->present());
	//delete (*it);
	m_observations.erase( it );
	MRPT_END
}


/*---------------------------------------------------------------
					getObservationByIndex
  ---------------------------------------------------------------*/
CObservationPtr CSensoryFrame::getObservationByIndex( const size_t &idx ) const
{
	MRPT_START
	if (idx>=size()) THROW_EXCEPTION_CUSTOM_MSG1("Index %u out of range.", static_cast<unsigned>(idx) );

	const_iterator it = begin()+idx;
	return *it;

	MRPT_END
}

/*---------------------------------------------------------------
					erase
  ---------------------------------------------------------------*/
CSensoryFrame::iterator CSensoryFrame::erase( const iterator &it)
{
	MRPT_START
	ASSERT_(it!=end())

	m_cachedMap.clear();

	return m_observations.erase(it);
	MRPT_END
}

/*---------------------------------------------------------------
					getObservationBySensorLabel
  ---------------------------------------------------------------*/
CObservationPtr CSensoryFrame::getObservationBySensorLabel(
	const std::string &label,
	const size_t &idx) const
{
	MRPT_START

	size_t  foundCount = 0;
	for (const_iterator it = begin();it!=end();++it)
		if ( !os::_strcmpi( (*it)->sensorLabel.c_str(), label.c_str() ) )
			if (foundCount++ == idx)
				return *it;

	return CObservationPtr();

	MRPT_END
}

/*---------------------------------------------------------------
						moveFrom
  ---------------------------------------------------------------*/
void CSensoryFrame::moveFrom( CSensoryFrame &sf )
{
	copy(sf.m_observations.begin(),sf.m_observations.end(), back_inserter(m_observations) );
	sf.m_observations.clear();
	m_cachedMap.clear();
}

/*---------------------------------------------------------------
						swap
  ---------------------------------------------------------------*/
void CSensoryFrame::swap( CSensoryFrame &sf )
{
	m_observations.swap(sf.m_observations);
	std::swap(m_cachedMap, sf.m_cachedMap);
}

/*---------------------------------------------------------------
						eraseByLabel
  ---------------------------------------------------------------*/
void CSensoryFrame::eraseByLabel(const std::string &label)
{
	for (iterator it = begin();it!=end();  )
	{
		if ( !os::_strcmpi( (*it)->sensorLabel.c_str(), label.c_str() ) )
		{
			it = erase(it);
		}
		else it++;
	}
	m_cachedMap.clear();
}

namespace mrpt
{
	namespace obs
	{
		// Tricky way to call to a library that depends on us, a sort of "run-time" linking:
		//  ptr_internal_build_points_map_from_scan2D is a functor in "mrpt-obs", set by "mrpt-maps" at its startup.
		extern void (*ptr_internal_build_points_map_from_scan2D)(const mrpt::obs::CObservation2DRangeScan &obs, mrpt::maps::CMetricMapPtr &out_map, const void *insertOps);
	}
}


/*---------------------------------------------------------------
						internal_buildAuxPointsMap
  ---------------------------------------------------------------*/
void CSensoryFrame::internal_buildAuxPointsMap( const void *options ) const
{
	if (!ptr_internal_build_points_map_from_scan2D)
		throw std::runtime_error("[CSensoryFrame::buildAuxPointsMap] ERROR: This function needs linking against mrpt-maps.\n");

	for (const_iterator it = begin();it!=end();++it)
		if (IS_CLASS(*it,CObservation2DRangeScan))
			(*ptr_internal_build_points_map_from_scan2D)( *((CObservation2DRangeScan*)it->pointer()),m_cachedMap, options);
}


bool  CSensoryFrame::insertObservationsInto( mrpt::maps::CMetricMap *theMap, const CPose3D *robotPose ) const
{
	bool	anyone = false;
	for (const_iterator it = begin();it!=end();++it)
		anyone|= (*it)->insertObservationInto(theMap, robotPose);
	return anyone;
}

