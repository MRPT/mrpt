/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::poses;
using namespace std;
using namespace mrpt::serialization;
using namespace mrpt::serialization::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CSimpleMap, CSerializable, mrpt::maps)

const auto fn_pair_make_unique = [](auto& ptr) {
	ptr.pose.reset(dynamic_cast<mrpt::poses::CPose3DPDF*>(ptr.pose->clone()));
	ptr.sf.reset(dynamic_cast<mrpt::obs::CSensoryFrame*>(ptr.sf->clone()));
};

CSimpleMap::CSimpleMap(const CSimpleMap& o) : m_posesObsPairs(o.m_posesObsPairs)
{
	for_each(
		m_posesObsPairs.begin(), m_posesObsPairs.end(), fn_pair_make_unique);
}

CSimpleMap& CSimpleMap::operator=(const CSimpleMap& o)
{
	MRPT_START
	if (this == &o) return *this;  // It may be used sometimes

	m_posesObsPairs = o.m_posesObsPairs;
	for_each(
		m_posesObsPairs.begin(), m_posesObsPairs.end(), fn_pair_make_unique);

	return *this;
	MRPT_END
}

void CSimpleMap::remove(size_t index)
{
	MRPT_START
	ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
	m_posesObsPairs.erase(m_posesObsPairs.begin() + index);
	MRPT_END
}

void CSimpleMap::set(
	size_t index, const CPose3DPDF::Ptr& in_posePDF,
	const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START
	ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
	if (in_posePDF) m_posesObsPairs[index].pose = in_posePDF;
	if (in_SF) m_posesObsPairs[index].sf = in_SF;
	MRPT_END
}

void CSimpleMap::set(
	size_t index, const CPosePDF::Ptr& in_posePDF,
	const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START
	ASSERTMSG_(index < m_posesObsPairs.size(), "Index out of bounds");
	if (in_posePDF)
		m_posesObsPairs[index].pose =
			CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*in_posePDF));
	if (in_SF) m_posesObsPairs[index].sf = in_SF;

	MRPT_END
}

void CSimpleMap::insert(
	const CPose3DPDF::Ptr& in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START

	Pair pair;

	pair.sf = in_SF;
	pair.pose = in_posePDF;

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

void CSimpleMap::insert(
	const CPose3DPDF& in_posePDF, const CSensoryFrame& in_SF)
{
	MRPT_START

	Pair pair;

	pair.sf = CSensoryFrame::Create(in_SF);
	pair.pose = CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF.clone()));
	ASSERT_(pair.pose);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

void CSimpleMap::insert(const CPosePDF& in_posePDF, const CSensoryFrame& in_SF)
{
	MRPT_START

	Pair pair;

	pair.sf = CSensoryFrame::Create(in_SF);
	pair.pose = CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF.clone()));
	ASSERT_(pair.pose);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

void CSimpleMap::insert(
	const CPosePDF::Ptr& in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	insert(CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*in_posePDF)), in_SF);
}

uint8_t CSimpleMap::serializeGetVersion() const { return 1; }
void CSimpleMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_posesObsPairs.size());
	for (const auto& p : m_posesObsPairs)
	{
		ASSERT_(p.pose);
		ASSERT_(p.sf);
		out << *p.pose << *p.sf;
	}
}

void CSimpleMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 1:
		{
			uint32_t i, n;
			clear();
			in >> n;
			m_posesObsPairs.resize(n);
			for (i = 0; i < n; i++)
				in >> m_posesObsPairs[i].pose >> m_posesObsPairs[i].sf;
		}
		break;
		case 0:
		{
			// There are 2D poses PDF instead of 3D: transform them:
			uint32_t i, n;
			clear();
			in >> n;
			m_posesObsPairs.resize(n);
			for (i = 0; i < n; i++)
			{
				CPosePDF::Ptr aux2Dpose;
				in >> aux2Dpose >> m_posesObsPairs[i].sf;
				m_posesObsPairs[i].pose =
					CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*aux2Dpose));
			}
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CSimpleMap::changeCoordinatesOrigin(const CPose3D& newOrigin)
{
	for (auto& m_posesObsPair : m_posesObsPairs)
	{
		ASSERT_(m_posesObsPair.pose);
		m_posesObsPair.pose->changeCoordinatesReference(newOrigin);
	}
}

bool CSimpleMap::saveToFile(const std::string& filName) const
{
	try
	{
		mrpt::io::CFileGZOutputStream fo(filName);
		archiveFrom(fo) << *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

bool CSimpleMap::loadFromFile(const std::string& filName)
{
	try
	{
		mrpt::io::CFileGZInputStream fi(filName);
		archiveFrom(fi) >> *this;
		return true;
	}
	catch (...)
	{
		return false;
	}
}
