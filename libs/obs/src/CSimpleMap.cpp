/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
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
	ptr.first.reset(dynamic_cast<mrpt::poses::CPose3DPDF*>(ptr.first->clone()));
	ptr.second.reset(
		dynamic_cast<mrpt::obs::CSensoryFrame*>(ptr.second->clone()));
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

size_t CSimpleMap::size() const { return m_posesObsPairs.size(); }
bool CSimpleMap::empty() const { return m_posesObsPairs.empty(); }
void CSimpleMap::clear() { m_posesObsPairs.clear(); }
void CSimpleMap::get(
	size_t index, CPose3DPDF::Ptr& out_posePDF,
	CSensoryFrame::Ptr& out_SF) const
{
	if (index >= m_posesObsPairs.size()) THROW_EXCEPTION("Index out of bounds");
	out_posePDF = m_posesObsPairs[index].first;
	out_SF = m_posesObsPairs[index].second;
}

void CSimpleMap::remove(size_t index)
{
	MRPT_START
	if (index >= m_posesObsPairs.size()) THROW_EXCEPTION("Index out of bounds");
	m_posesObsPairs.erase(m_posesObsPairs.begin() + index);
	MRPT_END
}

void CSimpleMap::set(
	size_t index, const CPose3DPDF::Ptr& in_posePDF,
	const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START
	if (index >= m_posesObsPairs.size()) THROW_EXCEPTION("Index out of bounds");
	if (in_posePDF) m_posesObsPairs[index].first = in_posePDF;
	if (in_SF) m_posesObsPairs[index].second = in_SF;
	MRPT_END
}

void CSimpleMap::set(
	size_t index, const CPosePDF::Ptr& in_posePDF,
	const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START

	if (index >= m_posesObsPairs.size()) THROW_EXCEPTION("Index out of bounds");

	if (in_posePDF)
		m_posesObsPairs[index].first =
			CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*in_posePDF));
	if (in_SF) m_posesObsPairs[index].second = in_SF;

	MRPT_END
}

void CSimpleMap::insert(
	const CPose3DPDF* in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = in_SF;
	pair.first =
		CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF->clone()));
	ASSERT_(pair.first);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void CSimpleMap::insert(
	const CPose3DPDF::Ptr& in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = in_SF;
	pair.first = in_posePDF;

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void CSimpleMap::insert(
	const CPose3DPDF* in_posePDF, const CSensoryFrame& in_SF)
{
	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = CSensoryFrame::Create(in_SF);
	pair.first =
		CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF->clone()));
	ASSERT_(pair.first);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void CSimpleMap::insert(const CPosePDF* in_posePDF, const CSensoryFrame& in_SF)
{
	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = CSensoryFrame::Create(in_SF);
	pair.first =
		CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF->clone()));
	ASSERT_(pair.first);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void CSimpleMap::insert(
	const CPosePDF* in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = in_SF;
	pair.first =
		CPose3DPDF::Ptr(dynamic_cast<CPose3DPDF*>(in_posePDF->clone()));
	ASSERT_(pair.first);

	m_posesObsPairs.push_back(pair);

	MRPT_END
}

/*---------------------------------------------------------------
						insert  2D
  ---------------------------------------------------------------*/
void CSimpleMap::insert(
	const CPosePDF::Ptr& in_posePDF, const CSensoryFrame::Ptr& in_SF)
{
	insert(CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*in_posePDF)), in_SF);
}

void CSimpleMap::insertToPos(
	size_t index, const CPose3DPDF::Ptr& in_posePDF,
	const CSensoryFrame::Ptr& in_SF)
{
	if (index >= m_posesObsPairs.size()) return;

	MRPT_START

	TPosePDFSensFramePair pair;

	pair.second = in_SF;
	pair.first = in_posePDF;

	auto iter = m_posesObsPairs.begin() + index;
	m_posesObsPairs.insert(iter, pair);

	MRPT_END
}

uint8_t CSimpleMap::serializeGetVersion() const { return 1; }
void CSimpleMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_posesObsPairs.size());
	for (const auto& p : m_posesObsPairs) out << *p.first << *p.second;
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
				in >> m_posesObsPairs[i].first >> m_posesObsPairs[i].second;
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
				in >> aux2Dpose >> m_posesObsPairs[i].second;
				m_posesObsPairs[i].first =
					CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*aux2Dpose));
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					changeCoordinatesOrigin
  ---------------------------------------------------------------*/
void CSimpleMap::changeCoordinatesOrigin(const CPose3D& newOrigin)
{
	for (auto& m_posesObsPair : m_posesObsPairs)
		m_posesObsPair.first->changeCoordinatesReference(newOrigin);
}

/** Save this object to a .simplemap binary file (compressed with gzip)
 * \sa loadFromFile
 * \return false on any error.
 */
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

/** Load the contents of this object from a .simplemap binary file (possibly
 * compressed with gzip)
 * \sa saveToFile
 * \return false on any error.
 */
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
