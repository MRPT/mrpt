/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
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
#include <mrpt/serialization/optional_serialization.h>

#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::poses;
using namespace std;
using namespace mrpt::serialization;
using namespace mrpt::serialization::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CSimpleMap, CSerializable, mrpt::maps)

const auto fn_pair_make_unique = [](auto& ptr)
{
  ptr.pose.reset(dynamic_cast<mrpt::poses::CPose3DPDF*>(ptr.pose->clone()));
  ptr.sf.reset(dynamic_cast<mrpt::obs::CSensoryFrame*>(ptr.sf->clone()));
};

CSimpleMap CSimpleMap::makeDeepCopy()
{
  CSimpleMap o = *this;
  for_each(o.m_keyframes.begin(), o.m_keyframes.end(), fn_pair_make_unique);
  return o;
}

void CSimpleMap::remove(size_t index)
{
  MRPT_START
  ASSERT_LT_(index, m_keyframes.size());
  m_keyframes.erase(m_keyframes.begin() + index);
  MRPT_END
}

void CSimpleMap::insert(const Keyframe& kf) { m_keyframes.emplace_back(kf); }

uint8_t CSimpleMap::serializeGetVersion() const { return 2; }
void CSimpleMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  out.WriteAs<uint32_t>(m_keyframes.size());
  for (const auto& p : m_keyframes)
  {
    ASSERT_(p.pose);
    ASSERT_(p.sf);
    out << *p.pose << *p.sf;
    out << p.localTwist;  // v2
  }
}

void CSimpleMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 2:
    case 1:
    {
      uint32_t i, n;
      clear();
      in >> n;
      m_keyframes.resize(n);
      for (i = 0; i < n; i++)
      {
        in >> m_keyframes[i].pose >> m_keyframes[i].sf;
        if (version >= 2) in >> m_keyframes[i].localTwist;
      }
    }
    break;
    case 0:
    {
      // There are 2D poses PDF instead of 3D: transform them:
      uint32_t i, n;
      clear();
      in >> n;
      m_keyframes.resize(n);
      for (i = 0; i < n; i++)
      {
        CPosePDF::Ptr aux2Dpose;
        in >> aux2Dpose >> m_keyframes[i].sf;
        m_keyframes[i].pose = CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*aux2Dpose));
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CSimpleMap::changeCoordinatesOrigin(const CPose3D& newOrigin)
{
  for (auto& m_posesObsPair : m_keyframes)
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
  catch (const std::exception& e)
  {
    std::cerr << "[CSimpleMap::loadFromFile]" << e.what() << std::endl;
    return false;
  }
}
