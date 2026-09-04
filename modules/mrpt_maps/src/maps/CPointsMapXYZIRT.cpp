/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::maps;

IMPLEMENTS_SERIALIZABLE(CPointsMapXYZIRT, CGenericPointsMap, mrpt::maps)

uint8_t CPointsMapXYZIRT::serializeGetVersion() const { return 0; }

// Binary layout kept identical to the pre-deprecation implementation, so
// files this class writes stay loadable by itself and vice versa.
void CPointsMapXYZIRT::serializeTo(mrpt::serialization::CArchive& out) const
{
  const uint32_t n = static_cast<uint32_t>(m_x.size());
  out << n;
  if (n > 0)
  {
    out.WriteBufferFixEndianness(m_x.data(), n);
    out.WriteBufferFixEndianness(m_y.data(), n);
    out.WriteBufferFixEndianness(m_z.data(), n);
  }

  const auto* intensity = getPointsBufferRef_float_field(POINT_FIELD_INTENSITY);
  const uint32_t nI = intensity ? static_cast<uint32_t>(intensity->size()) : 0;
  out << nI;
  if (nI > 0) out.WriteBufferFixEndianness(intensity->data(), nI);

  const auto* ring = getPointsBufferRef_uint16_field(POINT_FIELD_RING_ID);
  const uint32_t nR = ring ? static_cast<uint32_t>(ring->size()) : 0;
  out << nR;
  if (nR > 0) out.WriteBufferFixEndianness(ring->data(), nR);

  const auto* time = getPointsBufferRef_float_field(POINT_FIELD_TIMESTAMP);
  const uint32_t nT = time ? static_cast<uint32_t>(time->size()) : 0;
  out << nT;
  if (nT > 0) out.WriteBufferFixEndianness(time->data(), nT);

  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
}

void CPointsMapXYZIRT::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      mark_as_modified();

      uint32_t n;
      in >> n;
      resize(n);
      if (n > 0)
      {
        in.ReadBufferFixEndianness(m_x.data(), n);
        in.ReadBufferFixEndianness(m_y.data(), n);
        in.ReadBufferFixEndianness(m_z.data(), n);
      }

      uint32_t nI;
      in >> nI;
      if (nI > 0)
      {
        ASSERT_EQUAL_(nI, n);
        registerField_float(POINT_FIELD_INTENSITY);
        in.ReadBufferFixEndianness(m_float_fields.at(POINT_FIELD_INTENSITY).data(), nI);
      }

      uint32_t nR;
      in >> nR;
      if (nR > 0)
      {
        ASSERT_EQUAL_(nR, n);
        registerField_uint16(POINT_FIELD_RING_ID);
        in.ReadBufferFixEndianness(m_uint16_fields.at(POINT_FIELD_RING_ID).data(), nR);
      }

      uint32_t nT;
      in >> nT;
      if (nT > 0)
      {
        ASSERT_EQUAL_(nT, n);
        registerField_float(POINT_FIELD_TIMESTAMP);
        in.ReadBufferFixEndianness(m_float_fields.at(POINT_FIELD_TIMESTAMP).data(), nT);
      }

      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}
