/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** This class stores a message from a CAN BUS with the protocol J1939
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationCANBusJ1939 : public CObservation
{
  DEFINE_SERIALIZABLE(CObservationCANBusJ1939, mrpt::obs)

 public:
  /** Constructor.
   */
  CObservationCANBusJ1939() = default;

  /** Destructor
   */
  ~CObservationCANBusJ1939() override = default;
  /** The Parameter Group Number within this frame */
  uint16_t m_pgn{0};

  /** The address of the source node within this frame */
  uint8_t m_src_address{0};

  /** The priority */
  uint8_t m_priority{0};

  /** PDU Format */
  uint8_t m_pdu_format{0};

  /** PDU Specific */
  uint8_t m_pdu_spec{0};

  /** Data length  */
  uint8_t m_data_length{0};

  /** The data within this frame (0-8 bytes) */
  std::vector<uint8_t> m_data;

  /** The ASCII frame */
  std::vector<char> m_raw_frame;

  /** Not used */
  void getSensorPose(mrpt::poses::CPose3D&) const override {}
  void setSensorPose(const mrpt::poses::CPose3D&) override {}
  // See base class docs
  void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
