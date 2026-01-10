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

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/archiveFrom_std_vector.h>

using namespace mrpt;
using namespace mrpt::serialization;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CSerializable, CObject, mrpt::serialization)

void mrpt::serialization::ObjectToOctetVector(
    const CSerializable* o, std::vector<uint8_t>& out_vector)
{
  out_vector.clear();
  auto arch = archiveFrom(out_vector);
  arch << *o;
}

void mrpt::serialization::OctetVectorToObject(
    const std::vector<uint8_t>& in_data, CSerializable::Ptr& obj)
{
  obj.reset();

  if (in_data.empty()) return;
  auto arch = archiveFrom(in_data);
  obj = arch.ReadObject();
}
