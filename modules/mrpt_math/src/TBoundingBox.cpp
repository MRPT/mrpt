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

#include <mrpt/math/TBoundingBox.h>
#include <mrpt/serialization/CArchive.h>  // impl of << operator

#include <type_traits>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TBoundingBox>);
static_assert(std::is_trivially_copyable_v<TBoundingBoxf>);

mrpt::serialization::CArchive& mrpt::math::operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TBoundingBoxf& bb)
{
  in >> bb.min >> bb.max;
  return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TBoundingBoxf& bb)
{
  out << bb.min << bb.max;
  return out;
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
    mrpt::serialization::CArchive& in, mrpt::math::TBoundingBox& bb)
{
  in >> bb.min >> bb.max;
  return in;
}

mrpt::serialization::CArchive& mrpt::math::operator<<(
    mrpt::serialization::CArchive& out, const mrpt::math::TBoundingBox& bb)
{
  out << bb.min << bb.max;
  return out;
}
