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
#pragma once
#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/serialization/CArchive.h>

namespace mrpt::serialization
{
CArchive& operator>>(CArchive& s, mrpt::aligned_std_vector<float>& a);
CArchive& operator<<(CArchive& s, const mrpt::aligned_std_vector<float>& a);
CArchive& operator>>(CArchive& s, std::vector<float>& a);
CArchive& operator<<(CArchive& s, const std::vector<float>& a);
}  // namespace mrpt::serialization
