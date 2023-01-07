/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
