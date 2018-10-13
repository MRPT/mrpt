/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
/** Like calling a std::vector<>'s clear() method, but really forcing
 * deallocating the memory. */
template <class VECTOR_T>
inline void vector_strong_clear(VECTOR_T& v)
{
	VECTOR_T dummy;
	dummy.swap(v);
}
}  // namespace mrpt
