/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
/** Like calling a std::vector<>'s clear() method, but really forcing
 * deallocating the memory.
 * \ingroup mrpt_core_grp
 */
template <class VECTOR_T>
inline void vector_strong_clear(VECTOR_T& v)
{
	VECTOR_T dummy;
	dummy.swap(v);
}
}  // namespace mrpt
