/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
namespace utils
{
/** Calls "delete" to free an object only if the pointer is not nullptr, then
 * set the pointer to NULL. */
template <class T>
void delete_safe(T*& ptr)
{
	if (ptr)
	{
		delete ptr;
		ptr = nullptr;
	}
}

/** Like calling a std::vector<>'s clear() method, but really forcing
 * deallocating the memory. */
template <class VECTOR_T>
inline void vector_strong_clear(VECTOR_T& v)
{
	VECTOR_T dummy;
	dummy.swap(v);
}

/** Returns the smaller number >=len such that it's a multiple of 4 */
template <typename T>
T length2length4N(T len)
{
	if (0 != (len & 0x03)) len += (4 - (len & 0x03));
	return len;
}

}  // End of namespace
}  // end of namespace
