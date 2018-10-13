/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::containers
{
/** \addtogroup containers_grp
 * @{ */

/** Copy all the elements in a container (vector, deque, list) into a different
 * one performing the appropriate typecasting.
 *  The target container is automatically resized to the appropriate size, and
 * previous contents are lost.
 *  This can be used to assign std::vector's of different types:
 * \code
 *   std::vector<int>    vi(10);
 *   std::vector<float>  vf;
 *   vf = vi;   // Compiler error
 *   mrpt::containers::copy_container_typecasting(v1,vf);  // Ok
 * \endcode
 */
template <typename src_container, typename dst_container>
inline void copy_container_typecasting(
	const src_container& src, dst_container& trg)
{
	trg.resize(src.size());
	auto i = src.begin();
	auto last = src.end();
	auto target = trg.begin();
	for (; i != last; ++i, ++target)
		*target = static_cast<typename dst_container::value_type>(*i);
}
/** @} */  // end of grouping
}  // namespace mrpt::containers
