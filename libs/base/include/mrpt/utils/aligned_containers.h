/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_aligned_containers_H
#define mrpt_aligned_containers_H

#include <vector>
#include <map>
#include <list>
#include <deque>

// Fwrd. decl: 
namespace Eigen { template<class T> class aligned_allocator; }

namespace mrpt
{
	/** Helper types for STL containers with Eigen memory allocators.  (in #include <mrpt/utils/aligned_containers.h>)  */
	template <class TYPE1,class TYPE2=TYPE1>
	struct aligned_containers
	{
		typedef std::pair<TYPE1,TYPE2> pair_t;
		typedef std::vector<TYPE1, Eigen::aligned_allocator<TYPE1> > vector_t;
		typedef std::deque<TYPE1, Eigen::aligned_allocator<TYPE1> > deque_t;
		typedef std::list<TYPE1, Eigen::aligned_allocator<TYPE1> > list_t;
		typedef std::map<TYPE1,TYPE2,std::less<TYPE1>,Eigen::aligned_allocator<std::pair<const TYPE1,TYPE2> > > map_t;
		typedef std::multimap<TYPE1,TYPE2,std::less<TYPE1>,Eigen::aligned_allocator<std::pair<const TYPE1,TYPE2> > > multimap_t;
	};
}

#endif

