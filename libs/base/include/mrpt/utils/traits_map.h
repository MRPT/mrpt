/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_traits_maps_H
#define  mrpt_traits_maps_H

// File to be included from stl_extensions.h
#include <mrpt/utils/map_as_vector.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp
		  *  @{ */

		/** @name Trait helper classes for templatized selection of a std::map implementation
		    @{ */

		/**  Traits for using a std::map<> (sparse representation) \sa map_traits_map_as_vector */
		struct map_traits_stdmap {
			template <class KEY,class VALUE,class _LessPred = std::less<KEY>, class _Alloc = Eigen::aligned_allocator<std::pair<const KEY, VALUE> > >
			struct map : public std::map<KEY,VALUE,_LessPred,_Alloc> {
			};
		};

		/**  Traits for using a mrpt::utils::map_as_vector<> (dense, fastest representation) \sa map_traits_stdmap  */
		struct map_traits_map_as_vector	{
			template <class KEY,class VALUE,class _LessPred = std::less<KEY>, class _Alloc = Eigen::aligned_allocator<std::pair<const KEY, VALUE> > >
			struct map : public mrpt::utils::map_as_vector<KEY,VALUE> { };
		};

		/** @} */
		/** @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
