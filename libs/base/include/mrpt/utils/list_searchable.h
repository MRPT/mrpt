/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef  list_searchable_H
#define  list_searchable_H

// Note: This file is included from "stl_extensions.h"

#include <list>

namespace mrpt
{
	namespace utils
	{
		using namespace mrpt::utils::metaprogramming;
		using std::for_each;
		using std::string;

		/** This class implements a STL container with features of both, a std::set and a std::list.
		 * \ingroup stlext_grp
		  */
		template <class T>
		class list_searchable : public std::list<T>
		{
		public:
			void insert( const T &o ) { std::list<T>::push_back(o); }

			typename std::list<T>::iterator find( const T& i ) {
				return std::find(std::list<T>::begin(),std::list<T>::end(),i);
			}

			typename std::list<T>::const_iterator find( const T& i ) const {
				return std::find(std::list<T>::begin(),std::list<T>::end(),i);
			}

			/** Finds an element in a list of smart pointers, having "->pointer()", such as it matches a given plain pointer "ptr". */
			template <typename PTR>
			typename std::list<T>::iterator find_ptr_to( const PTR ptr )
			{
				for (typename std::list<T>::iterator it=std::list<T>::begin();it!=std::list<T>::end();it++)
					if (it->pointer()==ptr)
						return it;
				return std::list<T>::end();
			}

			/** Finds an element in a list of smart pointers, having "->pointer()", such as it matches a given plain pointer "ptr". */
			template <typename PTR>
			typename std::list<T>::const_iterator find_ptr_to( const PTR ptr ) const
			{
				for (typename std::list<T>::const_iterator it=std::list<T>::begin();it!=std::list<T>::end();it++)
					if (it->pointer()==ptr)
						return it;
				return std::list<T>::end();
			}

		};

	} // End of namespace
} // End of namespace
#endif
