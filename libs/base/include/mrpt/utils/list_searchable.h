/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
