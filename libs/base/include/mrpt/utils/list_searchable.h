/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  list_searchable_H
#define  list_searchable_H

#include <list>
#include <algorithm>

namespace mrpt
{
	namespace utils
	{
		using std::for_each;
		using std::string;

		/** This class implements a STL container with features of both, a std::set and a std::list.
		 * \note Defined in #include <mrpt/utils/list_searchable.h>
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
