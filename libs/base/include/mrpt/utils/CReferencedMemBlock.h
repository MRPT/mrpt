/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CReferencedMemBlock_H
#define  CReferencedMemBlock_H

#include <mrpt/base/link_pragmas.h>
#include <vector>
#include <utility>

// STL+ library:
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mrpt
{
	namespace utils
	{
		/** Represents a memory block (via "void*") that can be shared between several objects through copy operator (=).
		  *  It keeps the reference count and only when it comes to zero, the memory block is really freed.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CReferencedMemBlock : public stlplus::smart_ptr< std::vector<char> >
		{
		typedef stlplus::smart_ptr< std::vector<char> > base_t;
		public:
			/** Constructor with an optional size of the memory block */
			CReferencedMemBlock(size_t mem_block_size = 0 );

			/** Destructor, calls dereference_once. */
			virtual ~CReferencedMemBlock();

			/** Resize the shared memory block. */
			void resize(size_t mem_block_size );

			template <class T> T getAs()
			{
				if (!base_t::present())
					throw std::runtime_error("Trying to access to an uninitialized memory block");

				if( base_t::operator ->()->empty() )
					throw std::runtime_error("Trying to access to a memory block of size 0");

				return reinterpret_cast<T>( & base_t::operator ->()->operator [](0) );
			}

			template <class T> T getAs() const
			{
				if (!base_t::present())
					throw std::runtime_error("Trying to access to an uninitialized memory block");

				if( base_t::operator ->()->empty() )
					throw std::runtime_error("Trying to access to a memory block of size 0");

				return reinterpret_cast<const T>( & base_t::operator ->()->operator [](0) );
			}

		}; // End of class

	} // End of namespace
} // End of namespace

#endif
