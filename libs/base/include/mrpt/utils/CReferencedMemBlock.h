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
#ifndef  CReferencedMemBlock_H
#define  CReferencedMemBlock_H

#include <mrpt/utils/utils_defs.h>
#include <utility>

namespace mrpt
{
	namespace utils
	{
		/** Represents a memory block (via "void*") that can be shared between several objects through copy operator (=).
		  *  It keeps the reference count and only when it comes to zero, the memory block is really freed.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CReferencedMemBlock : public stlplus::smart_ptr< vector_byte >
		{
		public:
			/** Constructor with an optional size of the memory block */
			CReferencedMemBlock(size_t mem_block_size = 0 );

			/** Destructor, calls dereference_once. */
			virtual ~CReferencedMemBlock();

			/** Resize the shared memory block. */
			void resize(size_t mem_block_size );

			template <class T> T getAs()
			{
				if (!stlplus::smart_ptr< vector_byte >::present())
					THROW_EXCEPTION("Trying to access to an uninitialized memory block");

				if( stlplus::smart_ptr< vector_byte >::operator ->()->empty() )
					THROW_EXCEPTION("Trying to access to a memory block of size 0");

				return reinterpret_cast<T>( & stlplus::smart_ptr< vector_byte >::operator ->()->operator [](0) );
			}

			template <class T> T getAs() const
			{
				if (!stlplus::smart_ptr< vector_byte >::present())
					THROW_EXCEPTION("Trying to access to an uninitialized memory block");

				if( stlplus::smart_ptr< vector_byte >::operator ->()->empty() )
					THROW_EXCEPTION("Trying to access to a memory block of size 0");

				return reinterpret_cast<const T>( & stlplus::smart_ptr< vector_byte >::operator ->()->operator [](0) );
			}

		}; // End of class

	} // End of namespace
} // End of namespace

#endif
