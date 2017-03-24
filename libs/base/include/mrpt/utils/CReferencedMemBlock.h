/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CReferencedMemBlock_H
#define  CReferencedMemBlock_H

#include <mrpt/base/link_pragmas.h>
#include <vector>

// STL+ library:
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mrpt
{
	namespace utils
	{
		/** Represents a memory block (via "void*") that can be shared between several objects through copy operator (=).
		  *  It keeps the reference count and only when it comes to zero, the memory block is really freed.
		  * Behaves like std::shared_ptr<>.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CReferencedMemBlock
		{
		public:
			/** Constructor with an optional size of the memory block */
			CReferencedMemBlock(size_t mem_block_size = 0 );

			/** Destructor, calls dereference_once. */
			virtual ~CReferencedMemBlock();

			/** Resize the shared memory block. */
			void resize(size_t mem_block_size );

			template <class T> T* getAsPtr()
			{
				if (!m_data.present()) throw std::runtime_error("Trying to access to an uninitialized memory block");
				if (m_data->empty()) throw std::runtime_error("Trying to access to a memory block of size 0");
				return reinterpret_cast<T*>(&((*m_data)[0]));
			}
			template <class T> const T* getAsPtr() const
			{
				if (!m_data.present()) throw std::runtime_error("Trying to access to an uninitialized memory block");
				if (m_data->empty()) throw std::runtime_error("Trying to access to a memory block of size 0");
				return reinterpret_cast<const T*>(&((*m_data)[0]));
			}

			template <class T> T&       getAs()       { return *getAsPtr<T>(); }
			template <class T> const T& getAs() const { return *getAsPtr<T>(); }

			unsigned int alias_count() const;
			/** Frees the underlying memory block */
			void clear();
		protected:
			stlplus::smart_ptr< std::vector<char> > m_data;
		}; // End of class

	} // End of namespace
} // End of namespace

#endif
