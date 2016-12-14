/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/generic_copier_ptr.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp
		  * @{ */

		/** Smart pointer for polymorphic classes with a `clone()` method. 
		  * No shared copies, that is, each `clone_ptr<T>` owns a unique instance of `T`. 
		  * Copying a `clone_ptr<T>` invokes the copy operator for `T`.
		  * \sa copy_ptr<T>
		  */
		template <typename T>
		class clone_ptr : public internal::generic_copier_ptr<T, internal::CopyCloner<T> >
		{
		private:
			typedef internal::generic_copier_ptr<T, internal::CopyCloner<T> > ptr_base_t;
		public:
			/** Ctor from a pointer; takes ownership. */
			explicit clone_ptr(T * ptr) : ptr_base_t(ptr) {}
			/** Default ctor; init to nullptr. */
			clone_ptr() : ptr_base_t() {}
			/** copy ctor: makes a copy of the object via `clone()` */
			clone_ptr(const clone_ptr & o) : ptr_base_t(o) {}
			/** copy operator */
			clone_ptr<T> &operator =(const clone_ptr<T> & o) {
				if (this == &o) return *this;
				this->reset();
				m_ptr = copier_t().copy(o.m_ptr);
				return *this;
			}
		};

		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace
