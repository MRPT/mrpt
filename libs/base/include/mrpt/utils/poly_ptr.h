/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
		  * No shared copies, that is, each `poly_ptr<T>` owns a unique instance of `T`. 
		  * Copying a `poly_ptr<T>` invokes the copy operator for `T`.
		  * \sa copy_ptr<T>
		  */
		template <typename T>
		class poly_ptr : public internal::generic_copier_ptr<T, internal::CopyCloner<T> >
		{
		private:
			typedef internal::generic_copier_ptr<T, internal::CopyCloner<T> > ptr_base_t;
		public:
			/** Ctor from a pointer; takes ownership. */
			explicit poly_ptr(T * ptr) : ptr_base_t(ptr) {}
			/** Default ctor; init to nullptr. */
			poly_ptr() : ptr_base_t() {}
			/** copy ctor: makes a copy of the object via `clone()` */
			poly_ptr(const poly_ptr<T> & o) : ptr_base_t(o) {}
			/** copy operator */
			poly_ptr<T> &operator =(const poly_ptr<T> & o) {
				if (this == &o) return *this;
				this->reset();
				ptr_base_t::m_ptr = typename ptr_base_t::copier_t().copy(o.m_ptr);
				return *this;
			}
#if (__cplusplus>199711L)
			/** move ctor */
			poly_ptr(poly_ptr<T> && o) : ptr_base_t(o) {}
			/** move operator */
			poly_ptr<T> &operator =(const poly_ptr<T> && o) {
				if (this == &o) return *this;
				ptr_base_t::operator =(o);
				return *this;
			}
#endif
		};

		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace
