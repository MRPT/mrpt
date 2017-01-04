/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <stdexcept>
#include <cstdlib>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup stlext_grp
		  * @{ */

		/** Wrapper to a stlplus clone smart pointer to polymorphic classes, capable of handling 
		* copy operator, etc. making deep copies. 
		* Example use: `poly_ptr_ptr<mrpt::poses::CPosePDFPtr>`
		* \sa copy_ptr<T>
		*/
		template <typename T>
		class poly_ptr_ptr
		{
		public:
			/** Ctor from a smart pointer; makes deep copy. */
			poly_ptr_ptr(const T &ptr) {
				m_smartptr = ptr;
				m_smartptr.make_unique();
			}
			/** Default ctor; init to nullptr. */
			poly_ptr_ptr() {}
			/** copy ctor: makes a copy of the object via `clone()` */
			poly_ptr_ptr(const poly_ptr_ptr<T> & o) {
				m_smartptr = o.m_smartptr;
				m_smartptr.make_unique();
			}
			poly_ptr_ptr<T> & operator =(const poly_ptr_ptr<T> &o) {
				if (this == &o) return *this;
				m_smartptr = o.m_smartptr;
				m_smartptr.make_unique();
				return *this;
			}
			poly_ptr_ptr<T> & operator =(const T &o_ptr) {
				m_smartptr = o_ptr;
				m_smartptr.make_unique();
				return *this;
			}
#if (__cplusplus>199711L)
			/** move ctor */
			poly_ptr_ptr(poly_ptr_ptr && o) {
				m_smartptr = o.m_smartptr;
				o.m_smartptr.clear_unique();
			}
			/** move operator */
			poly_ptr_ptr<T> &operator =(poly_ptr_ptr<T> && o) {
				if (this == &o) return *this;
				m_smartptr = o.m_smartptr;
				o.m_smartptr.clear_unique();
				return *this;
			}
#endif
			~poly_ptr_ptr() {}

			typename T::value_type * pointer() {
				if (m_smartptr) return m_smartptr.pointer();
				else throw std::runtime_error("dereferencing NULL poly_ptr");
			}
			const typename T::value_type * pointer() const {
				if (m_smartptr) return m_smartptr.pointer();
				else throw std::runtime_error("dereferencing NULL poly_ptr");
			}

			typename T::value_type * operator->() { return pointer(); }
			const typename T::value_type * operator->() const { return pointer(); }

			typename T::value_type& operator*(void) { return *pointer(); }
			const typename T::value_type& operator*(void) const { return *pointer(); }

			operator bool() const { return m_smartptr.present(); }
			bool operator!(void) const { return !m_smartptr.present(); }

			const T & get_ptr() const { return m_smartptr; }
			T & get_ptr() { return m_smartptr; }

			void clear_unique() { m_smartptr.clear_unique();  }
		private:
			T m_smartptr;
		};

		/** @} */  // end of grouping
	} // End of namespace
} // End of namespace
