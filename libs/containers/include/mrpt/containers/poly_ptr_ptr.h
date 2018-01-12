/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <stdexcept>
#include <cstdlib>

namespace mrpt
{
namespace containers
{
/** \addtogroup mrpt_containers_grp
 * @{ */

/** Wrapper to a clone smart pointer to polymorphic classes, capable of handling
 * copy operator, etc. making deep copies.
 * Example use: `poly_ptr_ptr<mrpt::poses::CPosePDF::Ptr>`
 * \sa copy_ptr<T>
 */
template <typename T>
class poly_ptr_ptr
{
   public:
	/** Ctor from a smart pointer; makes deep copy. */
	poly_ptr_ptr(const T& ptr)
	{
		m_smartptr.reset(dynamic_cast<typename T::element_type*>(ptr->clone()));
	}
	/** Default ctor; init to nullptr. */
	poly_ptr_ptr() {}
	/** copy ctor: makes a copy of the object via `clone()` */
	poly_ptr_ptr(const poly_ptr_ptr<T>& o)
	{
		m_smartptr.reset(
			dynamic_cast<typename T::element_type*>(o.m_smartptr->clone()));
	}
	poly_ptr_ptr<T>& operator=(const poly_ptr_ptr<T>& o)
	{
		if (this == &o) return *this;
		m_smartptr.reset(
			dynamic_cast<typename T::element_type*>(o.m_smartptr->clone()));
		return *this;
	}
	poly_ptr_ptr<T>& operator=(const T& o_ptr)
	{
		m_smartptr.reset(
			dynamic_cast<typename T::element_type*>(o_ptr->clone()));
		return *this;
	}
	/** move ctor */
	poly_ptr_ptr(poly_ptr_ptr&& o)
	{
		m_smartptr = o.m_smartptr;
		o.m_smartptr.reset();
	}
	/** move operator */
	poly_ptr_ptr<T>& operator=(poly_ptr_ptr<T>&& o)
	{
		if (this == &o) return *this;
		m_smartptr = o.m_smartptr;
		o.m_smartptr.reset();
		return *this;
	}
	~poly_ptr_ptr() {}
	typename T::element_type* get()
	{
		if (m_smartptr)
			return m_smartptr.get();
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}
	const typename T::element_type* get() const
	{
		if (m_smartptr)
			return m_smartptr.get();
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}

	typename T::element_type* operator->() { return get(); }
	const typename T::element_type* operator->() const { return get(); }
	typename T::element_type& operator*(void) { return *get(); }
	const typename T::element_type& operator*(void)const { return *get(); }
	operator bool() const { return m_smartptr ? true : false; }
	bool operator!(void)const { return m_smartptr ? false : true; }
	const T& get_ptr() const { return m_smartptr; }
	T& get_ptr() { return m_smartptr; }
	void reset() { m_smartptr.reset(); }
   private:
	T m_smartptr;
};

/** @} */  // end of grouping
}  // namespace containers
}  // namespace mrpt
