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
namespace utils
{
/** \addtogroup stlext_grp
  * @{ */

namespace internal
{
template <typename T>
struct CopyStatic
{
	T* copy(const T* o)
	{
		if (!o) return nullptr;
		return new T(*o);
	}
};

template <typename T>
struct CopyCloner
{
	T* copy(const T* o)
	{
		if (!o) return nullptr;
		T* n = dynamic_cast<T*>(o->clone());
		if (!n)
			throw std::runtime_error("error: clone() returned unexpected type");
		return n;
	}
};

template <typename T, typename Copier>
class generic_copier_ptr
{
   public:
	typedef T value_type;
	typedef Copier copier_t;
	/** Ctor from a pointer; takes ownership. */
	explicit generic_copier_ptr(T* ptr) : m_ptr(ptr) {}
	/** Default ctor; init to nullptr. */
	generic_copier_ptr() : m_ptr(nullptr) {}
	/** copy ctor: makes a copy of the object via `clone()` */
	generic_copier_ptr(const generic_copier_ptr<T, Copier>& o)
		: m_ptr(Copier().copy(o.m_ptr))
	{
	}
	~generic_copier_ptr()
	{
		if (m_ptr) delete m_ptr;
	}

	void move_from(generic_copier_ptr& o)
	{
		m_ptr = o.m_ptr;
		o.m_ptr = nullptr;
	}
	/** move ctor */
	generic_copier_ptr(generic_copier_ptr<T, Copier>&& o)
	{
		m_ptr = o.m_ptr;
		o.m_ptr = nullptr;
	}
	/** move operator */
	generic_copier_ptr<T, Copier>& operator=(
		const generic_copier_ptr<T, Copier>&& o)
	{
		if (this == &o) return *this;
		m_ptr = o.m_ptr;
		o.m_ptr = nullptr;
		return *this;
	}

	T* operator->()
	{
		if (m_ptr)
			return m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}
	const T* operator->() const
	{
		if (m_ptr)
			return m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}

	T& operator*(void)
	{
		if (m_ptr)
			return *m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}
	const T& operator*(void)const
	{
		if (m_ptr)
			return *m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}

	T* get() { return m_ptr; }
	const T* get() const { return m_ptr; }
	operator bool() const { return m_ptr != nullptr; }
	bool operator!(void)const { return m_ptr == nullptr; }
	/** Releases the pointer (do not destroy the object) */
	T* release()
	{
		T* r = m_ptr;
		m_ptr = nullptr;
		return r;
	}

	void reset(T* ptr = nullptr)
	{
		if (ptr == m_ptr) return;
		if (m_ptr) delete m_ptr;
		m_ptr = ptr;
	}
	void resetDefaultCtor() { this->reset(new T()); }
   protected:
	T* m_ptr;
};

}  // end NS internal

/** @} */  // end of grouping
}  // End of namespace
}  // End of namespace
