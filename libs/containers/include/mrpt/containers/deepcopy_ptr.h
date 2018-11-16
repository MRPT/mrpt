/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
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
	using value_type = T;
	using copier_t = Copier;
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

	/** move ctor */
	generic_copier_ptr(generic_copier_ptr<T, Copier>&& o)
	{
		m_ptr = o.m_ptr;
		o.m_ptr = nullptr;
	}
	/** move operator */
	generic_copier_ptr<T, Copier>& operator=(generic_copier_ptr<T, Copier>&& o)
	{
		if (this == &o) return *this;
		m_ptr = o.m_ptr;
		o.m_ptr = nullptr;
		return *this;
	}

	/** copy operator */
	generic_copier_ptr<T, Copier>& operator=(
		const generic_copier_ptr<T, Copier>& o)
	{
		if (this == &o) return *this;
		this->reset();
		m_ptr = Copier().copy(o.m_ptr);
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

	T& operator*()
	{
		if (m_ptr)
			return *m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}
	const T& operator*() const
	{
		if (m_ptr)
			return *m_ptr;
		else
			throw std::runtime_error("dereferencing nullptr poly_ptr");
	}

	T* get() { return m_ptr; }
	const T* get() const { return m_ptr; }
	operator bool() const { return m_ptr != nullptr; }
	bool operator!() const { return m_ptr == nullptr; }
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

}  // namespace internal

/** Smart pointer for polymorphic classes with a `clone()` method.
 * No shared copies, that is, each `poly_ptr<T>` owns a unique instance of `T`.
 * Copying a `poly_ptr<T>` invokes the copy operator for `T`.
 * \sa copy_ptr<T>
 */
template <typename T>
using poly_ptr = internal::generic_copier_ptr<T, internal::CopyCloner<T>>;

/** Smart pointer for non-polymorphic classes.
 * No shared copies, that is, each `copy_ptr<T>` owns a unique instance of `T`.
 * Copying a `copy_ptr<T>` invokes the copy operator for `T`.
 * \sa poly_ptr<T>
 */
template <typename T>
using copy_ptr = internal::generic_copier_ptr<T, internal::CopyStatic<T>>;

/** @} */  // end of grouping
}  // namespace containers
}  // namespace mrpt
