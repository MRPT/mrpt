/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  safe_pointers_H
#define  safe_pointers_H

#include <mrpt/config.h>
#include <mrpt/utils/boost_join.h>
#include <mrpt/utils/mrpt_macros.h>  // ASSERT_()

namespace mrpt
{
namespace utils
{
	/** A wrapper class for pointers that can be safely copied with "=" operator without problems.
	  * This class does not keep any reference count nor automatically destroy the pointed data.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct safe_ptr_basic
	{
	protected:
		T *ptr;

	public:
		safe_ptr_basic() : ptr(NULL) { }
		safe_ptr_basic(const safe_ptr_basic<T> &o) : ptr(o.ptr) { }
		safe_ptr_basic(const T* p) : ptr(const_cast<T*>(p)) { }
		safe_ptr_basic<T> &operator =(T * p) { ptr = p; return *this; }

		safe_ptr_basic<T> &operator =(const safe_ptr_basic<T>&o)
		{
			ptr = o.ptr;
			return *this;
		}

		virtual ~safe_ptr_basic() {  }

		bool operator == ( const T *o ) const { return o==ptr; }
		bool operator == ( const safe_ptr_basic<T> &o )const { return o.ptr==ptr; }

		bool operator != ( const T *o )const { return o!=ptr; }
		bool operator != ( const safe_ptr_basic<T> &o )const { return o.ptr!=ptr; }

		T*& get() { return ptr; }
		const T* get()const { return ptr; }

		T *& operator ->() { ASSERT_(ptr); return ptr; }
		const T * operator ->() const  { ASSERT_(ptr); return ptr; }
	};

	/** A wrapper class for pointers that can be safely copied with "=" operator without problems.
	  * This class does not keep any reference count nor automatically destroy the pointed data.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct safe_ptr : safe_ptr_basic<T>
	{
	public:
		safe_ptr() : safe_ptr_basic<T>() { }
		safe_ptr(const safe_ptr<T> &o) : safe_ptr_basic<T>(o) { }
		safe_ptr(const T* p) : safe_ptr_basic<T>(p) { }

		virtual ~safe_ptr() { }

		T & operator *() { ASSERT_(safe_ptr_basic<T>::ptr); return *safe_ptr_basic<T>::ptr; }
		const T & operator *() const  { ASSERT_(safe_ptr_basic<T>::ptr); return *safe_ptr_basic<T>::ptr; }

		T & operator [](const size_t &i) { ASSERT_(safe_ptr_basic<T>::ptr); return safe_ptr_basic<T>::ptr[i]; }
		const T & operator [](const size_t &i) const { ASSERT_(safe_ptr_basic<T>::ptr); return safe_ptr_basic<T>::ptr[i]; }
	};


	/** A wrapper class for pointers that can NOT be copied with "=" operator, raising an exception at runtime if a copy is attempted.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct non_copiable_ptr_basic
	{
	protected:
		T *ptr;

	public:
		non_copiable_ptr_basic() : ptr(NULL) { }
		non_copiable_ptr_basic(const non_copiable_ptr_basic<T> &) : ptr(NULL) { THROW_EXCEPTION("Pointer non-copiable..."); }
		non_copiable_ptr_basic(const T* p) : ptr(const_cast<T*>(p)) { }
		non_copiable_ptr_basic<T> &operator =(T * p) { ptr = p; return *this; }

		non_copiable_ptr_basic<T> &operator =(const non_copiable_ptr_basic<T>&)
		{ THROW_EXCEPTION("Pointer non-copiable..."); }

		/** This method can change the pointer, since the change is made explicitly, not through copy operators transparent to the user. */
		void set( const T* p ) { ptr = const_cast<T*>(p); }

		virtual ~non_copiable_ptr_basic() {  }

		bool operator == ( const T *o ) const { return o==ptr; }
		bool operator == ( const non_copiable_ptr_basic<T> &o )const { return o.ptr==ptr; }

		bool operator != ( const T *o )const { return o!=ptr; }
		bool operator != ( const non_copiable_ptr_basic<T> &o )const { return o.ptr!=ptr; }

		T*& get() { return ptr; }
		const T* get()const { return ptr; }

		T** getPtrToPtr() { return &ptr; }

		T *& operator ->() { ASSERT_(ptr); return ptr; }
		const T * operator ->() const  { ASSERT_(ptr); return ptr; }
	};

	/** A wrapper class for pointers that can NOT be copied with "=" operator, raising an exception at runtime if a copy is attempted.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct non_copiable_ptr : non_copiable_ptr_basic<T>
	{
	public:
		non_copiable_ptr() : non_copiable_ptr_basic<T>() { }
		non_copiable_ptr(const non_copiable_ptr<T> &o) : non_copiable_ptr_basic<T>(o) { }
		non_copiable_ptr(const T* p) : non_copiable_ptr_basic<T>(p) { }

		non_copiable_ptr<T> &operator =(const T* p) { non_copiable_ptr_basic<T>::ptr = const_cast<T*>(p); return *this; }

		non_copiable_ptr<T> &operator =(const non_copiable_ptr<T>&)
		{ THROW_EXCEPTION("Pointer non-copiable..."); }

		virtual ~non_copiable_ptr() {  }

		T & operator *() { ASSERT_(non_copiable_ptr_basic<T>::ptr); return *non_copiable_ptr_basic<T>::ptr; }
		const T & operator *() const  { ASSERT_(non_copiable_ptr_basic<T>::ptr); return *non_copiable_ptr_basic<T>::ptr; }

		T & operator [](const size_t &i) { ASSERT_(non_copiable_ptr_basic<T>::ptr); return non_copiable_ptr_basic<T>::ptr[i]; }
		const T & operator [](const size_t &i) const { ASSERT_(non_copiable_ptr_basic<T>::ptr); return non_copiable_ptr_basic<T>::ptr[i]; }
	};

	/** A wrapper class for pointers whose copy operations from other objects of the same type are ignored, that is, doing "a=b;" has no effect neiter on "a" or "b".
	  * In turn, assigning a pointer with a direct "=" operation from a plain "T*" type is permited.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct ignored_copy_ptr
	{
	protected:
		T *ptr;

	public:
		ignored_copy_ptr() : ptr(NULL) { }
		ignored_copy_ptr(const ignored_copy_ptr<T> &) : ptr(NULL) { }
		ignored_copy_ptr(const T* p) : ptr(const_cast<T*>(p)) { }
		ignored_copy_ptr<T> &operator =(T * p) { ptr=p; return *this; }

		ignored_copy_ptr<T> &operator =(const ignored_copy_ptr<T>&) { return *this; }

		/** This method can change the pointer, since the change is made explicitly, not through copy operators transparent to the user. */
		void set( const T* p ) { ptr = const_cast<T*>(p); }

		virtual ~ignored_copy_ptr() {  }

		bool operator == ( const T *o ) const { return o==ptr; }
		bool operator == ( const ignored_copy_ptr<T> &o )const { return o.ptr==ptr; }

		bool operator != ( const T *o )const { return o!=ptr; }
		bool operator != ( const ignored_copy_ptr<T> &o )const { return o.ptr!=ptr; }

		T*& get() { return ptr; }
		const T* get()const { return ptr; }

		T** getPtrToPtr() { return &ptr; }

		T *& operator ->() { ASSERT_(ptr); return ptr; }
		const T * operator ->() const  { ASSERT_(ptr); return ptr; }
	};


	/** A wrapper class for pointers that, if copied with the "=" operator, should be set to NULL in the copy.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct copiable_NULL_ptr_basic
	{
	protected:
		T *ptr;

	public:
		copiable_NULL_ptr_basic() : ptr(NULL) { }
		copiable_NULL_ptr_basic(const copiable_NULL_ptr_basic<T> &) : ptr(NULL) {  }

		copiable_NULL_ptr_basic<T> &operator =(T * p) { ptr=p; return *this; }

		copiable_NULL_ptr_basic<T> &operator =(const copiable_NULL_ptr_basic<T>&) { ptr=NULL; return *this; }

		virtual ~copiable_NULL_ptr_basic() {  }

		bool operator == ( const T *o ) const { return o==ptr; }
		bool operator == ( const copiable_NULL_ptr_basic<T> &o )const { return o.ptr==ptr; }

		bool operator != ( const T *o )const { return o!=ptr; }
		bool operator != ( const copiable_NULL_ptr_basic<T> &o )const { return o.ptr!=ptr; }

		T*& get() { return ptr; }
		const T*& get()const { return ptr; }

		T *& operator ->() { ASSERT_(ptr); return ptr; }
		const T *& operator ->() const  { ASSERT_(ptr); return ptr; }
	};

	/** A wrapper class for pointers that, if copied with the "=" operator, should be set to NULL in the new copy.
	  * \sa CReferencedMemBlock, safe_ptr, non_copiable_ptr, copiable_NULL_ptr
	  * \ingroup mrpt_base_grp
	  */
	template <class T>
	struct copiable_NULL_ptr : copiable_NULL_ptr_basic<T>
	{
	public:
		copiable_NULL_ptr() : copiable_NULL_ptr_basic<T>() { }
		copiable_NULL_ptr(const copiable_NULL_ptr<T> &o) : copiable_NULL_ptr_basic<T>(o) { }

		copiable_NULL_ptr<T> &operator =(T * p) { copiable_NULL_ptr_basic<T>::ptr=p; return *this; }

		virtual ~copiable_NULL_ptr() { }

		T & operator *() { ASSERT_(copiable_NULL_ptr_basic<T>::ptr); return *copiable_NULL_ptr_basic<T>::ptr; }
		const T & operator *() const  { ASSERT_(copiable_NULL_ptr_basic<T>::ptr); return *copiable_NULL_ptr_basic<T>::ptr; }

		T & operator [](const size_t &i) { ASSERT_(copiable_NULL_ptr_basic<T>::ptr); return copiable_NULL_ptr_basic<T>::ptr[i]; }
		const T & operator [](const size_t &i) const { ASSERT_(copiable_NULL_ptr_basic<T>::ptr); return copiable_NULL_ptr_basic<T>::ptr[i]; }
	};



	typedef safe_ptr_basic<void> void_ptr;
	typedef non_copiable_ptr_basic<void> void_ptr_noncopy;

	} // End of namespace
} // End of namespace
#endif
