
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSENS_SHARED_POINTER_H
#define XSENS_SHARED_POINTER_H

#include <assert.h>
#include <atomic>

namespace xsens {

#ifndef XSENS_SHARED_POINTER_API
#define XSENS_SHARED_POINTER_API
#endif

#ifdef _DEBUG	// must use _DEBUG here, otherwise these values are not valid
#ifdef XSENS_64BIT
#define ptrValid(p) (p != (void*)0 && p != (void*)0xfeeefeeefeeefeeeULL && p != (void*)0xccccccccccccccccULL && p != (void*)0xbaadf00dbaadf00dULL && p != (void*)0xdeadbeefdeadbeefULL && p != (void*)0xababababababababULL && p != (void*)0xbdbdbdbdbdbdbdbdULL && p != (void*)0xfdfdfdfdfdfdfdfdULL && p != (void*)0xcdcdcdcdcdcdcdcdULL && p != (void*)0xddddddddddddddddULL)
#else
#define ptrValid(p) (p != (void*)0 && p != (void*)0xfeeefeee && p != (void*)0xcccccccc && p != (void*)0xbaadf00d && p != (void*)0xdeadbeef && p != (void*)0xabababab && p != (void*)0xbdbdbdbd && p != (void*)0xfdfdfdfd && p != (void*)0xcdcdcdcd && p != (void*)0xdddddddd)
#endif
#else
#define ptrValid(p) (p != 0)
#endif

/*! \brief Base class for shared operations.
	\details This class is a required base class when using shared pointers, since it provides
	the reference counter.
*/
class XSENS_SHARED_POINTER_API SharedData {
public:
	mutable volatile std::atomic_int m_sdAtomicRef;		//!< \brief The atomic reference count value

protected:
	//! \brief Constructor, initializes the reference count to 0
	SharedData() : m_sdAtomicRef(0) {}

	//! \internal \brief Destructor
	virtual ~SharedData() {}

	/*! \brief Copy constructor, initializes the reference count to 0 since the copy is a new object
		\note This function is not publicly accessible, since the inheriting class should specify its own copying
	*/
	SharedData(const SharedData&) : m_sdAtomicRef(0) {}

private:
	/*! \internal \brief The assignment operator is not implemented and not publicly accessible, since the
		inheriting class should specify its own copying and how the reference count is handled differs
	*/
	SharedData& operator =(const SharedData&) = delete;
};

/*! \brief A template class that contains reference counted data with copy-on-write semantics
	\details The SharedDataPointer class implements implicit sharing and is inspired on QSharedDataPointer.
	The contained type T should inherit from SharedData and should implement at least a basic constructor and
	either a copy constructor or a function that allows it to clone itself. In the latter case, a specialized
	clone() function should be written for the SharedDataPointer class.
*/
template <typename T>
class XSENS_SHARED_POINTER_API SharedDataPointer {
private:
	//! \internal \brief A pointer to the contained data.
	T* d;

	//! \internal \brief Increase the reference count by one
	inline void ref() const
	{
		if (d)
		{
			++d->m_sdAtomicRef;
		}
	}

	//! \internal \brief Decrease the reference count by one and possibly delete the referenced object
	inline void deref()
	{
		if (d && !--d->m_sdAtomicRef)
		{
			delete d;
			d = NULL;	// we should only set this to NULL if we have deleted it (self-assignment is a bitch)
		}
	}

protected:
	/*! \brief Create a copy of the referenced object
		\internal
		\details This function allows specializations to supply a different way of copying than by using
		the copy constructor. The default implementation uses the copy constructor.

		The function should not be virtual. Use specialization semantics to define the specialized
		implementation, for example: \code
		template<>
		XmeAuxUnitData* xsens::SharedDataPointer<XmeAuxUnitData>::clone() const { return d->clone(); }
		\endcode
	*/
	T* clone() const
	{
		if (d)
		{
			return new T(*d);
			}
		return new T;
	}

public:
	//! \brief Constructor, takes ownership of the pointer and sets the reference count to 1
	SharedDataPointer() : d(0) {}

	//! \brief Default constructor
	SharedDataPointer(T* p)
	{
		assert(ptrValid(p));
		++p->m_sdAtomicRef;
		d = p;
	}

	//! \brief Copy constructor, adds a reference to the stored object
	SharedDataPointer(const SharedDataPointer& p) : d(p.d)
	{
		ref();
	}

	//! \brief Destructor, dereferences the stored data. If the reference count hits 0, the data is deleted.
	~SharedDataPointer() { deref(); }

	/*! \brief Prepare the object for editing
		\details This function makes sure that the reference count is 1 so that we can safely edit the
		stored object. If the reference count is higher than 1, a deep copy is made. Otherwise the
		function simply returns.
	*/
	void detach()
	{
		if (!d)
		{
			d = clone();
			ref();
			return;
		}

		if (d->m_sdAtomicRef == 1)
			return;

		T* p = clone();
		deref();
		assert(ptrValid(p));
		d = p;
		ref();
	}

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* operator->() const { return d; }

	//! \brief Return a pointer to the data. This does not detach/copy the data.
	T* operator->() { detach();  return d; }

	//! \brief Return a mutable pointer to the data. This detaches/copies the data if the reference count is greater than 1.
	T* mutableData() { detach(); return d; }

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* constData() const { return d; }

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* data() const { return d; }

	//! \brief Whether this is a null-pointer
	bool empty() const { return d == NULL; }

	//! \brief Copy the reference from one SharedDataPointer to another.
	SharedDataPointer& operator = (const SharedDataPointer& p)
	{
		// catch self-assignment, using the proper order of ref and deref
		p.ref();
		deref();
		//assert(ptrValid(p.d)); // FKL: really, it is useless having null-objects when we cannot assign them.
		d = p.d;
		return *this;
	}

	//! \returns The pointer to this object
	T* get() const throw() {return d;}

	//! \returns The reference to this object
	T& operator* () const throw() {assert(d != 0); return *d;}

	/*! \brief Swaps the current object with an other object
		\param other The reference to an other object
	*/
	void swap(SharedDataPointer& other) throw() {std::swap(d, other.d);}

	/*! \brief Swaps the current object with an other object
		\details If no argument is passed the object will be reset
		\param p The pointer to an object
	*/
	void reset(T* p = 0) throw() {if (d != p) {SharedDataPointer(p).swap(*this);}}
};

/*! \brief A template class that contains reference counted data with explicit copy semantics
	\details The ExplicitlySharedDataPointer class implements implicit sharing and is inspired on QExplicitlySharedDataPointer.
	The contained type T should inherit from SharedData and should implement at least a basic constructor and
	either a copy constructor or a function that allows it to clone itself. In the latter case, a specialized
	clone() function should be written for the ExplicitlySharedDataPointer class.

	\note Where the SharedDataPointer class implements copy-on-write semantics (which means an implicit detach()),
	ExplicitlySharedDataPointer	requires the user to explicitly call detach() when necessary. The classes
	are otherwise identical.
*/
template <typename T>
class XSENS_SHARED_POINTER_API ExplicitlySharedDataPointer {
private:
	//! \internal \brief A pointer to the contained data.
	T* d;

	//! \internal \brief Increase the reference count by one
	void ref() const {
		if (d)
		{
			++d->m_sdAtomicRef;
		}
	}

	//! \internal \brief Decrease the reference count by one and possibly delete the referenced object
	void deref() {
		if (d && !--d->m_sdAtomicRef)
		{
			delete d;
			d = 0;
		}
	}

protected:
	/*! \brief Create a copy of the referenced object
		\internal
		\details This function allows specializations to supply a different way of copying than by using
		the copy constructor. The default implementation uses the copy constructor.

		The function should not be virtual. Use specialization semantics to define the specialized
		implementation, for example: \code
		template<>
		XmeAuxUnitData* xsens::ExplicitlySharedDataPointer<XmeAuxUnitData>::clone() const { return d->clone(); }
		\endcode
	*/
	T* clone() const { return new T(*d); }

public:
	//! \brief Constructor, creates a new default T object with reference count 1
	ExplicitlySharedDataPointer() { }

	//! \brief Copy constructor, adds a reference to the stored object
	ExplicitlySharedDataPointer(const ExplicitlySharedDataPointer& p) : d(p.d) { ref(); }

	//! \brief Constructor, takes ownership of the pointer and sets the reference count to 1
	ExplicitlySharedDataPointer(T* p)
	{
		if (p == NULL)
		{
			d = new T;
		}
		else
		{
			++p->m_sdAtomicRef;
			d = p;
		}
	}

	//! \brief Destructor, dereferences the stored data. If the reference count hits 0, the data is deleted.
	~ExplicitlySharedDataPointer() { deref(); }

	/*! \brief Prepare the object for editing
		\details This function makes sure that the reference count is 1 so that we can safely edit the
		stored object. If the reference count is higher than 1, a deep copy is made. Otherwise the
		function simply returns.
	*/
	void detach()
	{
		if (!d)
		{
			d = new T;
			ref();
			return;
		}

		if (d->m_sdAtomicRef.value() == 1)
			return;

		T* p = clone(d);
		deref();
		d = p;
	}

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* operator->() const { return d; }

	//! \brief Return a mutable pointer to the data. This does not detach/copy the data.
	T* operator->() { return d; }

	//! \brief Return an const pointer to the data. This does not detach/copy the data.
	const T* constData() const { return d; }

	//! \brief Return a mutable pointer to the data. This does not detach/copy the data.
	T* mutableData() { return d; }

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* data() const { return d; }

	//! \brief Whether this is a null-pointer
	bool empty() const { return d == NULL; }

	//! \brief Copy the reference from one SharedDataPointer to another.
	ExplicitlySharedDataPointer& operator = (const ExplicitlySharedDataPointer& p)
	{
		// catch self-assignment, using the proper order of ref and deref
		p.ref();
		deref();
		d = p.d;
		return *this;
	}

	//! \returns The pointer to this object
	T* get() const throw() {return d;}

	//! \returns The reference to this object
	T& operator* () const throw() {assert(d != 0); return *d;}

	/*! \brief Swaps the current object with an other object
		\param other The reference to an other object
	*/
	void swap(ExplicitlySharedDataPointer& other) throw() {std::swap(d, other.d);}

	/*! \brief Swaps the current object with an other object
		\details If no argument is passed the object will be reset
		\param p The pointer to an object
	*/
	void reset(T* p = 0) throw() {if (d != p) {ExplicitlySharedDataPointer(p).swap(*this);}}
};

/*! \brief A template class that contains a reference counted pointer
	\details The SharedDataPointer class implements implicit sharing and is inspired on QSharedDataPointer.
	The contained type T should inherit from SharedData and should implement at least a basic constructor and
	either a copy constructor or a function that allows it to clone itself. In the latter case, a specialized
	clone() function should be written for the SharedDataPointer class.
*/
template <typename T>
class XSENS_SHARED_POINTER_API SharedPointer {
private:
	//! \internal \brief A pointer to the contained data.
	T* d;

	//! \internal \brief Increase the reference count by one
	void ref() const
	{
		if (d != 0)
		{
			++d->m_sdAtomicRef;
		}
	}

	//! \internal \brief Decrease the reference count by one and possibly delete the referenced object
	void deref()
	{
		if (d != 0)
		{
			if (!--d->m_sdAtomicRef)
			{
				delete d;
				d = 0;
			}
		}
	}

public:
	//! \brief Null-pointer constructor
	SharedPointer() : d(0) {}

	//! \brief Copy constructor, adds a reference to the stored object
	SharedPointer(const SharedPointer& p) : d(p.d)
	{
		ref();
	}

	//! \brief Constructor, takes ownership of the pointer and sets the reference count to 1
	SharedPointer(T* p)
	{
		if (p)
			++p->m_sdAtomicRef;
		d = p;
	}

	//! \brief Destructor, dereferences the stored data. If the reference count hits 0, the data is deleted.
	~SharedPointer() { deref(); }

	//! \brief Return a const pointer to the data. This does not detach/copy the data.
	const T* operator()() const { return d; }

	//! \brief Return a mutable pointer to the data. This detaches/copies the data if the reference count is greater than 1.
	T* operator()() { return d; }

	//! \brief Copy the reference from one SharedDataPointer to another.
	SharedPointer& operator = (const SharedPointer& p)
	{
		// catch self-assignment, using the proper order of ref and deref
		p.ref();
		deref();
		d = p.d;
		return *this;
	}

	//! \returns True if this object is null
	bool empty() const { return d == NULL; }

	//! \returns The pointer to this object
	T* get() const throw() {return d;}

	//! \returns The pointer to this object. This does not detach/copy the data.
	T* operator -> () const throw() {assert(d != 0); return d;}

	//! \returns The reference to this object. This does not detach/copy the data.
	T& operator* () const throw() {assert(d != 0); return *d;}

	/*! \brief Swaps the current object with an other object
		\param other The reference to an other object
	*/
	void swap(SharedPointer& other) throw() {std::swap(d, other.d);}

	/*! \brief Swaps the current object with an other object
		\details If no argument is passed the object will be reset
		\param p The pointer to an object
	*/
	void reset(T* p = 0) throw() {if (d != p) {SharedPointer(p).swap(*this);}}
};

} // namespace xsens

#endif	// file guard
