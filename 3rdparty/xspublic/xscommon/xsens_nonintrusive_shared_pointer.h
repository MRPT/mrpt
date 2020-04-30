
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

#ifndef XSENS_NONINTRUSIVE_SHARED_POINTER_H
#define XSENS_NONINTRUSIVE_SHARED_POINTER_H

#include <assert.h>
#include <algorithm> // std::swap

namespace xsens
{

/*! \brief non-intrusive shared pointer class similair to boost::shared_ptr

	The NonIntrusiveSharedPointer class template stores a pointer to a dynamically allocated object, typically with a C++ new-expression.
	The object pointed to is guaranteed to be deleted when the last NonIntrusiveSharedPointer pointing to it is destroyed or reset.
	Every NonIntrusiveSharedPointer meets the CopyConstructible and Assignable requirements of the C++ Standard Library, and so can be used in standard library containers.
*/
template <typename T>
class NonIntrusiveSharedPointer
{
public:
	typedef T ElementType; //!< Provides the type of the template parameter T.

	/*! \brief NonIntrusiveSharedPointer Constructor.
		Constructs a NonIntrusiveSharedPointer which takes ownership of the pointer p.
		The pointer p has to be allocated by a C++ new-expression or may be null.
		\param[in] p The pointer the NonIntrusiveSharedPointer takes ownership of
	*/
	explicit NonIntrusiveSharedPointer(ElementType* p = 0) : m_data(0) {
		if (p != 0) {
			m_data = new Data(p);
		}
	}

	/*! \brief NonIntrusiveSharedPointer Destructor.
		Destroys a NonIntrusiveSharedPointer instance. If this was the last reference to the owned
		pointer, the pointer will be deleted.
	*/
	~NonIntrusiveSharedPointer() throw() {
		if (m_data != 0) {
			if (--m_data->m_count == 0) {
				delete m_data->m_pointer;
				delete m_data;
			}
		}
	}

	/*! \brief NonIntrusiveSharedPointer Copy constructor.
		Constructs a NonIntrusiveSharedPointer that shares ownership with the other NonIntrusiveSharedPointer
		(may be empty if other is empty (null)
		\param[in] other The shared pointer to copy from (share ownership of container pointer)
	*/
	NonIntrusiveSharedPointer(NonIntrusiveSharedPointer const & other) : m_data(other.m_data) {
		if (m_data != 0) {
			++m_data->m_count;
		}
	}

	/*! \brief NonIntrusiveSharedPointer assignment operator
		Equivalent to NonIntrusiveSharedPointer(other).swap(*this)
	*/
	NonIntrusiveSharedPointer& operator = (NonIntrusiveSharedPointer other) { // intentional pass-by-value serves as a temporary copy
		other.swap(*this);
		return *this;
	}

	/*!
		Exchanges the contents of the two NonIntrusiveSharedPointers.
		\param[in] other The NonIntrusiveSharedPointer to swap contents with.
	*/
	inline void swap(NonIntrusiveSharedPointer<T>& other) throw() {
		std::swap(m_data, other.m_data);
	}

	/*!
		\pre The stored pointer may not be null and the NonIntrusiveSharedPointer may not be empty.
		\returns a reference to the object pointed to by the stored pointer.
	*/
	inline ElementType& operator* () const throw() {
		assert(m_data != 0 && m_data->m_pointer != 0);
		return *m_data->m_pointer;
	}

	/*!
		\pre The stored pointer may not be null and the NonIntrusiveSharedPointer may not be empty.
		\returns the stored pointer
	*/
	inline ElementType* operator -> () const throw() {
		assert(m_data != 0 && m_data->m_pointer != 0);
		return m_data->m_pointer;
	}

	/*!
		\returns the stored pointer or 0 if empty
	*/
	inline ElementType* get() const throw() {
		return (m_data != 0) ? m_data->m_pointer : 0;
	}


	/*! \brief Equivalent of NonIntrusiveSharedPointer<T>(p).swap(*this)
	*/
	void reset(ElementType* p = 0) throw() {
		if (m_data == 0 || m_data->m_pointer != p) {
			NonIntrusiveSharedPointer<T>(p).swap(*this);
		}
	}

	/*! \brief Returns true if the shared pointer points to a NULL object */
	inline bool isNull() const {
		return get() == 0;
	}

private:
	/*!
		A struct that holds the reference count and the actual stored (shared) pointer.
		An instance of this struct is shared (by means of the m_data member) by NonIntrusiveSharedPointers
		that shared the same data.
	*/
	struct Data {
		Data(ElementType* p) : m_pointer(p), m_count(1) {}
		ElementType* m_pointer;
		size_t m_count;
	};

	Data* m_data; //!< A pointer to the shared data.
};

/*!
	Equivalent to p1.swap(p2)
	Matches the interface of std::swap. Provided as an aid to generic programming.
*/
template <typename T>
void swap(NonIntrusiveSharedPointer<T>& p1, NonIntrusiveSharedPointer<T>& p2) throw() {
	p1.swap(p2);
}

} // namespace xsens

namespace std
{
	/*!
		Equivalent to p1.swap(p2)
		Matches the interface of std::swap. Provided as an aid to generic programming.
	*/
	template <typename T>
	void swap(xsens::NonIntrusiveSharedPointer<T>& p1, xsens::NonIntrusiveSharedPointer<T>& p2) throw()
	{
		p1.swap(p2);
	}
}

#endif
