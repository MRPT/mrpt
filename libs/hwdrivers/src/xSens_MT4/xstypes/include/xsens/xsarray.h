/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSARRAY_H
#define XSARRAY_H

#include "xstypesconfig.h"

#define XSARRAY_DECL(T)\
	T* const m_data;			/*!< Pointer to contained data buffer */\
	const XsSize m_size;		/*!< Size of used data buffer in number of items */\
	const XsSize m_reserved;	/*!< Size of allocated data buffer in number of items */\
	const int m_flags;			/*!< Flags for data management */\
	XsArrayDescriptor const* const m_descriptor;		/*!< Describes how to handle the items in the array */

#define XSARRAY_STRUCT(S,T)		struct S { XSARRAY_DECL(T) }
#define XSARRAY_INITIALIZER(D)	{ 0, 0, 0, XSDF_Managed, D }

/*! \cond XS_INTERNAL */
typedef void (*XsArrayItemSwapFunc)(void*, void*);
typedef void (*XsArrayItemStructFunc)(void*);
typedef void (*XsArrayItemCopyFunc)(void*, void const*);
typedef int (*XsArrayItemCompareFunc)(void const*, void const*);
#if defined(__GNUC__)
#define XSEXPCASTITEMSWAP	(XsArrayItemSwapFunc)
#define XSEXPCASTITEMMAKE	(XsArrayItemStructFunc)
#define XSEXPCASTITEMCOPY	(XsArrayItemCopyFunc)
#define XSEXPCASTITEMCOMP	(XsArrayItemCompareFunc)
#else
#define XSEXPCASTITEMSWAP
#define XSEXPCASTITEMMAKE
#define XSEXPCASTITEMCOPY
#define XSEXPCASTITEMCOMP
#endif
/*! \endcond */

/*! \brief This object describes how to treat the data in an array.
	\details Ususally there is one static instance per type of array that will be used by all
	XsArrays of that type.
*/
struct XsArrayDescriptor {
#ifndef __cplusplus
	const
#else
protected:
	template <typename T, XsArrayDescriptor const& D, typename I> friend struct XsArrayImpl;
#endif
	XsSize itemSize;									//!< The size of an array item in bytes
	void (*itemSwap)(void* a, void* b);					//!< The function to use for swapping the data of two array items. \param a Pointer to first item to swap. \param b Pointer to second item to swap.
	void (*itemConstruct)(void* e);						//!< The function to use for constructing a new array item. May be 0 for simple types. \param e Pointer to item to construct.
	void (*itemCopyConstruct)(void* e, void const* s);	//!< The function to use for constructing a new array item with a source initializer. This may not be 0. \param e Pointer to item to construct. \param s Pointer to source item to copy from.
	void (*itemDestruct)(void* e);						//!< The function to use for destructing a array item. May be 0 for simple types. \param e Pointer to item to destruct.
	void (*itemCopy)(void* to, void const* from);		//!< The function to use for copying the data of \a from to \a to. \param to Pointer to item to copy to. \param from Pointer to item to copy from.
	int (*itemCompare)(void const* a, void const* b);	//!< The function to use for comparing two items. \param a Left hand side of comparison. \param b Right hand side of comparison. \returns The function will return 0 when the items are equal. When greater/less comparison is possible, the function should return < 0 if a < b and > 0 if a > b.
};
typedef struct XsArrayDescriptor XsArrayDescriptor;

#ifdef __cplusplus
#include <iterator>
#include <stdexcept>
extern "C" {
#endif

XSTYPES_DLL_API void XsArray_construct(void* thisPtr, XsArrayDescriptor const* const descriptor, XsSize count, void const* src);
XSTYPES_DLL_API void XsArray_copyConstruct(void* thisPtr, void const* src);
XSTYPES_DLL_API void XsArray_destruct(void* thisPtr);
XSTYPES_DLL_API void XsArray_assign(void* thisPtr, XsSize count, void const* src);
XSTYPES_DLL_API void XsArray_resize(void* thisPtr, XsSize count);
XSTYPES_DLL_API void XsArray_reserve(void* thisPtr, XsSize count);
XSTYPES_DLL_API void XsArray_copy(void* thisPtr, void const* src);
XSTYPES_DLL_API void XsArray_append(void* thisPtr, void const* other);
XSTYPES_DLL_API void XsArray_insert(void* thisPtr, XsSize index, XsSize count, void const* src);
XSTYPES_DLL_API void XsArray_erase(void* thisPtr, XsSize index, XsSize count);
XSTYPES_DLL_API void XsArray_swap(void* a, void* b);
XSTYPES_DLL_API int XsArray_compare(void const* a, void const* b);
XSTYPES_DLL_API int XsArray_compareSet(void const* a, void const* b);
XSTYPES_DLL_API int XsArray_find(void const* thisPtr, void const* needle);
XSTYPES_DLL_API void const* XsArray_at(void const* thisPtr, XsSize index);
XSTYPES_DLL_API void* XsArray_atIndex(void* thisPtr, XsSize index);
XSTYPES_DLL_API void XsArray_removeDuplicates(void* thisPtr);

struct XsArray {
	XSARRAY_DECL(void)
#ifdef __cplusplus
	//! \copydoc XsArray_construct
	inline XsArray(XsArrayDescriptor const* descriptor, XsSize count = 0, void const* src = 0)
		: m_data(0)
		, m_size(0)
		, m_reserved(0)
		, m_flags(0)
		, m_descriptor(0)
	{
		XsArray_construct(this, descriptor, count, src);
	}

	//! \copydoc XsArray_copyConstruct
	inline XsArray(const XsArray& src)
		: m_data(0)
		, m_size(0)
		, m_reserved(0)
		, m_flags(0)
		, m_descriptor(0)
	{
		XsArray_copyConstruct(this, &src);
	}

	//! \brief Creates a array that references the data supplied in \a ref without allocating the data itself
	inline explicit XsArray(XsArrayDescriptor const* descriptor, void* ref, XsSize count, XsDataFlags flags)
		: m_data(ref)
		, m_size(count)
		, m_reserved(count)
		, m_flags(flags)
		, m_descriptor(descriptor)
	{
	}

	//! \brief Destructor
	~XsArray()
	{
		XsArray_destruct(this);
	}

	/*! \brief Assignment operator
		\details Copies the values in \a src into \a this
		\param src The array to copy from
		\return A reference to this
	*/
	inline XsArray const& operator=(const XsArray& src)
	{
		if (this != &src)
			XsArray_copy(this, &src);
		return *this;
	}
#endif
};

typedef struct XsArray XsArray;

#ifdef __cplusplus
} // extern "C"

/*! \brief A C++ wrapper for XsArray, this is a template class for the actual specialized classes
	\tparam T The type of the contained values
	\tparam D The descriptor to use for this specific array implementation. This must be statically allocated and its lifetime must encompass the lifetime of objects that use it.
	\tparam I The class that inherits from the XsArrayImpl. Some functions (such as the streaming operator) require the inheriting type to be returned for proper functionality.
*/
template <typename T, XsArrayDescriptor const& D, typename I>
struct XsArrayImpl : protected XsArray { // MRPT
	//! \brief The contained type
	typedef T value_type;

	//! \brief A shorthand for the type of this specific implementation
	typedef XsArrayImpl<T, D, I> ArrayImpl;

	/*! \brief Construct an XsArray
		\param count the number of items in src
		\param src pointer to an array of output configurations
		\sa XsArray_construct
	 */
	inline XsArrayImpl<T, D, I>(XsSize count = 0, T const* src = 0)
		: XsArray(&D, count, src)
	{
	}

	//! \brief Constructs the XsArray as a copy of \a other
	inline XsArrayImpl<T, D, I>(ArrayImpl const& other)
		: XsArray(other)
	{
	}
#ifndef XSENS_NOITERATOR
	//! \brief Constructs the XsArray with a copy of the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsArrayImpl<T, D, I>(Iterator const& beginIt, Iterator const& endIt)
		: XsArray(&D, 0, 0)
	{
		ptrdiff_t diff = endIt-beginIt;
		if (diff > 0)
		{
			reserve((XsSize) diff);
			for (Iterator it = beginIt; it != endIt; ++it)
				push_back(*it);
		}
	}
#endif
	//! \brief Creates the XsArray as a reference to the data supplied in \a ref
	inline explicit XsArrayImpl<T, D, I>(T* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: XsArray(&D, ref, sz, flags)
	{
	}

	//! \copydoc XsArray_destruct
	inline ~XsArrayImpl()
	{
		XsArray_destruct(this);
	}

	//! \brief Clears the array \sa XsArray_destruct()
	inline void clear()
	{
		XsArray_destruct(this);
	}

	/*! \brief Tests \a other for equality
		\param other the array to compare against
		\returns true if the two arrays are equal
		\sa XsArray_compareArray
	 */
	inline bool operator == (ArrayImpl const& other) const
	{
		return !XsArray_compare(this, &other);
	}

	/*! \brief Tests \a other for inequality
		\param other the array to compare against
		\returns true if the two arrays are not equal
		\sa XsArray_compareArray
	*/
	inline bool operator != (ArrayImpl const& other) const
	{
		return !!XsArray_compare(this, &other);
	}

	//! \copydoc XsArray_reserve
	inline void reserve(XsSize count)
	{
		XsArray_reserve(this, count);
	}

	//! \brief Returns the reserved space in number of items
	inline XsSize reserved() const
	{
		return m_reserved;
	}

	//! \brief Returns the XsArrayDescriptor describing the array
	inline XsArrayDescriptor const& descriptor() const
	{
		return *m_descriptor;
	}

protected:
#ifndef XSENS_NOITERATOR
	/*! \brief STL-style iterator */
	template <ptrdiff_t F, typename R, typename Derived>
	struct IteratorImplBase {
	public:
		//! \brief Difference between two items
		typedef ptrdiff_t difference_type;
		//! \brief The contained type
		typedef T value_type;
		//! \brief Type of pointer
		typedef T* pointer;
		//! \brief Type of reference
		typedef T& reference;
#ifndef XSENS_NO_STL
		//! \brief The category of this type of iterator (random access)
		typedef std::random_access_iterator_tag iterator_category;
#endif
		//! \brief The type of the inherited class that is actually this class
		typedef Derived this_type;
		//! \brief The direction of the iterator, +1 = forward, -1 = reverse
		static const ptrdiff_t direction = F;
	protected:
		//! \brief Basic constructor
		inline explicit IteratorImplBase(void* p = 0) : m_ptr((T*) p) {}
		//! \brief Basic constructor
		inline explicit IteratorImplBase(T* p) : m_ptr(p) {}
		//! \brief Copy constructor
		inline IteratorImplBase(this_type const& i) : m_ptr(i.m_ptr) {}
	public:
		//! \brief Assignment operator
		inline this_type operator =(void* p)
		{
			m_ptr = (T*) p;
			return this_type(m_ptr);
		}
		//! \brief Assignment operator
		inline this_type operator =(T* p)
		{
			m_ptr = p;
			return this_type(m_ptr);
		}
		//! \brief Assignment operator
		inline this_type operator =(this_type const& i)
		{
			m_ptr = i.m_ptr;
			return this_type(m_ptr);
		}
		//! \brief Prefix increment by one operator
		inline this_type operator ++()
		{
			m_ptr = (T*) ptrAt(m_ptr, F);
			return this_type(m_ptr);
		}
		//! \brief Postfix increment by one operator
		inline this_type operator ++(int)
		{
			this_type p(m_ptr);
			m_ptr = (T*) ptrAt(m_ptr, F);
			return p;
		}
		//! \brief Prefix decrement by one operator
		inline this_type operator --()
		{
			m_ptr = (T*) ptrAt(m_ptr, -F);
			return this_type(m_ptr);
		}
		//! \brief Postfix decrement by one operator
		inline this_type operator --(int)
		{
			this_type p(m_ptr);
			m_ptr = (T*) ptrAt(m_ptr, -F);
			return p;
		}
		//! \brief Increment by \a count operator
		inline this_type operator +=(ptrdiff_t count)
		{
			m_ptr = ptrAt(m_ptr, F*count);
			return this_type(m_ptr);
		}
		//! \brief Decrement by \a count operator
		inline this_type operator -=(ptrdiff_t count)
		{
			m_ptr = ptrAt(m_ptr, -F*count);
			return this_type(m_ptr);
		}
		//! \brief Addition by \a count operator
		inline this_type operator +(ptrdiff_t count) const
		{
			return this_type(ptrAt(m_ptr, F*count));
		}
		//! \brief Subtraction by \a count operator
		inline this_type operator -(ptrdiff_t count) const
		{
			return this_type(ptrAt(m_ptr, -F*count));
		}
		/*! \brief Returns the difference in number of items between the two iterators
			\details The function computes the difference between this iterator and \a other. The
			difference may be negative if \a other is 'ahead' of this. The function takes the direction
			of the iterator into consideration.
			\param other The iterator to subtract from this.
			\returns The difference between the two iterators.
		*/
		inline difference_type operator - (const this_type& other) const
		{
			return (F * (reinterpret_cast<char*>(m_ptr) - reinterpret_cast<char*>(other.m_ptr))) / D.itemSize;
		}
		//! \brief Iterator comparison
		inline bool operator == (this_type const& i) const { return m_ptr == i.m_ptr; }
		//! \brief Iterator comparison, taking direction into account
		inline bool operator <= (this_type const& i) const { return (F==1)?(m_ptr <= i.m_ptr):(m_ptr >= i.m_ptr); }
		//! \brief Iterator comparison, taking direction into account
		inline bool operator <  (this_type const& i) const { return (F==1)?(m_ptr <  i.m_ptr):(m_ptr >  i.m_ptr); }
		//! \brief Iterator comparison
		inline bool operator != (this_type const& i) const { return m_ptr != i.m_ptr; }
		//! \brief Iterator comparison, taking direction into account
		inline bool operator >= (this_type const& i) const { return (F==1)?(m_ptr >= i.m_ptr):(m_ptr <= i.m_ptr); }
		//! \brief Iterator comparison, taking direction into account
		inline bool operator >  (this_type const& i) const { return (F==1)?(m_ptr >  i.m_ptr):(m_ptr <  i.m_ptr); }
		//! \brief Dereferencing operator
		inline R& operator *() const { return *(R*) ptr(); }
		//! \brief Pointer operator
		inline R* operator ->() const { return (R*) ptr(); }
		//! \brief Access to internal pointer object, use should be avoided
		inline T* ptr() const { return m_ptr; }
	private:
		//! \brief The internal pointer
		T* m_ptr;
	};
#endif

public:
#ifndef XSENS_NOITERATOR
	/*! \brief A non-const iterator implementation */
	template <ptrdiff_t F>
	struct IteratorImpl : public IteratorImplBase<F, T, IteratorImpl<F> >
	{
	private:
		//! \brief A shorthand for the base object
		typedef IteratorImplBase<F, T, IteratorImpl<F> > ParentType;
	public:
		//! \brief Basic constructor
		inline IteratorImpl(void* p = 0) : ParentType(p) {}
		//! \brief Basic constructor
		inline IteratorImpl(T* p) : ParentType(p) {}
		//! \brief Copy constructor
		inline IteratorImpl(const IteratorImpl& i) : ParentType(i) {}
	};

	/*! \brief A const iterator implementation */
	template <ptrdiff_t F>
	struct IteratorImplConst : public IteratorImplBase<F, T const, IteratorImplConst<F> >
	{
	private:
		//! \brief A shorthand for the base object
		typedef IteratorImplBase<F, T const, IteratorImplConst<F> > ParentType;
	public:
		//! \brief Basic constructor
		inline IteratorImplConst(void* p = 0) : ParentType(p) {}
		//! \brief Basic constructor
		inline IteratorImplConst(T* p) : ParentType(p) {}
		//! \brief Copy constructor
		inline IteratorImplConst(IteratorImpl<F> const& i) : ParentType(i.ptr()) {}
		//! \brief Copy constructor
		inline IteratorImplConst(IteratorImplConst const& i) : ParentType(i) {}
	};

	//! \brief STL-style mutable forward iterator
	typedef IteratorImpl<1> iterator;
	//! \brief STL-style mutable reverse iterator
	typedef IteratorImpl<-1> reverse_iterator;
	//! \brief STL-style mutable const forward iterator
	typedef IteratorImplConst<1> const_iterator;
	//! \brief STL-style mutable const reverse iterator
	typedef IteratorImplConst<-1> const_reverse_iterator;

	/*! \brief STL-style const_iterator to the first data item in the array */
	inline const_iterator begin() const { return const_iterator(m_data); }
	/*! \brief STL-style const_iterator to the first data item past the end of the array */
	inline const_iterator end() const { return begin() + (ptrdiff_t) size(); }

	/*! \brief STL-style const_reverse_iterator to the first data item in the reversed array */
	inline const_reverse_iterator rbegin() const { return rend() - (ptrdiff_t) size(); }
	/*! \brief STL-style const_reverse_iterator to the first data item past the end of the reversed array */
	inline const_reverse_iterator rend() const { return const_reverse_iterator(m_data) + (ptrdiff_t) 1; }

	/*! \brief STL-style iterator to the first data item in the array */
	inline iterator begin() { return iterator(m_data); }
	/*! \brief STL-style iterator to the first data item past the end of the array */
	inline iterator end() { return begin() + (ptrdiff_t) size(); }

	/*! \brief STL-style reverse_iterator to the first data item in the reversed array */
	inline reverse_iterator rbegin() { return rend() - (ptrdiff_t) size(); }
	/*! \brief STL-style reverse_iterator to the first data item past the end of the reversed array */
	inline reverse_iterator rend() { return reverse_iterator(m_data) + (ptrdiff_t) 1; }
#endif
	/*! \brief indexed data access operator */
	inline T & operator[] (XsSize index) const
	{
		assert(index < m_size);
		return *ptrAt(m_data, index);
	}
	/*! \brief indexed data access operator */
	inline T& operator[] (XsSize index)
	{
		assert(index < m_size);
		return *ptrAt(m_data, (ptrdiff_t) index);
	}
	/*! \brief indexed data access \sa operator[] \param index Index of item to access. \returns The item at \a index (by value). */
	inline T const& at(XsSize index) const
	{
#ifdef XSENS_NO_EXCEPTIONS
		assert(index >= m_size);
#else
		if (index >= m_size)
			throw std::out_of_range("index out of range");
#endif
		return *ptrAt(m_data, index);
	}
	/*! \brief indexed data access \sa operator[] \param index Index of item to access. \returns The item at \a index (by value). */
	inline T& at(XsSize index)
	{
#ifdef XSENS_NO_EXCEPTIONS
		assert(index >= m_size);
#else
		if (index >= m_size)
			throw std::out_of_range("index out of range");
#endif
		return *ptrAt(m_data, index);
	}
	/*! \brief Insert \a item at \a index
		\param item The item to insert
		\param index The index to insert the item. When beyond the end of the array, the item is appended.
		\sa XsArray_insert
	*/
	inline void insert(T const& item, XsSize index)
	{
		insert(&item, index, 1);
	}
	/*! \brief Insert \a item at \a index
		\param items The items to insert
		\param index The index to insert the items. When beyond the end of the array, the items are appended.
		\param count The number of items to insert
		\sa XsArray_insert
	*/
	inline void insert(T const* items, XsSize index, XsSize count)
	{
		XsArray_insert(this, index, count, items);
	}

#ifndef XSENS_NOITERATOR
	/*! \brief Insert \a item before iterator \a it
		\param item The item to insert
		\param it The iterator before which to insert the item.
		\sa XsArray_insert
	*/
	inline void insert(T const& item, const_iterator it)
	{
		insert(&item, indexOf(it), 1);
	}
	/*! \brief Insert \a item before iterator \a it
		\param item The item to insert
		\param it The iterator before which to insert the item.
		\sa XsArray_insert
	*/
	inline void insert(T const& item, const_reverse_iterator it)
	{
		insert(&item, indexOf(it), 1);
	}

	/*! \brief Insert \a item at \a index
		\param items The items to insert
		\param it The iterator before which to insert the item.
		\param count The number of items to insert
		\sa XsArray_insert
	*/
	inline void insert(T const* items, const_iterator it, XsSize count)
	{
		insert(items, indexOf(it), count);
	}
	/*! \brief Insert \a item at \a index
		\param items The items to insert
		\param it The iterator before which to insert the item.
		\param count The number of items to insert
		\sa XsArray_insert
	*/
	inline void insert(T const* items, const_reverse_iterator it, XsSize count)
	{
		insert(items, indexOf(it), count);
	}
#endif

	/*! \brief Adds \a item to the end of the array \sa XsArray_insert \param item The item to append to the array. */
	inline void push_back(T const& item)
	{
		insert(&item, (XsSize) -1, 1);
	}
	/*! \brief Removes \a count items from the end of the array \sa XsArray_erase \param count The number items to remove */
	inline void pop_back(XsSize count = 1)
	{
		if (count >= size())
			erase(0, (XsSize) -1);
		else
			erase(size()-count, count);
	}
	/*! \brief Adds \a item to the start of the array \sa XsArray_insert \param item The item to insert at the front of the array */
	inline void push_front(T const& item)
	{
		insert(&item, 0, 1);
	}
	/*! \brief Removes \a count items from the start of the array \param count The number items to remove \sa XsArray_erase */
	inline void pop_front(XsSize count = 1)
	{
		erase(0, count);
	}
	/*! \brief Returns the number of items currently in the array
		\returns The number of items currently in the array
		\sa reserved \sa setSize \sa resize
	*/
	inline XsSize size() const
	{
		return m_size;
	}
	/*! \brief Removes \a count items from the array starting at \a index. \param index The index of the first item to remove. \param count The number of items to remove. */
	inline void erase(XsSize index, XsSize count = 1)
	{
		XsArray_erase(this, index, count);
	}
#ifndef XSENS_NOITERATOR
	/*! \brief Removes the item at iterator \a it. \details \param it The item to remove. \returns An iterator pointing to the next item after the erased item. */
	inline iterator erase(iterator it)
	{
		XsSize idx = indexOf(it);
		erase(idx, 1);
		return (idx < size()) ? ptrAt(m_data, idx) : end();
	}
	/*! \brief Removes the item at iterator \a it. \details \param it The item to remove. \returns An iterator pointing to the next item after the erased item. */
	inline reverse_iterator erase(reverse_iterator it)
	{
		XsSize idx = indexOf(it);
		erase(idx, 1);
		return (idx < size()) ? ptrAt(m_data, idx) : rend();
	}
#endif
	/*! \copydoc XsArray_assign \sa XsArray_assign */
	inline void assign(XsSize count, T const* src)
	{
		XsArray_assign(this, count, src);
	}
	/*! \copydoc XsArray_resize \sa XsArray_resize */
	inline void resize(XsSize count)
	{
		XsArray_resize(this, count);
	}
	/*! \brief Set the size of the array to \a count. \details The contents of the array after this operation are undefined. \param count The desired new size fo the array. \sa XsArray_assign \sa reserve \sa resize */
	inline void setSize(XsSize count)
	{
		if (count != m_size)
			XsArray_assign(this, count, 0);
	}
	/*! \copydoc XsArray_append \sa XsArray_append */
	inline void append(ArrayImpl const& other)
	{
		XsArray_append(this, &other);
	}
	/*! \brief Assignment operator, copies \a other into this, overwriting the old contents \param other The array to copy from \returns A reference to this \sa XsArray_copy */
	inline ArrayImpl& operator=(ArrayImpl const& other)
	{
		if (this != &other)
			XsArray_copy(this, &other);
		return *this;
	}
	/*! \brief Returns whether the array is empty. \details This differs slightly from a straight check for size() != 0 in that it also works for fixed-size XsArrays. \returns true if the array is empty. */
	inline bool empty() const
	{
		return (size() == 0) || (m_data == 0) || (m_flags & XSDF_Empty);
	}

#ifndef XSENS_NOITERATOR
	/*! \brief Return the inheriting object */
	inline I const& inherited() const { return *static_cast<I const*>(this); }

	/*! \brief Return the inheriting object */
	inline I& inherited() { return *static_cast<I*>(this); }

#endif
	/*! \brief Swap the contents of the array with those of \a other. \param other The array to swap contents with. \sa XsArray_swap*/
	inline void swap(ArrayImpl& other)
	{
		XsArray_swap(this, &other);
	}

	/*! \brief Append \a item in a stream-like manner.
		\details This allows for constructing XsArrays easily like this:
		XsInt64Array array = XsInt64Array() << 1 << 2 << 3;
		\param item The item to append to the array.
		\returns A reference to the modified array
		\sa push_back
	*/
#ifndef XSENS_NOITERATOR
	inline I& operator <<(T const& item)
	{
		push_back(item);
		return inherited();
	}
#endif

	/*! \copydoc XsArray_find */
	inline int find(T const& needle) const
	{
		return XsArray_find(this, &needle);
	}
#ifndef XSENS_NOITERATOR
	/*! \brief Returns the linear index of \a it in the array
		\param it The iterator to analyze
		\returns The linear forward index of the item pointed to by \a it. If \a it points to before the
		beginning of the array it returns 0. If it points beyond the end, it returns the current size()
	*/
	template <ptrdiff_t F, typename R, typename Derived>
	XsSize indexOf(IteratorImplBase<F,R,Derived> const& it) const
	{
		ptrdiff_t d = ((char const*) it.ptr() - (char const*) m_data);
		if (d >= 0)
		{
			XsSize r = d/D.itemSize;
			if (r <= size())
				return r;
			return size();
		}
		return 0;
	}
#endif

	/*! \copydoc XsArray_removeDuplicates */
	inline void removeDuplicates()
	{
		XsArray_removeDuplicates(this);
	}
protected: // MRPT
	/*! \internal
		\brief Generic pointer movement function
		\details This function adds \a count items to \a ptr based on the size specified in \a descriptor
		\param descriptor The array descriptor that contains the size of a array item
		\param ptr The pointer to start from
		\param count The number of items to move up or down. count may be negative
		\returns The requested pointer.
		\note The return value loses its constness, take care when using this function directly. In most cases
		it should not be necessary to use this function in user code.
	*/
	inline static T* ptrAt(void const* ptr, ptrdiff_t count)
	{
		return (T*)(void*)(((char*)ptr)+count*D.itemSize);
	}
};
#endif

#endif // file guard
