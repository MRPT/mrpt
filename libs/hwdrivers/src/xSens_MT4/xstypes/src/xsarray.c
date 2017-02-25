/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsarray.h"
#include "xsatomicint.h"
#include <stdlib.h>

#ifndef XSENS_NO_ALLOC_TRACKING
XsAtomicInt XSTYPES_DLL_API XsArray_allocCount = XSATOMICINT_INITIALIZER;	//!< The number of times XsArray_ functions have allocated memory
XsAtomicInt XSTYPES_DLL_API XsArray_freeCount = XSATOMICINT_INITIALIZER;	//!< The number of times XsArray_ functions have freed memory
#define INC_ALLOC()		XsAtomicInt_preIncrement(&XsArray_allocCount)
#define INC_FREE()		XsAtomicInt_preIncrement(&XsArray_freeCount)
#else
#define INC_ALLOC()		((void)0)
#define INC_FREE()		((void)0)
#endif

/*! \class XsArray
	\brief Provides generic storage for data in an array and manipulation operations on that data
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \cond NODOXYGEN */
#define elemSize(thisArray)			(thisArray->m_descriptor->itemSize)
#define elemAtX(b, i, thisArray)	((void*)(((char*) (b))+((i)*elemSize(thisArray))))
#define elemAt(b, i)				elemAtX(b, i, thisArray)
/*! \endcond */

/*! \relates XsArray
	\brief Initializes the XsArray with space for \a count items and copies them from \a src 
	\details This function initializes the object reserving \a count items in the buffer. \a count may
	be 0. If \a src is not 0, \a count items from \a src will be copied.
	\param descriptor The descriptor of the data in the list
	\param count The number of items to reserve space for. When \a src is not NULL, thisArray is also the number of items copied from \a src
	\param src A pointer to an array of objects to copy, may be NULL, ignored when \a count is 0
*/
void XsArray_construct(void* thisPtr, XsArrayDescriptor const* const descriptor, XsSize count, void const* src)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	*((XsArrayDescriptor const**) &thisArray->m_descriptor) = descriptor;
	*((XsSize*) &thisArray->m_size) = count;

	if (thisArray->m_size)
	{
		// init to size
		*((void**) &thisArray->m_data) = malloc(thisArray->m_size*elemSize(thisArray));
		INC_ALLOC();

		// init the configurations
		if (src)
		{
			XsSize i;
			assert(thisArray->m_descriptor->itemCopyConstruct);
			for (i=0; i<thisArray->m_size; ++i)
				thisArray->m_descriptor->itemCopyConstruct(elemAt(thisArray->m_data, i), elemAt(src, i));
		}
		else if (thisArray->m_descriptor->itemConstruct)
		{
			XsSize i;
			for (i=0; i<thisArray->m_size; ++i)
				thisArray->m_descriptor->itemConstruct(elemAt(thisArray->m_data, i));
		}
	}
	else
		*((void**) &thisArray->m_data) = 0;
	*((XsSize*) &thisArray->m_reserved) = thisArray->m_size;
	*((int*) &thisArray->m_flags) = XSDF_Managed;
}

/*! \relates XsArray
	\brief Initializes the XsArray with a copy of \a src 
	\param src A pointer to the objects to copy. The object may be empty, but src may not be 0
*/
void XsArray_copyConstruct(void* thisPtr, void const* src)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	XsArray const* srcArray = (XsArray const*) src;
	assert(srcArray);
	XsArray_construct(thisArray, srcArray->m_descriptor, srcArray->m_size, srcArray->m_data);
}

/*! \relates XsArray
	\brief Clears and frees memory allocated by the XsArray
	\note After XsArray_destruct is called, the object is empty but valid,
	ie. it can be used as if XsArray_construct has been called on it.
*/
void XsArray_destruct(void* thisPtr)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	if (thisArray->m_data && (thisArray->m_flags & XSDF_Managed))
	{
		XsSize i;
		// clear contents
		if (thisArray->m_descriptor->itemDestruct)
			for (i=0; i<thisArray->m_reserved; ++i)
				thisArray->m_descriptor->itemDestruct(elemAt(thisArray->m_data, i));
		free((void*) thisArray->m_data);
		INC_FREE();
	}
	// init to 0
	*((void**) &thisArray->m_data) = 0;
	*((XsSize*) &thisArray->m_size) = 0;
	*((XsSize*) &thisArray->m_reserved) = 0;
	*((int*) &thisArray->m_flags) = (thisArray->m_flags & (XSDF_Managed | XSDF_FixedSize));
}

/*! \relates XsArray
	\brief Reinitializes the XsArray with space for \a count items and copies them from \a src 
	\details This function reinitializes the object reserving space for at least \a count items in the
	buffer. \a count may be 0. If \a src is not 0, \a count items will be copied from \a src.
	Previous data will be cleared automatically, but the reserved space will not be reduced.
	\param count the number of items in src
	\param src a pointer to an array of output configuration objects
	\sa XsArray_reserve
*/
void XsArray_assign(void* thisPtr, XsSize count, void const* src)
{
	XsSize i;
	XsArray* thisArray = (XsArray*) thisPtr;

	// check if we need to reallocate
	if (count > thisArray->m_reserved)
	{
		assert(thisArray->m_flags & XSDF_Managed);

		if (thisArray->m_data)
			XsArray_destruct(thisArray);
		XsArray_construct(thisArray, thisArray->m_descriptor, count, src);
		return;
	}

	// no reallocation necessary, clear excess objects
	if (thisArray->m_descriptor->itemDestruct)
		for (i=count; i<thisArray->m_size; ++i)
			thisArray->m_descriptor->itemDestruct(elemAt(thisArray->m_data, i));

	if (src)
		for (i=0; i<count; ++i)
			thisArray->m_descriptor->itemCopy(elemAt(thisArray->m_data, i), elemAt(src, i));

	*((XsSize*) &thisArray->m_size) = count;
}

/*! \relates XsArray
	\brief Resizes the existing list to \a count items
	\details This function will keep the data of the remaining items intact.
	\param count the number of items the list should have
	\sa XsArray_reserve \sa XsArray_assign
*/
void XsArray_resize(void* thisPtr, XsSize count)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	if (thisArray->m_size == count)
		return;
	if (thisArray->m_size == 0)
	{
		XsArray_assign(thisArray, count, 0);
		return;
	}
	if (count < thisArray->m_size)
	{
		XsArray_erase(thisArray, count, thisArray->m_size - count);
		return;
	}
	if (count > thisArray->m_reserved)
		XsArray_reserve(thisArray, count);
	*((XsSize*) &thisArray->m_size) = count;
}

/*! \relates XsArray
	\brief Reserves space for \a count items
	\details This function reserves space for exactly \a count items unless \a count is less than the 
	current list size. The function will retain the current data in the list.
	\param count The number of items to reserve space for
	\sa XsArray_assign
*/
void XsArray_reserve(void* thisPtr, XsSize count)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	XsArray tmp = { 0, thisArray->m_size, 0, XSDF_Managed, thisArray->m_descriptor };
	XsSize i;

	if (count < thisArray->m_size)
		count = thisArray->m_size;

	if (count == thisArray->m_reserved)
		return;

	if (!(thisArray->m_flags & XSDF_Managed))
	{
		// attempting thisArray on an unmanaged list is ignored silently
		return;
	}

	if (!count)
	{
		XsArray_destruct(thisArray);
		return;
	}

	*((XsSize*) &tmp.m_reserved) = count;

	// init to size
	*((void**) &tmp.m_data) = malloc(tmp.m_reserved*elemSize(thisArray));
	INC_ALLOC();

	if (thisArray->m_descriptor->itemConstruct)
		for (i=0; i<tmp.m_reserved; ++i)
			thisArray->m_descriptor->itemConstruct(elemAt(tmp.m_data, i));
	
	for (i=0; i<thisArray->m_size; ++i)
		thisArray->m_descriptor->itemSwap(elemAt(thisArray->m_data, i), elemAt(tmp.m_data, i));

	XsArray_destruct(thisArray);
	XsArray_swap(thisArray, &tmp);
}

/*! \relates XsArray
	\brief Copy the contents of \a src to thisArray */
void XsArray_copy(void* thisPtr, void const* src)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	XsArray const* srcArray = (XsArray const*) src;

	if (srcArray == thisArray)
	{
		return;
	}
	XsArray_assign(thisArray, srcArray->m_size, srcArray->m_data);
}

/*! \relates XsArray
	\brief Appends the \a other list to thisArray list
	\param other The list to append to thisArray list. \a other may point to thisArray list
*/
void XsArray_append(void* thisPtr, void const* other)
{
	XsSize i;
	XsArray* thisArray = (XsArray*) thisPtr;
	XsArray const* otherArray = (XsArray const*) other;

	if (otherArray->m_size == 0)
		return;

	if (otherArray == thisArray)
	{
		if (thisArray->m_size + thisArray->m_size > thisArray->m_reserved)
			XsArray_reserve(thisArray, thisArray->m_size + thisArray->m_size);	// maybe reserve more here?

		for (i=0; i<thisArray->m_size; ++i)
			thisArray->m_descriptor->itemCopy(elemAt(thisArray->m_data, i+thisArray->m_size), elemAt(thisArray->m_data, i));

		*((XsSize*) &thisArray->m_size) = thisArray->m_size + thisArray->m_size;
		return;
	}

	if (thisArray->m_size == 0)
	{
		XsArray_copy(thisArray, otherArray);
		return;
	}

	if (thisArray->m_size + otherArray->m_size > thisArray->m_reserved)
		XsArray_reserve(thisArray, thisArray->m_size + otherArray->m_size);	// maybe reserve more here?

	for (i=0; i<otherArray->m_size; ++i)
		thisArray->m_descriptor->itemCopy(elemAt(thisArray->m_data, i+thisArray->m_size), elemAt(otherArray->m_data, i));

	*((XsSize*) &thisArray->m_size) = thisArray->m_size + otherArray->m_size;
}

/*! \relates XsArray
	\brief Insert \a count items from \a src at \a index in the array
	\param index The index to use for inserting. Anything beyond the end of the array (ie. -1) will
	append to the actual end of the array.
	\param count The number of items to insert
	\param src The items to insert, may not be 0 unless count is 0
*/
void XsArray_insert(void* thisPtr, XsSize index, XsSize count, void const* src)
{
	XsSize s;
	XsArray* thisArray = (XsArray*) thisPtr;
	int i,d = (int) count;
	if (thisArray->m_size + count > thisArray->m_reserved)
		XsArray_reserve(thisArray, ((thisArray->m_size + count)*3)/2);		// we reserve 50% more space here to handle multiple sequential insertions efficiently

	// fix index if beyond end of list
	if (index > thisArray->m_size)
		index = thisArray->m_size;

	// move items to the back by swapping
	for (i = ((int)thisArray->m_size)-1; i >= (int) index; --i)
		thisArray->m_descriptor->itemSwap(elemAt(thisArray->m_data, i), elemAt(thisArray->m_data, i+d));

	// copy items to the array
	for (s = 0; s < count; ++s)
		thisArray->m_descriptor->itemCopy(elemAt(thisArray->m_data, s+index), elemAt(src, s));

	// update size
	*((XsSize*) &thisArray->m_size) = thisArray->m_size + count;
}

/*! \relates XsArray
	\brief Swap the contents of \a a with those of \a b
	\details Where possible, the pointers and administrative values are swapped. If for some reason
	thisArray is not possible, the lists are swapped one element at a time.
	\param a The list to swap with \a b
	\param b The list to swap with \a a
*/
void XsArray_swap(void* a, void* b)
{
	XsArray* aArray = (XsArray*) a;
	XsArray* bArray = (XsArray*) b;

	if (!aArray->m_data && !bArray->m_data)
		return;
	if ((!aArray->m_data || (aArray->m_flags & XSDF_Managed)) && (!bArray->m_data || (bArray->m_flags & XSDF_Managed)))
	{
		// administrative swap
		XsArray tmp;
		*((void**) &tmp.m_data) = aArray->m_data;
		*((void**) &aArray->m_data) = bArray->m_data;
		*((void**) &bArray->m_data) = tmp.m_data;

		*((XsSize*) &tmp.m_size) = aArray->m_size;
		*((XsSize*) &aArray->m_size) = bArray->m_size;
		*((XsSize*) &bArray->m_size) = tmp.m_size;

		*((XsSize*) &tmp.m_reserved) = aArray->m_reserved;
		*((XsSize*) &aArray->m_reserved) = bArray->m_reserved;
		*((XsSize*) &bArray->m_reserved) = tmp.m_reserved;

		*((int*) &tmp.m_flags) = aArray->m_flags;
		*((int*) &aArray->m_flags) = bArray->m_flags;
		*((int*) &bArray->m_flags) = tmp.m_flags;
	} else {
		// elementwise swap
		XsSize i;
		assert(aArray->m_size == bArray->m_size);
		for (i = 0; i < aArray->m_size; ++i)
			aArray->m_descriptor->itemSwap(elemAtX(aArray->m_data, i, aArray), elemAtX(bArray->m_data, i, bArray));
	}
}

/*! \relates XsArray
	\brief Removes a \a count items from the list starting at \a index
*/
void XsArray_erase(void* thisPtr, XsSize index, XsSize count)
{
	XsSize i, newCount;
	XsArray* thisArray = (XsArray*) thisPtr;

	if (count == 0 || index >= thisArray->m_size)
		return;

	if (count+index > thisArray->m_size)
		count = thisArray->m_size - index;

	newCount = thisArray->m_size - count;

	// move items into the gap by swapping
	for (i = index; i < newCount; ++i)
		thisArray->m_descriptor->itemSwap(elemAt(thisArray->m_data, i), elemAt(thisArray->m_data, i+count));

	*((XsSize*) &thisArray->m_size) = newCount;
}

/*! \relates XsArray
	\brief Returns non-zero if the lists are different, 0 if they're equal
	\details This function compares the two lists in-order
	\param a The left hand side of the comparison
	\param b The right hand side of the comparison
	\return -1 if \a a is smaller in some way than \a b, 1 if it is larger in some way
	and 0 if both lists are equal. Please note that not all lists have items that can be accurately
	tested for less than or greater than, but can be tested for (in-)equality. So the sign of the
	return value should be treated with knowledge of the data type in mind.
	\sa XsArray_compareSet
*/
int XsArray_compare(void const* a, void const* b)
{
	XsSize i;
	XsArray const* aArray = (XsArray const*) a;
	XsArray const* bArray = (XsArray const*) b;

	if (aArray == bArray)
		return 0;

	if (aArray->m_size != bArray->m_size)
		return (aArray->m_size < bArray->m_size)?-1:1;

	assert(aArray->m_descriptor->itemCompare);
	// we could theoretically only check the sizes and ignore the element-comparison in thisArray case
	for (i = 0; i < aArray->m_size; ++i) // loop over all elements of the lists
	{
		int r = aArray->m_descriptor->itemCompare(elemAtX(aArray->m_data, i, aArray), elemAtX(bArray->m_data, i, bArray));
		if (r)
			return r;
	}
	return 0;
}

/*! \relates XsArray
	\brief Returns -1 if \a a is smaller in some way than \a b, 1 if it is larger in some way and 0 if both lists are equal
	\details This function compares the two lists out-of-order
	\param a The left hand side of the comparison
	\param b The right hand side of the comparison
	\return -1 if \a a is smaller in some way than \a b, 1 if it is larger in some way
	and 0 if both lists are equal. Please note that not all lists have items that can be accurately
	tested for less than or greater than, but can be tested for (in-)equality. So the sign of the
	return value should be treated with knowledge of the data type in mind.
	\sa XsArray_compare
*/
int XsArray_compareSet(void const* a, void const* b)
{
	XsSize n, m;
	XsArray const* aArray = (XsArray const*) a;
	XsArray const* bArray = (XsArray const*) b;

	if (aArray == bArray)
		return 0;

	if (aArray->m_size != bArray->m_size)
		return (aArray->m_size < bArray->m_size)?-1:1;

	for (n = 0; n < aArray->m_size; ++n) // loop over all elements of list aArray
	{
		int found = 0;
		for (m = 0; m < bArray->m_size; ++m)	// loop over all elements of list bArray
		{
			if (aArray->m_descriptor->itemCompare(elemAtX(aArray->m_data, n, aArray), elemAtX(bArray->m_data, m, bArray)) == 0)
			{
				found = 1;
				break;
			}
		}
		if (!found)
			return -1;
	}
	return 0;
}

/*! \relates XsArray
	\brief Returns the index of \a needle in the list or -1 if it wasn't found
	\details The search does not assume any kind of ordering of the items so in a worst-case scenario it
	will go through the entire list.
	\param needle A pointer to the value to search for
	\returns The index of where \a needle was found or -1 if it wasn't found.
*/
int XsArray_find(void const* thisPtr, void const* needle)
{
	XsSize i;
	XsArray const* thisArray = (XsArray const*) thisPtr;

	assert(thisArray->m_descriptor->itemCompare);
	// we could theoretically only check the sizes and ignore the element-comparison in thisArray case
	for (i = 0; i < thisArray->m_size; ++i) // loop over all elements of the lists
		if (!thisArray->m_descriptor->itemCompare(elemAt(thisArray->m_data, i), needle))
			return (int) i;
	return -1;
}

/*! \relates XsArray
	\brief Returns a pointer to the item at the supplied \a index or a null pointer if it is out of bounds
	\param index The index of the item to return
	\returns A pointer to the item or NULL if \a index is out of bounds
*/
void const* XsArray_at(void const* thisPtr, XsSize index)
{
	XsArray const* thisArray = (XsArray const*) thisPtr;
	if (index >= thisArray->m_size)
		return 0;
	return elemAt(thisArray->m_data, index);
}

/*! \relates XsArray
	\brief Returns a pointer to the item at the supplied \a index or a null pointer if it is out of bounds
	\param index The index of the item to return
	\returns A pointer to the item or NULL if \a index is out of bounds
*/
void* XsArray_atIndex(void* thisPtr, XsSize index)
{
	XsArray* thisArray = (XsArray*) thisPtr;
	if (index >= thisArray->m_size)
		return 0;
	return elemAt(thisArray->m_data, index);
}

/*! \relates XsArray
	\brief Removes duplicate entries from the array, keeping only the first instance of each value
	\todo Optimize for speed by grouping erases
*/
void XsArray_removeDuplicates(void* thisPtr)
{
	XsSize i,j;
	XsArray* thisArray = (XsArray*) thisPtr;
	if (thisArray->m_size > 1)
	{
		for (i = 0; i < thisArray->m_size-1; ++i)
		{
			for (j = thisArray->m_size-1; j > i; --j)
			{
				if (!thisArray->m_descriptor->itemCompare(elemAt(thisArray->m_data, i), elemAt(thisArray->m_data, j)))
				{
					XsArray_erase(thisPtr, j, 1);
				}
			}
		}
	}
}

/*! @} */ 
