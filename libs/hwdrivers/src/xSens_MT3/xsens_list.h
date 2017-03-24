/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _XSENS_LIST_H_2006_06_08
#define _XSENS_LIST_H_2006_06_08

#include <mrpt/utils/mrpt_stdint.h>

#ifndef _JANITORS_H_2006_05_01
#	include "xsens_janitors.h"
#endif

#ifdef _XSENS_COMPILE
#	ifndef _XSENS_LIST_WITH_MATH
#		define _XSENS_LIST_WITH_MATH
#	endif
#	ifndef _XSENS_LIST_IO
#		define _XSENS_LIST_IO
#	endif
#endif

#ifdef _DEBUG
#	define _XSENS_LIST_RANGE_CHECKS
#endif

#ifdef _XSENS_LIST_WITH_MATH
#	include <math.h>
#	ifndef _XSENS_MATH_H_2006_05_31
#		include "xsens_math.h"
#	endif
#endif

#define XSENS_LIST_NOTFOUND	0xFFFFFFFF

#ifdef _XSENS_LIST_RANGE_CHECKS
//#	define XSENS_LIST_THROW throw(...)   // JLBC: Changed for MRPT in Linux in Debug
#	define XSENS_LIST_THROW
#else
#	define XSENS_LIST_THROW
#endif

namespace xsens {

	/*! \brief Dynamic list class

		This class can store items of the given type. If the type supports the < operator
		it can also be sorted.
		Items in the list can be accessed through the [] operator or the get() function.

		Do NOT use any item type that requires a constructor to work correctly. Pointers to these
		objects can work though.
	*/
	template <typename T>
	class List
	{
	private:
		void operator = (const List& list);	//!< intentionally NOT implemented due to ambiguous nature
			//! Sorts the list in an ascending order, using the T::< operator.
		void qSort(uint32_t left, uint32_t right);
			//! Sorts the list in an ascending order, using the T::< operator on dereferenced list items.
		void qSortDeref(uint32_t left, uint32_t right);

	protected:
		T* m_data;							//!< The array containing the items
		uint32_t m_max;				//!< The current size of the data array
		uint32_t m_count;				//!< The number of items currently in the list
		JanitorClassFunc<List<T> >* m_jcf;	//!< Used to clean up the list on exit
		bool m_manage;

			//! Construct a list as a reference to a raw list
		List(const uint32_t size, T* src, bool manage);
	public:

			//! A comparison function type, should return -1, 0 or 1 for <, == and >
		typedef int32_t (*cmpFunc) (const T&,const T&);

			//! Standard constructor, creates an empty list with some room for items.
		List();
			//! Construct a list with a capacity of at least the given size.
		List(const uint32_t size);
			//! Construct a list as a direct copy of another list
		List(const List<T>& src);
			//! Construct a list as a copy of a raw list
		List(const uint32_t size, const T* src);
			//! Destroy the list. This does NOT automatically delete items IN the list.
		~List();

			//! Calls delete for all items in the list and then clears the list.
		void deleteAndClear(void);
			//! Calls free for all items in the list and then clears the list.
		void freeAndClear(void);
			//! Clears the list without explicitly deleting anything.
		void clear(void);
			//! Resizes the list to at least the given size.
		void resize(uint32_t newSize);
			//! Adds an item to the end of the list.
		void append(const T& item);
			//! Adds a number of items to the end of the list.
		void appendList(uint32_t count, const T* lst);
			//! Adds the contents of the source list to the end of the list.
		void appendDeepCopy(const List<T>& source);
			//! Adds the contents of the source list to the end of the list.
		void appendShallowCopy(const List<T>& source);
			//! Adds a copy of a referenced item to the end of the list.
			template <typename TB>
		void appendCopy(const TB& item);
			//! Adds a related item to the end of the list, using the T = TR operator.
			template <typename TR>
		void appendRelated(const TR& item);
			//! Removes an item at the given index in the list.
		void remove(const uint32_t index) XSENS_LIST_THROW;
			//! Swaps two items in the list.
		void swap(const uint32_t i, const uint32_t j) XSENS_LIST_THROW;
			//! Removes an item at the given index in the list.
		void deleteAndRemove(const uint32_t index) XSENS_LIST_THROW;
			//! Removes an item at the given index in the list.
		void freeAndRemove(const uint32_t index) XSENS_LIST_THROW;
			//! Retrieves the last item.
		T& last(void) const XSENS_LIST_THROW;
			//! Retrieves the item at the given index. An index beyond the end returns the first item.
		T& get(const uint32_t index) const XSENS_LIST_THROW;
			//! Retrieves the item at the given index. An index beyond the end probably causes an exception.
		T& operator [] (const uint32_t index) const XSENS_LIST_THROW;
			//! Inserts an item at the given index, shifting any items below it down one spot.
		void insert(const T& item, const uint32_t index);
			//! Inserts a copy of the referenced item at the given index, shifting any items below it down one spot.
			template <typename TB>
		void insertCopy(const TB& item, const uint32_t index);
			//! Assumes the list is sorted and inserts the item at the appropriate spot.
		uint32_t insertSorted(const T& item);
			//! Assumes the list is sorted by dereferenced values and inserts the item at the appropriate spot.
		uint32_t insertSortedDeref(const T& item);
			//! Assumes the list is sorted and inserts a copy of the referenced item at the appropriate spot.
			template <typename TB>
		uint32_t insertSortedCopy(const TB& item);
			//! Returns the number of items currently in the list.
		uint32_t length(void) const { return m_count; }
			//! Sorts the list in an ascending order, using the T::< operator.
		void sortAscending(void);
			//! Sorts the list in an ascending order, using the T::< operator on dereferenced list items.
		void sortAscendingDeref(void);
			//! Sorts the first list in an ascending order, using the T::< operator, the second list will be updated the same way.
			template <typename T2>
		void twinSortAscending(List<T2>& twin);
			//! Finds an item in an unsorted list (walk over all items) using the T::== operator
			template <typename TB>
		uint32_t find(const TB& item) const;
			//! Finds an item in an unsorted list (walk over all items) using the T::== operator on dereferenced list items
			template <typename TB>
		uint32_t findDeref(const TB& item) const;
			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators
			template <typename TB>
		uint32_t findSorted(const TB& item) const;
			//! Finds an item in a sorted list (binary search) using the T::== and T::< operators on dereferenced list items
			template <typename TB>
		uint32_t findSortedDeref(const TB& item) const;
			//! Reverse the order of the list, useful for sorted lists that are read/created in the reverse order
		void reverse(void);
			//! Removes items from the end of the list.
		void removeTail(const uint32_t count) XSENS_LIST_THROW;
		void deleteAndRemoveTail(const uint32_t count) XSENS_LIST_THROW;
		void freeAndRemoveTail(const uint32_t count) XSENS_LIST_THROW;

			//! Type for an equality compare function, should return true when NOT equal
		typedef int32_t (__cdecl * InequalityFunction)(const T&, const T&);
			//! Finds an item in an unsorted list (walk over all items) using the given inequality function
		uint32_t find(const T item, InequalityFunction fnc) const;

		void deleteItemsOnDestroy(void);
		void freeItemsOnDestroy(void);

			//! Removes any duplicate entries and returns the number of items removed. Items are compared directly.
		uint32_t removeDuplicateEntries(void);
			//! Removes any duplicate entries and returns the number of items removed. Items are compared after dereferencing.
		uint32_t removeDuplicateEntriesDeref(void);

			//! Make a copy of the list, duplicating list items i with: copy[i] = new TB(*source[i])
			template <typename TB>
		void isDeepCopyOf(const List<T>& source);
			//! Overwrites the current list with a shallow (memcopy) copy of another list.
		void isShallowCopyOf(const List<T>& source);

			//! Returns the start of the linear data buffer
		const T* getBuffer(void) const { return m_data; }
		#ifdef _XSENS_LIST_IO
			#ifdef _XSENS_LIST_WITH_MATH
					//! Saves the list as a matrix in a .mat file. This assumes that T is a Vector<T,E>
				void saveAsMatlab(const char* filename, const char *varname) const;
			#endif
		#endif
	};

	class IntList : public List<uint32_t>
	{
	private:
		void operator = (const IntList& list);	//!< intentionally NOT implemented due to ambiguous nature
	public:
			//! Standard constructor, creates an empty list with some room for items.
		IntList() : List<uint32_t>() {}
			//! Construct a list with a capacity of at least the given size.
		IntList(const uint32_t size) : List<uint32_t>(size) {}
			//! Construct a list as a direct copy of another list
		IntList(const IntList& src) : List<uint32_t>(src) {}
			//! Construct a list as a reference to a raw list
		IntList(const uint32_t size, uint32_t* src) : List<uint32_t>(size,src,false) {}

		bool operator == (const IntList& lst);

		void addValue(int32_t value);
		int32_t deserialize(const char* str);
		int32_t readFromString(const char* str);
		int32_t serialize(char* buffer) const;
		void setIncremental(const uint32_t start, const uint32_t end, const int32_t step);
		int32_t writeToString(char* buffer) const;
		int32_t writeToStringHex(char* buffer) const;
	};
} // end of xsens namespace

#endif	// _XSENS_LIST_H_2006_06_08
