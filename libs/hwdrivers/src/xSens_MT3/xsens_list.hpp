/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _XSENS_LIST_HPP_2006_06_08
#define _XSENS_LIST_HPP_2006_06_08

#ifndef _XSENS_LIST_H_2006_06_08
#	include "xsens_list.h"
#endif

#ifdef _XSENS_LIST_IO
#	include <iostream>
#endif

/* Jerome Monceaux : bilock@gmail.com
 * Add a specific case for apple
 */
#ifdef __APPLE__
# include <malloc/malloc.h>
#else
# include <malloc.h>
#endif

#include <cstdlib>
#include <cstring>

namespace xsens {

#define CMT_LIST_LINEAR_SEARCH_TRESHOLD	10

template <typename T>
List<T>::List()
{
	m_max = 16;
	m_count = 0;
	m_data = (T*) malloc(m_max * sizeof(T));
//	_ASSERT((void*) m_data != (void*) 0x00392E90);
	m_jcf = NULL;
	m_manage = true;
}

template <typename T>
List<T>::List(uint32_t size)
{
	m_max = size;
	if (m_max == 0)
		m_max = 1;
	m_count = 0;
	m_data = (T*) malloc(m_max * sizeof(T));
	m_jcf = NULL;
	m_manage = true;
}

template <typename T>
List<T>::List(const List<T>& src)
{
	m_max = src.m_max;
	if (m_max == 0)
		m_max = 1;
	m_count = src.m_count;
	m_data = (T*) malloc(m_max * sizeof(T));
	m_jcf = NULL;
	if (m_count > 0)
		memcpy(m_data,src.m_data,m_count*sizeof(T));
	m_manage = true;
}

template <typename T>
List<T>::List(const uint32_t size, const T* src)
{
	m_max = size;
	if (m_max == 0)
		m_max = 1;
	m_count = size;
	m_data = (T*) malloc(m_max * sizeof(T));
	m_jcf = NULL;
	if (m_count > 0)
		memcpy(m_data,src,m_count * sizeof(T));
	m_manage = true;
}

template <typename T>
List<T>::List(const uint32_t size, T* src, bool manage)
{
	m_max = size;
	m_count = size;
	m_data = src;
	m_jcf = NULL;
	m_manage = manage;
}

template <typename T>
List<T>::~List()
{
	if (m_jcf != NULL)
		delete m_jcf;
	if (m_manage && m_data != NULL)
		free(m_data);
	m_jcf = NULL;
	m_data = NULL;
}

template <typename T>
void List<T>::deleteAndClear(void)
{
	for (unsigned i=0;i<m_count;++i)
		delete m_data[i];
	m_count = 0;
}

template <typename T>
void List<T>::freeAndClear(void)
{
	for (unsigned i=0;i<m_count;++i)
		free(m_data[i]);
	m_count = 0;
}

template <typename T>
void List<T>::clear(void)
{
	m_count = 0;
}

template <typename T>
void List<T>::resize(uint32_t newSize)
{
	if (m_manage)
	{
		if (newSize == m_max)
			return;
		if (m_count > newSize)
			m_max = m_count;
		else
			m_max = newSize;
		if (m_max == 0)
			m_max = 1;	// size 0 is not allowed

		m_data = (T*) realloc(m_data,m_max * sizeof(T));
	}
}

template <typename T>
void List<T>::append(const T& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = item;
}

template <typename T>
void List<T>::appendList(uint32_t count, const T* lst)
{
	if (m_count+count > m_max)
		resize(m_max + count + m_max/2);
	for (unsigned i = 0; i < count; ++i)
		m_data[m_count++] = lst[i];
}

template <typename T>
void List<T>::appendDeepCopy(const List<T>& source)
{
	if (m_max < source.m_count + m_count)
		resize(source.m_count + m_count);

	for (uint32_t i = 0;i<source.m_count;++i)
		m_data[m_count++] = new T(*source.m_data[i]);
}

template <typename T>
void List<T>::appendShallowCopy(const List<T>& source)
{
	if (m_max < source.m_count + m_count)
		resize(source.m_count + m_count);

	for (uint32_t i = 0;i<source.m_count;++i)
		m_data[m_count++] = source.m_data[i];
}

template <typename T>
template <typename TB>
void List<T>::appendCopy(const TB& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = new TB(item);
}

template <typename T>
template <typename TR>
void List<T>::appendRelated(const TR& item)
{
	if (m_count == m_max)
		resize(m_max + 1 + m_max/2);
	m_data[m_count++] = item;
}

template <typename T>
T& List<T>::last(void) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count == 0)
			throw "List.last: empty list";
	#endif
	return m_data[m_count-1];
}

template <typename T>
T& List<T>::get(const uint32_t index) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw "List.get: index out of bounds";
	#endif
	if (index >= m_count)
		return m_data[m_count-1];
	return m_data[index];
}

template <typename T>
void List<T>::insert(const T& item, const uint32_t index)
{
	if (m_count == m_max)
		resize(1 + m_max + (m_max >> 1));
	for (unsigned i=m_count;i>index;--i)
		m_data[i] = m_data[i-1];
	if (index <= m_count)
		m_data[index] = item;
	else
		m_data[m_count] = item;
	m_count++;
}

template <typename T>
template <typename TB>
void List<T>::insertCopy(const TB& item, const uint32_t index)
{
	if (m_count == m_max)
		resize(1 + m_max + (m_max >> 1));
	for (unsigned i=m_count;i>index;--i)
		m_data[i] = m_data[i-1];
	if (index <= m_count)
		m_data[index] = new TB(item);
	else
		m_data[m_count] = new TB(item);
	m_count++;
}

template <typename T>
T& List<T>::operator [] (const uint32_t index) const XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw "List[]: index out of bounds";
	#endif
	return m_data[index];
}

#if defined(_XSENS_LIST_WITH_MATH) && defined(_XSENS_LIST_IO)

template <typename T>
std::ostream& operator << (std::ostream& os, List<T>& t)
{
	os << '[' << t.length() << "]{ ";
	for (unsigned i=0 ; i<t.length() ; ++i)
		os << t[i] << " ";
	os << '}';
	return os;
}

#ifndef _CMTMATLABHEADERS
#define _CMTMATLABHEADERS
	struct MatlabFileHeader {
		char	description[116];
		int32_t	data_offset1;
		int32_t	data_offset2;
		int16_t	version;
		int16_t	endian;
	};

	struct MatlabDataHeader {
		int32_t data_type;
		int32_t n_bytes;
	};

	struct MatlabMatrixHeader {
		int32_t flags_data_type;
		int32_t flags_data_size;
		int32_t flags0,flags1;

		int32_t dimensions_data_type;
		int32_t dimensions_data_size;
		int32_t dimensions_m, dimensions_n;

		int32_t name_type;
		int32_t name_length;
	};
#endif

//! save list of vectors as matrix in ".mat" file for MATLAB
template <typename T>
void List<T>::saveAsMatlab(const char* filename, const char *varname) const
{
	// new header
	
	size_t i, j;
	MatlabFileHeader file_header;
	MatlabMatrixHeader matrix_header;
	int32_t name_pad;
	MatlabDataHeader inner_header;
	MatlabDataHeader outer_header;

	FILE* fp = fopen(filename,"wb");
	if (fp == NULL)
		return;

	strcpy(file_header.description,"cmtMath/Xsens");
	for (i = strlen(file_header.description); i <116; ++i)
		file_header.description[i] = ' ';
	file_header.data_offset1 = 0;
	file_header.data_offset2 = 0;
	file_header.version = 0x0100;
	file_header.endian = (int16_t) 'M' << 8 | 'I';

	//matlab  mat;

	matrix_header.flags_data_type = 6;
	matrix_header.flags_data_size = 8;
	matrix_header.flags0 = 6;	// mxDOUBLE_CLASS (double precision array)
	matrix_header.flags1 = 0;
	matrix_header.dimensions_data_type = 5;
	matrix_header.dimensions_data_size = 8;
	matrix_header.dimensions_m = m_count;
	matrix_header.dimensions_n = m_data[0]->size();
	matrix_header.name_type = 1;
	
	if ( varname == (char *)NULL )
		matrix_header.name_length = 0;
	else
		matrix_header.name_length = (int32_t) strlen(varname);

	name_pad = matrix_header.name_length & 7;

	inner_header.data_type = 9;	// double
	inner_header.n_bytes = sizeof(double) * m_count * matrix_header.dimensions_n;

	outer_header.data_type = 14;
	outer_header.n_bytes = sizeof(matrix_header) + matrix_header.name_length + name_pad
						+ sizeof(MatlabDataHeader) + inner_header.n_bytes;
	
	fwrite((char*) &file_header,sizeof(MatlabFileHeader),1,fp);
	fwrite((char*) &outer_header,sizeof(MatlabDataHeader),1,fp);
	fwrite((char*) &matrix_header,sizeof(MatlabMatrixHeader),1,fp);
	fwrite(varname,sizeof(char),matrix_header.name_length,fp);

	if (name_pad != 0)
		fwrite("\0\0\0\0\0\0\0",sizeof(char),8-name_pad,fp);

	fwrite((char*) &inner_header,sizeof(MatlabDataHeader),1,fp);

	// write actual data
	// column major order: ORDER == COL_ORDER
	double tmp;
	for ( j = 0; j < (size_t) matrix_header.dimensions_n; j++ )
		for ( i = 0; i < m_count; i++ )
		{
			tmp = (*m_data[i])[(unsigned)j];
			fwrite(&tmp,sizeof(double),1,fp);
		}

	fclose(fp);
}
#endif	// _XSENS_LIST_WITH_MATH && _XSENS_LIST_IO

template <typename T>
void List<T>::qSort(uint32_t left, uint32_t right)
{
	uint32_t l_hold, r_hold;
	T pivot;

	l_hold = left;
	r_hold = right;
	pivot = m_data[left];
	while (left < right)
	{
		while (!(m_data[right] < pivot) && (left < right))
			right--;
		if (left != right)
		{
			m_data[left] = m_data[right];
			left++;
		}
		while (!(pivot < m_data[left]) && (left < right))
			left++;
		if (!(left == right))
		{
			m_data[right] = m_data[left];
			right--;
		}
	}
	m_data[left] = pivot;
	if (l_hold < left)
		qSort(l_hold, left-1);
	if (r_hold > left)
		qSort(left+1, r_hold);
}

template <typename T>
void List<T>::qSortDeref(uint32_t left, uint32_t right)
{
	uint32_t l_hold, r_hold;
	T pivot;

	l_hold = left;
	r_hold = right;
	pivot = m_data[left];
	while (left < right)
	{
		while (!(*m_data[right] < *pivot) && (left < right))
			right--;
		if (left != right)
		{
			m_data[left] = m_data[right];
			left++;
		}
		while (!(*pivot < *m_data[left]) && (left < right))
			left++;
		if (!(left == right))
		{
			m_data[right] = m_data[left];
			right--;
		}
	}
	m_data[left] = pivot;
	if (l_hold < left)
		qSortDeref(l_hold, left-1);
	if (r_hold > left)
		qSortDeref(left+1, r_hold);
}

//#define XSENS_LIST_QSORT
//#define XSENS_LIST_COMBSORT
#define XSENS_LIST_JOBSORT
// when nothing is defined, bubble sort is used

template <typename T>
void List<T>::sortAscending(void)
{
	if (m_count <= 1)
		return;
#if defined(XSENS_LIST_QSORT)
	qSort(0,m_count-1);
#elif defined(XSENS_LIST_COMBSORT)

	uint32_t gap = m_count;
	double dgap;
	uint32_t swaps = 0;
	T temp;

	while (gap > 1 || swaps > 0)
	{
		if (gap > 1)
		{
			dgap = floor(((double) gap) / 1.247330950103979);
			gap = (uint32_t) dgap;
			if (gap == 10 || gap == 9)
				gap = 11;
		}

		uint32_t gappedCount = m_count-gap;
		swaps = 0;
		for (uint32_t i = 0; i < gappedCount; ++i)
		{
			if (m_data[i] > m_data[i+gap])
			{
				temp = m_data[i];
				m_data[i] = m_data[i+gap];
				m_data[i+gap] = temp;
				++swaps;
			}
		}
	}
#elif defined(XSENS_LIST_JOBSORT)
	struct Linker {
		Linker *prev, *next;
		uint32_t index;

		T item;
	};

	auto* list = (Linker*) malloc(m_count*sizeof(Linker));

	list[0].prev = NULL;
	list[0].next = NULL;
	list[0].index = 0;
	list[0].item = m_data[0];

	Linker* curr = list;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		list[i].index = i;
		list[i].item = m_data[i];
		if (m_data[i] < m_data[curr->index])
		{
			while (curr->prev != NULL)
			{
				curr = curr->prev;
				if (!(m_data[i] < m_data[curr->index]))
				{
					// insert after this
					list[i].next = curr->next;
					list[i].prev = curr;
					curr->next->prev = &list[i];
					curr->next = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = NULL;
				list[i].next = curr;
				curr->prev = &list[i];
				curr = &list[i];
			}
		}
		else
		{
			while (curr->next != NULL)
			{
				curr = curr->next;
				if (m_data[i] < m_data[curr->index])
				{
					// insert before this
					list[i].next = curr;
					list[i].prev = curr->prev;
					curr->prev->next = &list[i];
					curr->prev = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = curr;
				list[i].next = NULL;
				curr->next = &list[i];
				curr = &list[i];
			}
		}
	}

	// go to start of list
	while (curr->prev != NULL) curr = curr->prev;

	// copy sorted list back
	for (uint32_t i = 0; i < m_count; ++i)
	{
		m_data[i] = curr->item;
		curr = curr->next;
	}

	free(list);
#else
	uint32_t swaps;
	T temp;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		swaps = 0;
		for (uint32_t j = end-1; j >= i; --j)
		{
			if (m_data[j] < m_data[j-1])
			{
				temp = m_data[j];
				m_data[j] = m_data[j-1];
				m_data[j-1] = temp;
				++swaps;
			}
		}
		if (swaps == 0)
			break;
	}
#endif
}

template <typename T>
void List<T>::sortAscendingDeref(void)
{
	if (m_count <= 1)
		return;
#if defined(XSENS_LIST_QSORT)
	qSortDeref(0,m_count-1);
#elif defined(XSENS_LIST_COMBSORT)

	uint32_t gap = m_count;
	double dgap;
	uint32_t swaps = 0;
	T temp;

	while (gap > 1 || swaps > 0)
	{
		if (gap > 1)
		{
			dgap = floor(((double) gap) / 1.247330950103979);
			gap = (uint32_t) dgap;
			if (gap == 10 || gap == 9)
				gap = 11;
		}

		uint32_t gappedCount = m_count-gap;
		swaps = 0;
		for (uint32_t i = 0; i < gappedCount; ++i)
		{
			if (*m_data[i+gap] < *m_data[i])
			{
				temp = m_data[i];
				m_data[i] = m_data[i+gap];
				m_data[i+gap] = temp;
				++swaps;
			}
		}
	}
#elif defined(XSENS_LIST_JOBSORT)
	struct Linker {
		Linker *prev, *next;
		uint32_t index;

		T item;
	};

	auto* list = (Linker*) malloc(m_count*sizeof(Linker));

	list[0].prev = NULL;
	list[0].next = NULL;
	list[0].index = 0;
	list[0].item = m_data[0];

	Linker* curr = list;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		list[i].index = i;
		list[i].item = m_data[i];
		if (*m_data[i] < *m_data[curr->index])
		{
			while (curr->prev != NULL)
			{
				curr = curr->prev;
				if (!(*m_data[i] < *m_data[curr->index]))
				{
					// insert after this
					list[i].next = curr->next;
					list[i].prev = curr;
					curr->next->prev = &list[i];
					curr->next = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = NULL;
				list[i].next = curr;
				curr->prev = &list[i];
				curr = &list[i];
			}
		}
		else
		{
			while (curr->next != NULL)
			{
				curr = curr->next;
				if (*m_data[i] < *m_data[curr->index])
				{
					// insert before this
					list[i].next = curr;
					list[i].prev = curr->prev;
					curr->prev->next = &list[i];
					curr->prev = &list[i];
					curr = &list[i];
					break;
				}
			}
			if (curr != &list[i])
			{
				list[i].prev = curr;
				list[i].next = NULL;
				curr->next = &list[i];
				curr = &list[i];
			}
		}
	}

	// go to start of list
	while (curr->prev != NULL) curr = curr->prev;

	// copy sorted list back
	for (uint32_t i = 0; i < m_count; ++i)
	{
		m_data[i] = curr->item;
		curr = curr->next;
	}

	free(list);
#else
	uint32_t swaps;
	T temp;

	for (uint32_t i = 1; i < m_count; ++i)
	{
		swaps = 0;
		for (uint32_t j = end-1; j >= i; --j)
		{
			if (*(m_data[j]) < *(m_data[j-1]))
			{
				temp = m_data[j];
				m_data[j] = m_data[j-1];
				m_data[j-1] = temp;
				++swaps;
			}
		}
		if (swaps == 0)
			break;
	}
#endif
}

template <typename T>
template <typename T2>
void List<T>::twinSortAscending(List<T2>& twin)
{
	if (m_count <= 1)
		return;

	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (m_count != twin.m_count)
			throw "List.twinSortAscending: sizes do not match";
	#endif
	uint32_t iteration = 0;
	uint32_t mini;
	T tmp;
	T2 tmp2;
	if (m_count > 1)
	while (iteration < m_count-1)
	{
		mini = iteration;
		for (uint32_t i=iteration+1;i<m_count;++i)
		{
			if (m_data[i] < m_data[mini])
				mini = i;
		}
		if (mini != iteration)
		{
			tmp = m_data[mini];
			m_data[mini] = m_data[iteration];
			m_data[iteration] = tmp;

			tmp2 = twin.m_data[mini];
			twin.m_data[mini] = twin.m_data[iteration];
			twin.m_data[iteration] = tmp2;
		}
		++iteration;
	}
}

template <typename T>
void List<T>::remove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw "List.remove: index out of bounds";
	#endif
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
void List<T>::removeTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw "List.removeTail: list size less than remove count";
	#endif
	if (m_count > count)
	{
		m_count -= count;
		return;
	}
	m_count = 0;
}

template <typename T>
void List<T>::deleteAndRemoveTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw "List.deleteAndRemoveTail: list size less than remove count";
	#endif
	if (m_count > count)
	{
		for (unsigned i = 0;i < count;++i)
			delete m_data[--m_count];
		return;
	}
	deleteAndClear();
}

template <typename T>
void List<T>::freeAndRemoveTail(const uint32_t count) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (count > m_count)
			throw "List.freeAndRemoveTail: list size less than remove count";
	#endif
	if (m_count > count)
	{
		for (unsigned i = 0;i < count;++i)
			free(m_data[--m_count]);
		return;
	}
	freeAndClear();
}

template <typename T>
uint32_t List<T>::removeDuplicateEntries(void)
{
	uint32_t removed = 0;
	for (uint32_t i=0;i < m_count; ++i)
	{
		for (uint32_t j=i+1;j < m_count; ++j)
		{
			if (m_data[i] == m_data[j])
			{
				remove(j);
				++removed;
				--j;
			}
		}
	}
	return removed;
}

template <typename T>
uint32_t List<T>::removeDuplicateEntriesDeref(void)
{
	uint32_t removed = 0;
	for (uint32_t i=0;i < m_count; ++i)
	{
		for (uint32_t j=i+1;j < m_count; ++j)
		{
			if (*(m_data[i]) == *(m_data[j]))
			{
				remove(j);
				++removed;
				--j;
			}
		}
	}
	return removed;
}

template <typename T>
void List<T>::deleteAndRemove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw "List.deleteAndRemove: index out of bounds";
	#endif
	delete m_data[index];
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
void List<T>::freeAndRemove(const uint32_t index) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (index >= m_count)
			throw "List.freeAndRemove: index out of bounds";
	#endif
	free(m_data[index]);
	if (index == m_count-1)
	{
		--m_count;
		return;
	}
	--m_count;
	for (unsigned i = index;i < m_count;++i)
		m_data[i] = m_data[i+1];
}

template <typename T>
template <typename TB>
uint32_t List<T>::find(const TB& item) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (((const T*)m_data)[i] == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
uint32_t List<T>::find(const T item, InequalityFunction fnc) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (!fnc(m_data[i],item))
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findDeref(const TB& item) const
{
	for (uint32_t i=0;i<m_count;++i)
		if (*(m_data[i]) == item)
			return i;
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSorted(const TB& item) const
{
	if (m_count < CMT_LIST_LINEAR_SEARCH_TRESHOLD)			// for small lists, it is faster to simply walk the list
		return find(item);

	uint32_t x = m_count;
	uint32_t n = 1;
	uint32_t i;

	while(x >= n)
	{
		i = (x + n) >> 1;

		if (m_data[i-1] == item)
			return i-1;
		if (m_data[i-1] < item)
			n = i+1;
		else
			x = i-1;
	}
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
template <typename TB>
uint32_t List<T>::findSortedDeref(const TB& item) const
{
	if (m_count < CMT_LIST_LINEAR_SEARCH_TRESHOLD)			// for small lists, it is faster to simply walk the list
		return findDeref(item);

	uint32_t x = m_count;
	uint32_t n = 1;
	uint32_t i;

	while(x >= n)
	{
		i = (x + n) >> 1;

		if (*(m_data[i-1]) == item)
			return i-1;
		if (*(m_data[i-1]) < item)
			n = i+1;
		else
			x = i-1;
	}
	return XSENS_LIST_NOTFOUND;
}

template <typename T>
uint32_t List<T>::insertSorted(const T& item)
{
	uint32_t i;
	if (m_count < CMT_LIST_LINEAR_SEARCH_TRESHOLD)
	{
		for (i=0;i<m_count;++i)
			if (item < m_data[i])
			{
				insert(item,i);
				return i;
			}
		append(item);
		return m_count-1;
	}
	else
	{
		uint32_t x = m_count;
		uint32_t n = 1;

		while(x >= n)
		{
			i = (x + n) >> 1;

			if (m_data[i-1] == item)
			{
				insert(item,i-1);
				return i-1;
			}
			if (m_data[i-1] < item)
				n = i+1;
			else
				x = i-1;
		}
		insert(item,n-1);
		return n-1;
	}
}

template <typename T>
uint32_t List<T>::insertSortedDeref(const T& item)
{
	uint32_t i;
	if (m_count < CMT_LIST_LINEAR_SEARCH_TRESHOLD)
	{
		for (i=0;i<m_count;++i)
			if (*item < *m_data[i])
			{
				insert(item,i);
				return i;
			}
		append(item);
		return m_count-1;
	}
	else
	{
		uint32_t x = m_count;
		uint32_t n = 1;

		while(x >= n)
		{
			i = (x + n) >> 1;

			if (*(m_data[i-1]) == *item)
			{
				insert(item,i-1);
				return i-1;
			}
			if (*(m_data[i-1]) < *item)
				n = i+1;
			else
				x = i-1;
		}
		insert(item,n-1);
		return n-1;
	}
}

template <typename T>
template <typename TB>
uint32_t List<T>::insertSortedCopy(const TB& item)
{
	uint32_t i;
	if (m_count < CMT_LIST_LINEAR_SEARCH_TRESHOLD)
	{
		for (i=0;i<m_count;++i)
			if (item < m_data[i])
			{
				insertCopy<TB>(item,i);
				return i;
			}
		append(item);
		return m_count-1;
	}
	else
	{
		uint32_t x = m_count;
		uint32_t n = 1;

		while(x >= n)
		{
			i = (x + n) >> 1;

			if (m_data[i-1] == item)
			{
				insertCopy<TB>(item,i-1);
				return i-1;
			}
			if (m_data[i-1] < item)
				n = i+1;
			else
				x = i-1;
		}
		insertCopy<TB>(item,n-1);
		return n-1;
	}
}

template <typename T>
void List<T>::deleteItemsOnDestroy(void)
{
	if (m_jcf != NULL)
	{
		m_jcf->disable();
		delete m_jcf;
	}
	m_jcf = new JanitorClassFunc<List<T>, void>(*this,&List<T>::deleteAndClear);
}

template <typename T>
void List<T>::freeItemsOnDestroy(void)
{
	if (m_jcf != NULL)
	{
		m_jcf->disable();
		delete m_jcf;
	}
	m_jcf = new JanitorClassFunc<List<T>, void>(*this,&List<T>::freeAndClear);
}

template <typename T>
template <typename TB>
void List<T>::isDeepCopyOf(const List<T>& source)
{
	m_count = 0;
	if (m_max < source.m_count)
		resize(source.m_count);
	m_count = source.m_count;
	for (uint32_t i = 0;i<m_count;++i)
		m_data[i] = new TB(*source.m_data[i]);
}

template <typename T>
void List<T>::isShallowCopyOf(const List<T>& x)
{
	m_count = 0;
	if (m_max < x.m_count)
		resize(x.m_count);
	m_count = x.m_count;
	for (uint32_t i = 0;i<m_count;++i)
		m_data[i] = x.m_data[i];
}

template <typename T>
void List<T>::swap(const uint32_t i, const uint32_t j) XSENS_LIST_THROW
{
	#ifdef _XSENS_LIST_RANGE_CHECKS
		if (i >= m_count || j >= m_count)
			throw "List.swap: index out of bounds";
	#endif
	T tmp = m_data[i];
	m_data[i] = m_data[j];
	m_data[j] = tmp;
}

template <typename T>
void List<T>::reverse(void)
{
	uint32_t half = m_count / 2;
	for (uint32_t i = 0, end=m_count-1; i < half; ++i,--end)
	{
		T tmp = m_data[i];
		m_data[i] = m_data[end];
		m_data[end] = tmp;
	}
}

} // end of xsens namespace

#endif	// _XSENS_LIST_HPP_2006_06_08

