/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  circular_buffer_H
#define  circular_buffer_H

// Note: This file is included from "stl_extensions.h"

#include <mrpt/utils/utils_defs.h>
#include <vector>

namespace mrpt
{
	namespace utils
	{
		/** A circular buffer of fixed size (defined at construction-time), implemented with a std::vector as the underlying storage.
		 * \ingroup stlext_grp
		  */
		template <typename T>
		class circular_buffer
		{
		private:
			std::vector<T>	m_data;
			const size_t	m_size;
			size_t			m_next_read,m_next_write;

		public:
			circular_buffer(const size_t size) :
				m_data(size),
				m_size(size),
				m_next_read(0),
				m_next_write(0)
			{
				if (m_size<=2) throw std::invalid_argument("size must be >2");
			}
			//virtual ~circular_buffer()  { }

			/** Insert a copy of the given element in the buffer.
			  * \exception std::out_of_range If the buffer run out of space.
			  */
			void push(T d) {
				m_data[m_next_write++]=d;
				if (m_next_write==m_size) m_next_write=0;

				if (m_next_write==m_next_read)
					throw std::out_of_range("push: circular_buffer is full");
			}

			/** Insert a reference of the given element in the buffer.
			  * \exception std::out_of_range If the buffer run out of space.
			  */
			void push_ref(const T &d) {
				m_data[m_next_write++]=d;
				if (m_next_write==m_size) m_next_write=0;

				if (m_next_write==m_next_read)
					throw std::out_of_range("push: circular_buffer is full");
			}

			/** Insert an array of elements in the buffer.
			  * \exception std::out_of_range If the buffer run out of space.
			  */
			void push_many(T *array_elements, size_t count) {
				while (count--)
					push(*array_elements++);
			}

			/** Retrieve an element from the buffer.
			  * \exception std::out_of_range If the buffer is empty.
			  */
			T pop() {
				if (m_next_read==m_next_write)
					throw std::out_of_range("pop: circular_buffer is empty");

				const size_t i = m_next_read++;
				if (m_next_read==m_size) m_next_read=0;
				return m_data[i];
			}

			/** Retrieve an element from the buffer.
			  * \exception std::out_of_range If the buffer is empty.
			  */
			void pop(T &out_val) {
				if (m_next_read==m_next_write)
					throw std::out_of_range("pop: circular_buffer is empty");

				out_val=m_data[m_next_read++];
				if (m_next_read==m_size) m_next_read=0;
			}

			/** Pop a number of elements into a user-provided array.
			  * \exception std::out_of_range If the buffer has less elements than requested.
			  */
			void pop_many(T *out_array, size_t count) {
				while (count--)
					pop(*out_array++);
			}

			/** Return the number of elements available for read ("pop") in the buffer (this is NOT the maximum size of the internal buffer)
			  * \sa capacity
			  */
			size_t size() const {
				if (m_next_write>=m_next_read)
						return m_next_write-m_next_read;
				else	return m_next_write + (m_size-m_next_read);
			}

			/** Return the maximum capacity of the buffer.
			  * \sa size
			  */
			size_t capacity() const {
				return m_size;
			}

			/** The maximum number of elements that can be written ("push") without rising an overflow error.
			  */
			size_t available() const {
				return (capacity()-size())-1;
			}

			/** Delete all the stored data, if any. */
			void clear() {
				m_next_write = m_next_read = 0;
			}

		};  // end class circular_buffer

	} // End of namespace
} // End of namespace
#endif
