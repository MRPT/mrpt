/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  circular_buffer_H
#define  circular_buffer_H

#include <vector>
#include <stdexcept>

namespace mrpt
{
	namespace utils
	{
		/** A circular buffer of fixed size (defined at construction-time), implemented with a std::vector as the underlying storage.
		 * \ingroup stlext_grp 
		  * \note Defined in #include <mrpt/utils/circular_buffer.h>
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
			  * \exception std::out_of_range If the buffer has less elements than requested. */
			void pop_many(T *out_array, size_t count) {
				while (count--)
					pop(*out_array++);
			}

			/** Peek (see without modifying) what is to be read from the buffer if pop() was to be called.
			  * \exception std::out_of_range If the buffer is empty. */
			T peek() const {
				if (m_next_read==m_next_write) throw std::out_of_range("peek: circular_buffer is empty");
				return m_data[m_next_read];
			}
			/** Like peek(), but seeking ahead in the buffer (index=0 means the immediate next element, index=1 the following one, etc.)
			  * \exception std::out_of_range If trying to read passing the number of available elements. */
			T peek(size_t index) const {
				if (index>=this->size()) throw std::out_of_range("peek: seek out of range");
				return m_data[(m_next_read + index)%m_size];
			}

			/** Like peek(), for multiple elements, storing a number of elements into a user-provided array.
			  * \exception std::out_of_range If the buffer has less elements than requested. */
			void peek_many(T *out_array, size_t count) const {
				size_t peek_read = m_next_read;
				while (count--)
				{
					if (peek_read==m_next_write) throw std::out_of_range("peek: circular_buffer is empty");
					T val =m_data[peek_read++];
					if (peek_read==m_size) peek_read=0;
					*out_array++ = val;
				}
			}

			/** Return the number of elements available for read ("pop") in the buffer (this is NOT the maximum size of the internal buffer)
			  * \sa capacity */
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
