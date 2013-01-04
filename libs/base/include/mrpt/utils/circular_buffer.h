/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
