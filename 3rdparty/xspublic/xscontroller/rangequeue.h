
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

#ifndef RANGE_QUEUE_H
#define RANGE_QUEUE_H

#include <list>
#include <xscommon/xsens_mutex.h>

/*! \class RangeQueue
	\brief A range queue
*/
template <class T>
class RangeQueue
{
public:
	/*! \brief Default constructor
	*/
	RangeQueue() : m_count(0)
	{
	}

	/*! \brief Default destructor
	*/
	virtual ~RangeQueue()
	{
	}

	/*! \brief Clears the queue
	*/
	void clear()
	{
		xsens::Lock locky(&m_mutex);
		m_queue.clear();
		m_count = 0;
	}

	/*! \brief Adds a range to the back of the queue
		Only a range newer than the most recent range in the queue is added. Older (also partial) are ignored
		Also \a end must be larger or equal than \a start
		\param start : The first index of the range
		\param end: The last index of the range
	*/
	void pushBack(T start, T end)
	{
		if (end < start)
			return;

		xsens::Lock locky(&m_mutex);
		if (!empty() && (start <= last()))
			return;

		Range newRange(start, end);
		JLTRACEG("Adding range: [" << start << " - " << end << "]");
		m_queue.push_back(newRange);
		m_count += 1 + end - start;
	}

	/*! \brief Removes a specific index from the queue
		\param index The index to remove
		\details This function removes the given index from the queue by splitting the containing range at the \a index
	*/
	void remove(T index)
	{
		xsens::Lock locky(&m_mutex);
		if (empty())
			return;

		for (auto r = m_queue.rbegin(); r != m_queue.rend(); ++r)
		{
			if ( (index > r->m_end) || (index < r->m_start) )
				continue;

			--m_count;
			if (r->m_start == r->m_end)
			{
				m_queue.erase(--(r.base()));
			}
			else if (r->m_start == index)
			{
				r->m_start++;
			}
			else if (r->m_end == index)
			{
				r->m_end--;
			}
			else
			{
				Range split(r->m_start, index - 1);
				r->m_start = index + 1;
				m_queue.insert(--(r.base()), split);
			}
			return;
		}
	}

	/*! \brief Removes all the indices upto and including \a upto from the queue
	*/
	void popFront(T upto)
	{
		xsens::Lock locky(&m_mutex);

		while (!m_queue.empty() && (m_queue.front().m_end <= upto))
		{
			auto const& i = m_queue.front();
			m_count -= (i.m_end - i.m_start) + 1;
			m_queue.pop_front();
		}

		if (empty())
		{
			JLDEBUGG("Removed upto: " << upto << ", now empty");
			m_count = 0;
			return;
		}
		if (m_queue.front().m_start <= upto)
		{
			m_count -= 1 + upto - m_queue.front().m_start;
			m_queue.front().m_start = upto + 1;
		}
		JLDEBUGG("Removed upto: " << upto << ", new first = " << m_queue.front().m_start);
	}

	/*! \brief Removes all the indices upto and including \a upto from the queue
	*/
	void popBack(T upto)
	{
		xsens::Lock locky(&m_mutex);

		while (!m_queue.empty() && (m_queue.back().m_start >= upto))
		{
			auto const& i = m_queue.back();
			m_count -= (i.m_end - i.m_start) + 1;
			m_queue.pop_back();
		}

		if (empty())
		{
			JLDEBUGG("Removed upto: " << upto << ", now empty");
			m_count = 0;
			return;
		}
		if (m_queue.back().m_end >= upto)
		{
			m_count -= 1 + m_queue.back().m_end - upto;
			m_queue.back().m_end = upto - 1;
		}
		JLDEBUGG("Removed upto: " << upto << ", new last = " << m_queue.back().m_end);
	}

	/*! \brief Returns the first index of the first range in the queue
	*/
	T first() const
	{
		xsens::Lock locky(&m_mutex);
		if (empty())
			return illegalIndex();
		return m_queue.front().m_start;
	}

	/*! \brief Returns the last (highest) index in the queue
	*/
	T last() const
	{
		xsens::Lock locky(&m_mutex);
		if (empty())
			return illegalIndex();

		return m_queue.back().m_end;
	}

	/*!	\returns a range limit.
	 *	\param index: start or end limit to return.
	 *
	 *	Concatenating the start and end points for each range will generate a list of limits,
	 *	this function gives access to such a list.
	 *	For example:
	 *	- index 0: start of the first range
	 *	- index 1: end of the first range
	 *	- index 2: start of the second range
	 *	- ...
	 */
	const T operator[](std::size_t index) const
	{
		xsens::Lock locky(&m_mutex);

		std::size_t range = index / 2;
		uint32_t end = index % 2;

		if (m_queue.size() < range)
			return illegalIndex();

		auto it = m_queue.begin();

		for (uint32_t i = 0; i < range; ++i)
			++it;

		if (end)
			return it->m_end;
		else
			return it->m_start;
	}

	/*! \brief Checks if the given index is contained in the queue
	*/
	bool contains(uint32_t index) const
	{
		xsens::Lock locky(&m_mutex);
		for (auto i = m_queue.begin(); i != m_queue.end(); ++i)
		{
			if ( (index >= i->m_start) && (index <= i->m_end) )
				return true;
		}
		return false;
	}

	/*! \brief Returns the total number of indices captured by the ranges in the queue
	*/
	T count() const
	{
		xsens::Lock locky(&m_mutex);
		return m_count;
	}

	/*! \brief Returns the recounted total number of indices captured by the ranges in the queue*/
	T recount()
	{
		xsens::Lock locky(&m_mutex);
		m_count = 0;
		for (auto i = m_queue.begin(); i != m_queue.end(); ++i)
		{
			m_count += (i->m_end - i->m_start) + 1;
		}
		return m_count;
	}

	/*! \brief Returns true if the queue is empty
	*/
	bool empty() const
	{
		return m_queue.empty();
	}

	/*! \brief Returns the value for an illegal index
	*/
	static const T illegalIndex()
	{
		return (T)-1;
	}

	/*!
	 *	\brief Copy all ranges within the given limits [start, end].
	 *	If the limit falls in a certain range, that range is split and included
	 *	from the start until the limit.
	 */
	void copy(RangeQueue<T>& destination, T start, T end)
	{
		xsens::Lock locky(&m_mutex);

		for (auto i = m_queue.begin(); i != m_queue.end(); ++i)
		{
			if ((i->m_start >= start && start <= i->m_end) && (i->m_start <= end))
			{
				destination.pushBack(i->m_start, std::min(i->m_end, end));
			}
			else if (i->m_start < start && i->m_start < end)
			{
				destination.pushBack(start, std::min(i->m_end, end));
			}
		}
	}

private:
	struct Range
	{
		T m_start;
		T m_end;
		Range(T start, T end)
			: m_start(start)
			, m_end(end)
		{}
	};
	T m_count;

	std::list<Range> m_queue;
	mutable xsens::Mutex m_mutex;
};

#endif
