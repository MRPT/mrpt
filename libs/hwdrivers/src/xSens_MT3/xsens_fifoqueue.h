/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _FIFOQUEUE_H_2006_05_03
#define _FIFOQUEUE_H_2006_05_03

//#define _LOG_QUEUE
#if (defined(_DEBUG) || defined(_LOG_ALWAYS)) && defined(_LOG_QUEUE)
#	include <typeinfo.h>
#else
#	define QUEUELOG(...)
#endif

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A FIFO queue with limited length (cyclic).

	The class is based on the STL queue class, but has a limited size. If more items are
	inserted than would fit, the oldest item is overwritten. The class can only handle 
	pointer types.
*/
template <class T, bool E=true>
class FifoQueue {
private:
#if !defined(QUEUELOG)
// write to screen/debug-stream
void QUEUELOG(const char *str, ...)
{
	#if defined(_LOG_TO_STDOUT)
		va_list ptr;
		va_start(ptr,str);
		vprintf(str,ptr);
	#else
	//#ifdef _LOG_TO_DBVIEW
		char buf[2048];

		va_list ptr;
		va_start(ptr,str);
		vsprintf(buf,str,ptr);

		OutputDebugString(buf);
	#endif
}
#endif
protected:
	size_t m_maxCount;
	size_t m_currentCount;
	size_t m_first;
	bool m_deleteOnOverwrite;

	T*	m_list;
public:
	typedef T		value_type;		//!< The type of the value stored in this queue.
	typedef size_t	size_type;		//!< The type of a 'size' value.

	//! Create an empty queue with capacity size.
	FifoQueue(size_type size=16, bool delOnOverwrite = true)
	{
		QUEUELOG("[%p]FifoQueue.new (base), T=%s, size=%d, del=%d\n",this,typeid(T).name(),size,delOnOverwrite);

		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
		m_deleteOnOverwrite = delOnOverwrite;
	}
	
	//! The copy constructor.
	template <bool E2>
	FifoQueue(const FifoQueue<T,E2>& q)
	{
		QUEUELOG("[%p]FifoQueue.new (copy), T=%s, size=%d, count=%d, del=%d\n",this,typeid(T).name(),q.m_maxCount,q.m_currentCount,q.m_deleteOnOverwrite);
		m_maxCount = q.m_maxCount;
		m_list = new T[m_maxCount];
		m_currentCount = q.m_currentCount;
		m_deleteOnOverwrite = q.m_deleteOnOverwrite;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];
	}
	
	void eraseAndClear(void)
	{
		QUEUELOG("[%p]FifoQueue.eraseAndClear, T=%s, count=%d\n",this,typeid(T).name(),m_currentCount);
		for (size_t i = 0;i<m_currentCount;++i)
			delete m_list[(i+m_first) % m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! The destructor.
	~FifoQueue()
	{
		QUEUELOG("[%p]FifoQueue.delete, T=%s, count=%d\n",this,typeid(T).name(),m_currentCount);
		if (E)
			eraseAndClear();
		m_maxCount = 0;
		delete[] m_list;
	}

	//! The assignment operator.
	template <bool E2>
	FifoQueue<T,E>& operator=(const FifoQueue<T,E2>& q)
	{
		QUEUELOG("[%p]FifoQueue.operator =, T=%s, max=%d, count=%d, smax=%d, scount=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,q.m_maxCount,q.m_currentCount);
		if (m_maxCount != q.m_maxCount)
		{
			delete[] m_list;
			m_maxCount = q.m_maxCount;
			m_list = new T[m_maxCount];
		}
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];

		return *this;
	}
	
	//! Resize the queue, note that this function clears the queue.
	void resize(const size_t size)
	{
		QUEUELOG("[%p]FifoQueue.resize, T=%s, max=%d, count=%d, size=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,size);
		if (E)
			eraseAndClear();
		delete[] m_list;
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! Return true if the queue is empty.
	bool empty() const
	{
		return (m_currentCount == 0);
	}
	
	//! Return the maximum number of elements in the queue.
	size_type size() const
	{
		return m_maxCount;
	}
	
	//! Return the number of elements currnetly in the queue.
	size_type length() const
	{
		return m_currentCount;
	}

	//! Return the oldest element in the queue.
	value_type& front()
	{
		return m_list[m_first];
	}
	
	//! Return the oldest element in the queue.
	const value_type& front() const
	{
		return m_list[m_first];
	}
	
	//! Return the newest element in the queue.
	value_type& back()
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Return the newest element in the queue.
	const value_type& back() const
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Insert x at the back of the queue.
	void push(const value_type& x)
	{
		QUEUELOG("[%p]FifoQueue.push, T=%s, max=%d, count=%d, x=%p\n",this,typeid(T).name(),m_maxCount,m_currentCount,x);
		if (m_currentCount == m_maxCount)
		{
			if (m_deleteOnOverwrite)
				delete (m_list[m_first]);
			
			m_list[m_first] = x;
			m_first = (m_first+1) % m_maxCount;
		}
		else
		{
			m_list[(m_first + m_currentCount++) % m_maxCount] = x;
		}
	}
	
	//! Remove the element at the front of the queue.
	void pop(void)
	{
		QUEUELOG("[%p]FifoQueue.pop, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		m_first = (m_first+1) % m_maxCount;
		--m_currentCount;
	}

	//! Remove the element at the back of the queue.
	void popBack(void)
	{
		QUEUELOG("[%p]FifoQueue.popBack, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		--m_currentCount;
	}

	//! Return the index'th oldest item from the queue
	const value_type& operator[] (size_t index) const
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	//! Return the index'th oldest item from the queue
	value_type& operator[] (size_t index)
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	void clear(void)
	{
		QUEUELOG("[%p]FifoQueue.clear, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		m_currentCount = 0;
		m_first = 0;
	}

	void remove(size_t index)
	{
		QUEUELOG("[%p]FifoQueue.remove, T=%s, max=%d, count=%d, index=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,index);
		if (index >= m_currentCount)
			return;
		if (index == 0)
			pop();
		else
		{
			--m_currentCount;
			for (size_t i=index;i<m_currentCount;++i)
				m_list[(m_first + i) % m_maxCount] = m_list[(1 + m_first + i) % m_maxCount];
		}
	}
};


//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief A FIFO queue with limited length (cyclic).

	The class is based on the STL queue class, but has a limited size. If more items are
	inserted than would fit, the oldest item is overwritten. The class can only handle 
	non-pointer types.
*/
template <class T>
class FifoQueueBasic {
private:
#if !defined(QUEUELOG)
// write to screen/debug-stream
void QUEUELOG(const char *str, ...)
{
	#if defined(_LOG_TO_STDOUT)
		va_list ptr;
		va_start(ptr,str);
		vprintf(str,ptr);
	#else
	//#ifdef _LOG_TO_DBVIEW
		char buf[2048];

		va_list ptr;
		va_start(ptr,str);
		vsprintf(buf,str,ptr);

		OutputDebugString(buf);
	#endif
}
#endif
protected:
	size_t m_maxCount;
	size_t m_currentCount;
	size_t m_first;

	T*	m_list;
public:
	typedef T		value_type;		//!< The type of the value stored in this queue.
	typedef size_t	size_type;		//!< The type of a 'size' value.

	//! Create an empty queue with capacity size.
	FifoQueueBasic(size_type size=16)
	{
		QUEUELOG("[%p]FifoQueueBase.new (base), T=%s, size=%d\n",this,typeid(T).name(),size);
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}
	
	//! The copy constructor.
	FifoQueueBasic(const FifoQueueBasic<T>& q)
	{
		QUEUELOG("[%p]FifoQueueBase.new (copy), T=%s, size=%d\n",this,typeid(T).name(),q.m_maxCount);
		m_maxCount = q.m_maxCount;
		m_list = new T[m_maxCount];
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];
	}
	
	void eraseAndClear(void)
	{
		QUEUELOG("[%p]FifoQueueBase.eraseAndClear, T=%s, count=%d\n",this,typeid(T).name(),m_currentCount);
		for (size_t i = 0;i<m_currentCount;++i)
			delete m_list[(i+m_first) % m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! The destructor.
	~FifoQueueBasic()
	{
		QUEUELOG("[%p]FifoQueueBase.delete, T=%s, count=%d\n",this,typeid(T).name(),m_currentCount);
		m_maxCount = 0;
		delete[] m_list;
	}

	//! The assignment operator.
	FifoQueueBasic<T>& operator=(const FifoQueueBasic<T>& q)
	{
		QUEUELOG("[%p]FifoQueueBase.operator =, T=%s, max=%d, count=%d, smax=%d, scount=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,q.m_maxCount,q.m_currentCount);
		if (m_maxCount != q.m_maxCount)
		{
			delete[] m_list;
			m_maxCount = q.m_maxCount;
			m_list = new T[m_maxCount];
		}
		m_currentCount = q.m_currentCount;
		m_first = 0;
		for (size_t i = 0;i<m_currentCount;++i)
			m_list[i] = q.m_list[(i+q.m_first) % m_maxCount];

		return *this;
	}
	
	//! Resize the queue, note that this function clears the queue.
	void resize(const size_t size)
	{
		QUEUELOG("[%p]FifoQueueBase.resize, T=%s, max=%d, count=%d, size=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,size);
		delete[] m_list;
		if (size > 0)
			m_maxCount = size;
		else
			m_maxCount = 1;
		m_list = new T[m_maxCount];
		m_currentCount = 0;
		m_first = 0;
	}

	//! Return true if the queue is empty.
	bool empty() const
	{
		return (m_currentCount == 0);
	}
	
	//! Return the maximum number of elements in the queue.
	size_type size() const
	{
		return m_maxCount;
	}
	
	//! Return the number of elements currnetly in the queue.
	size_type length() const
	{
		return m_currentCount;
	}

	//! Return the oldest element in the queue.
	value_type& front()
	{
		return m_list[m_first];
	}
	
	//! Return the oldest element in the queue.
	const value_type& front() const
	{
		return m_list[m_first];
	}
	
	//! Return the newest element in the queue.
	value_type& back()
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Return the newest element in the queue.
	const value_type& back() const
	{
		return m_list[(m_first + m_currentCount - 1) % m_maxCount];
	}
	
	//! Insert x at the back of the queue.
	void push(const value_type& x)
	{
		QUEUELOG("[%p]FifoQueueBase.push, T=%s, max=%d, count=%d, x=%p\n",this,typeid(T).name(),m_maxCount,m_currentCount,x);
		if (m_currentCount == m_maxCount)
		{
			m_list[m_first] = x;
			m_first = (m_first+1) % m_maxCount;
		}
		else
		{
			m_list[(m_first + m_currentCount++) % m_maxCount] = x;
		}
	}

	//! Insert x at the front of the queue (LIFO operation).
	void push_front(const value_type& x)
	{
		QUEUELOG("[%p]FifoQueueBase.push_front, T=%s, max=%d, count=%d, x=%p\n",this,typeid(T).name(),m_maxCount,m_currentCount,x);
		m_first = (m_first+m_maxCount-1)%m_maxCount;
		if (m_currentCount == 0)
			m_first = 0;
		m_list[m_first] = x;
		if (m_currentCount < m_maxCount)
			++m_currentCount;
	}

	//! Remove the element at the front of the queue.
	void pop(void)
	{
		QUEUELOG("[%p]FifoQueueBase.pop, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		if (m_currentCount > 0)
		{
			m_first = (m_first+1) % m_maxCount;
			--m_currentCount;
		}
	}

	//! Remove the element at the back of the queue.
	void popBack(void)
	{
		QUEUELOG("[%p]FifoQueueBase.popBack, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		if (m_currentCount > 0)
			--m_currentCount;
	}

	//! Return the index'th oldest item from the queue
	const value_type& operator[] (size_t index) const
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	//! Return the index'th oldest item from the queue
	value_type& operator[] (size_t index)
	{
		if (index >= m_currentCount)
			return m_list[(m_first + m_currentCount - 1) % m_maxCount];
		else
			return m_list[(m_first + index) % m_maxCount];
	}

	void clear(void)
	{
		QUEUELOG("[%p]FifoQueueBase.clear, T=%s, max=%d, count=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount);
		m_currentCount = 0;
		m_first = 0;
	}

	void remove(size_t index)
	{
		QUEUELOG("[%p]FifoQueueBase.remove, T=%s, max=%d, count=%d, index=%d\n",this,typeid(T).name(),m_maxCount,m_currentCount,index);
		if (index >= m_currentCount)
			return;
		if (index == 0)
			pop();
		else
		{
			--m_currentCount;
			for (size_t i=index;i<m_currentCount;++i)
				m_list[(m_first + i) % m_maxCount] = m_list[(1 + m_first + i) % m_maxCount];
		}
	}
};

} // end of xsens namespace

#endif	// _FIFOQUEUE_H_2006_05_03
