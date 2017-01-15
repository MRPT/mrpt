/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSUSHORTVECTOR_H
#define XSUSHORTVECTOR_H

#ifndef __cplusplus
#define XSUSHORTVECTOR_INITIALIZER {{0,0,0}}
#else
#include "xstypedefs.h"
#endif

/*! \brief A vector containing 3 short values. */
struct XsUShortVector {
#ifdef __cplusplus
	//! \brief Constructor that creates the vector with all components set to 0
	inline XsUShortVector()
	{
		m_data[0] = 0;
		m_data[1] = 0;
		m_data[2] = 0;
	}

	//! \brief Constructor that creates the vector with all components set to given values \a v1 \a v2 and \a v3
	inline XsUShortVector(unsigned short v1, unsigned short v2, unsigned short v3)
	{
		m_data[0] = v1;
		m_data[1] = v2;
		m_data[2] = v3;
	}
	
	//! \brief Constructor that creates the vector with all components set to values in array \a a
	inline explicit XsUShortVector(const unsigned short* a)
	{
		m_data[0] = a[0];
		m_data[1] = a[1];
		m_data[2] = a[2];
	}

	//! \brief Constructor that creates the vector and initializes it with data from the \a other vector
	inline XsUShortVector(const XsUShortVector& other)
	{
		m_data[0] = other.m_data[0];
		m_data[1] = other.m_data[1];
		m_data[2] = other.m_data[2];
	}
	
	//! \brief Assignment operator copies the data from the \a other vector to this vector
	inline const XsUShortVector& operator = (const XsUShortVector& other)
	{
		if (this != &other)
		{
			m_data[0] = other.m_data[0];
			m_data[1] = other.m_data[1];
			m_data[2] = other.m_data[2];
		}
		return *this;
	}
	
	//! \brief Comparison operator, returns true if the contents of the \a other vector match those of this vector
	inline bool operator == (const XsUShortVector& other) const
	{
		return	m_data[0] == other.m_data[0] &&
				m_data[1] == other.m_data[1] &&
				m_data[2] == other.m_data[2];
	}

	//! \brief Return the size of the vector (always 3)
	inline XsSize size() const
	{
		return 3;
	}

	//! \brief Return a value from the vector (needed to allow generated C# access to these elements)
	inline unsigned short at(int index)
	{
		return m_data[index];
	}

	//! \brief Returns the \a index'th item in the vector
	inline unsigned short operator[](XsSize index) const
	{
		assert(index < 3);
		return m_data[index];
	}

	//! \brief Returns a reference the \a index'th item in the vector
	inline unsigned short& operator[](XsSize index)
	{
		assert(index < 3);
		return m_data[index];	//lint !e1536
	}
private:
#endif

	unsigned short m_data[3];	//!< vector component storage
};

typedef struct XsUShortVector XsUShortVector;

#endif	// XSUSHORTVECTOR_H
