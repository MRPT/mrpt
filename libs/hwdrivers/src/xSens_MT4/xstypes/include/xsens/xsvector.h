/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSVECTOR_H
#define XSVECTOR_H

#include "xsmath.h"
#include <stddef.h>
#include <string.h>	// memcpy

struct XsVector;
struct XsQuaternion;

#ifdef __cplusplus
#include <vector>
#include <algorithm>
extern "C" {
#else
#define XSVECTOR_INITIALIZER	{ NULL, 0, 0 }
typedef struct XsVector XsVector;
#endif

XSTYPES_DLL_API void XsVector_ref(XsVector* thisPtr, XsSize sz, XsReal* buffer, XsDataFlags flags);
XSTYPES_DLL_API void XsVector_construct(XsVector* thisPtr, XsSize sz, const XsReal* src);
XSTYPES_DLL_API void XsVector_assign(XsVector* thisPtr, XsSize sz, const XsReal* src);
XSTYPES_DLL_API void XsVector_destruct(XsVector* thisPtr);
XSTYPES_DLL_API void XsVector_copy(XsVector* copy, XsVector const* src);
XSTYPES_DLL_API XsReal XsVector_dotProduct(const XsVector* a, const XsVector* b);
XSTYPES_DLL_API XsReal XsVector_cartesianLength(const XsVector* thisPtr);
XSTYPES_DLL_API void XsVector_setZero(XsVector* thisPtr);
XSTYPES_DLL_API int XsVector_empty(const XsVector* thisPtr);
XSTYPES_DLL_API void XsVector_multiplyScalar(const XsVector* thisPtr, XsReal scalar, XsVector* dest);
XSTYPES_DLL_API void XsVector_angularVelocityFromQuaternion(XsVector* thisPtr, XsReal deltaT, const struct XsQuaternion* quat);
XSTYPES_DLL_API void XsVector_swap(XsVector* a, XsVector* b);
XSTYPES_DLL_API void XsVector_fill(XsVector* thisPtr, XsReal value);
XSTYPES_DLL_API int XsVector_equal(const XsVector* thisPtr, const XsVector* thatPtr);

#ifdef __cplusplus
} // extern "C"
#endif
#ifndef XSENS_NO_PACK
#pragma pack(push,1)
#endif
struct XsVector {
XSCPPPROTECTED
	XsReal* const m_data;		//!< \protected Points to contained data buffer
	const XsSize m_size;		//!< \protected Size of contained data buffer in elements
	const int m_flags;			//!< \protected Flags for data management

#ifdef __cplusplus
	//! \brief Return the data management flags of the vector.
	inline int flags() { return m_flags; }
public:
	//! \brief Initialize a vector, empty or using the data in the supplied \a sz and \a src
	inline explicit XsVector(XsSize sz = 0, const XsReal* src = 0)
		: m_data(0)
		, m_size(0)
		, m_flags(0)
	{
		if (sz)
			XsVector_construct(this, sz, src);
	}

	//! \brief Initialize a vector using the supplied \a other vector
	inline XsVector(const XsVector& other)
		: m_data(0)
		, m_size(0)
		, m_flags(0)
	{
		*this = other;
	}

	//! \brief Initialize a vector that references the supplied data
	inline explicit XsVector(XsReal* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_size(sz)
		, m_flags(flags)
	{
	}

	//! \brief Initialize a vector that references the supplied data
	inline explicit XsVector(const XsVector& other, XsReal* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_size(sz)
		, m_flags(flags)
	{
		XsVector_copy(this, &other);
	}

	//! \copydoc XsVector_angularVelocityFromQuaternion
	inline explicit XsVector(const XsQuaternion& quat, XsReal deltaT)
		: m_data(0)
		, m_size(0)
		, m_flags(0)
	{
		XsVector_angularVelocityFromQuaternion(this, deltaT, &quat);
	}

	//! \brief Assignment operator. Copies from \a other into this
	inline XsVector& operator=(const XsVector& other)
	{
		XsVector_copy(this, &other);
		return *this;
	}

	//! \copydoc XsVector_destruct
	inline ~XsVector()
	{
		XsVector_destruct(this);
	}

	//! \copydoc XsVector_assign
	inline void assign(XsSize sz, const XsReal* src)
	{
		XsVector_assign(this, sz, src);
	}

	/*! \brief Sets the size of the XsVector to \a sz items
		\param sz The desired size of the vector
		\sa XsVector_assign
	*/
	inline void setSize(XsSize sz)
	{
		XsVector_assign(this, sz, 0);
	}

	//! \brief Returns the number of elements in the vector
	inline XsSize size() const
	{
		return m_size;
	}

	//! \brief Return a const pointer to the data
	inline const XsReal* data() const
	{
		return m_data;
	}

	//! \brief Multiply the vector by \a scalar and return the result
	inline XsVector operator * (XsReal scalar) const
	{
		XsVector v(m_size);
		for (XsSize i = 0; i < m_size; ++i)
			v.m_data[i] = m_data[i] * scalar;
		return v;
	}

	//! \brief Multiply the vector by \a scalar and store the result in this vector
	inline void operator *=(XsReal scalar)
	{
		for (XsSize i = 0; i < m_size; ++i)
			m_data[i] *= scalar;
	}

	//! \brief Returns a reference to the \a index'th item in the vector
	inline XsReal& at(XsSize index)
	{
		assert(index < m_size);
		return m_data[index];
	}

	//! \brief Returns a const reference to the \a index'th item in the vector
	inline const XsReal& at(XsSize index) const
	{
		assert(index < m_size);
		return m_data[index];
	}

	//! \brief Returns the \a index'th item in the vector
	inline XsReal value(XsSize index) const
	{
		assert(index < m_size);
		return m_data[index];
	}

	//! \brief Sets the \a index'th item in the vector
	inline void setValue(XsSize index, XsReal val)
	{
		assert(index < m_size);
		m_data[index] = val;
	}

	//! \brief Returns the \a index'th item in the vector
	inline XsReal operator[](XsSize index) const
	{
		assert(index < m_size);
		return m_data[index];
	}

	//! \brief Returns a reference the \a index'th item in the vector
	inline XsReal& operator[](XsSize index)
	{
		assert(index < m_size);
		return m_data[index];
	}

	//! \brief \copybrief XsVector_dotProduct
	inline XsReal dotProduct(const XsVector &v) const
	{
		return XsVector_dotProduct(this, &v);
	}

	//! \copydoc XsVector_cartesianLength
	inline XsReal cartesianLength() const
	{
		return XsVector_cartesianLength(this);
	}

	//! \brief \copybrief XsVector_setZero
	inline void setZero()
	{
		return XsVector_setZero(this);
	}

	//! \brief \copybrief XsVector_empty
	inline bool empty() const
	{
		return 0 != XsVector_empty(this);
	}

	//! \copydoc XsVector_angularVelocityFromQuaternion
	inline XsVector& angularVelocityFromQuaternion(const XsQuaternion& quat, XsReal deltaT)
	{
		XsVector_angularVelocityFromQuaternion(this, deltaT, &quat);
		return *this;
	}

	//! \brief Return \e this - \a sub
	XsVector operator-(const XsVector& sub) const
	{
		assert(m_size == sub.m_size);
		XsVector tmp(m_size);
		for (XsSize i = 0; i < m_size; ++i)
			tmp[i] = m_data[i] - sub.m_data[i];
		return tmp;
	}

	//! \brief Return \e this + \a sub
	XsVector operator+(const XsVector& sub) const
	{
		assert(m_size == sub.m_size);
		XsVector tmp(m_size);
		for (XsSize i = 0; i < m_size; ++i)
			tmp[i] = m_data[i] + sub.m_data[i];
		return tmp;
	}

	//! \brief Return true when the values in this vector are exactly (to the last bit) equal to \a other
	bool operator==(const XsVector& other) const
	{
		return 0 != XsVector_equal(this, &other);
	}

#ifndef XSENS_NO_STL
	//! \brief Returns the XsVector as a std::vector of XsReal
	inline std::vector<XsReal> toVector() const
	{
		std::vector<XsReal> tmp(m_size);		
		if (m_size)
			memcpy(&tmp[0], m_data, m_size * sizeof(XsReal));
		return tmp;
	}
#endif

	/*! \brief Fill the vector with zeroes */
	inline void zero()
	{
		for (XsSize i = 0; i < m_size; ++i)
			m_data[i] = XsMath_zero;
	}

	/*! \brief Fill the vector with \a value */
	inline void fill(XsReal value)
	{
		for (XsSize i = 0; i < m_size; ++i)
			m_data[i] = value;
	}

	/*! \brief Swap the contents of \a b with this
		\details This function swaps the internal buffers so no actual data is moved around. For unmanaged
		data an elementwise swap is done, but only if the vectors are the same size.
		\param b Object whose contents will be swapped with this
	*/
	inline void swap(XsVector& b)
	{
		XsVector_swap(this, &b);
	}

#endif
};
#ifndef XSENS_NO_PACK
#pragma pack(pop)
#endif

#ifdef __cplusplus
//! \brief Multiplies all values in the vector \a v by \a scalar
inline XsVector operator *(XsReal scalar, const XsVector &v)
{
	return v*scalar;
}
#endif

#endif // file guard
