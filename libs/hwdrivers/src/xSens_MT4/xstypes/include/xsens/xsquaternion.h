/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSQUATERNION_H
#define XSQUATERNION_H

#include "xsmath.h"

struct XsEuler;
struct XsMatrix;
struct XsVector;
struct XsQuaternion;

#ifdef __cplusplus
extern "C" {
#else
#define XSQUATERNION_INITIALIZER { { { XsMath_zero, XsMath_zero, XsMath_zero, XsMath_zero } } }
typedef struct XsQuaternion XsQuaternion;
#endif

XSTYPES_DLL_API void XsQuaternion_destruct(XsQuaternion* thisPtr);
XSTYPES_DLL_API int XsQuaternion_empty(const XsQuaternion* thisPtr);
XSTYPES_DLL_API void XsQuaternion_inverse(const XsQuaternion* thisPtr, XsQuaternion* dest);
XSTYPES_DLL_API XsReal XsQuaternion_normalized(const XsQuaternion* thisPtr, XsQuaternion* dest);
XSTYPES_DLL_API XsReal XsQuaternion_normalize(XsQuaternion* thisPtr);
XSTYPES_DLL_API void XsQuaternion_fromEulerAngles(XsQuaternion* thisPtr, const struct XsEuler* src);
XSTYPES_DLL_API void XsQuaternion_fromRotationMatrix(XsQuaternion* thisPtr, const struct XsMatrix* ori);
XSTYPES_DLL_API const XsQuaternion* XsQuaternion_identity(void);
XSTYPES_DLL_API void XsQuaternion_multiply(const XsQuaternion* left, const XsQuaternion* right, XsQuaternion* dest);
XSTYPES_DLL_API void XsQuaternion_swap(XsQuaternion* a, XsQuaternion* b);
XSTYPES_DLL_API void XsQuaternion_copy(XsQuaternion* copy, XsQuaternion const* src);
XSTYPES_DLL_API int XsQuaternion_equal(XsQuaternion const* a, XsQuaternion const* b);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsQuaternion {
XSCPPPROTECTED
	union {
		struct {
			XsReal m_w;		//!< Stores the w component of the quaternion
			XsReal m_x;		//!< Stores the x component of the quaternion
			XsReal m_y;		//!< Stores the y component of the quaternion
			XsReal m_z;		//!< Stores the z component of the quaternion
		};
		XsReal m_data[4];	//!< Stores the quaternion in an array of four elements
	};
#ifdef __cplusplus
public:
	//! \brief Construct a quaternion with the supplied values, or zero's by default
	inline explicit XsQuaternion(XsReal w = XsMath_zero, XsReal x = XsMath_zero, XsReal y = XsMath_zero, XsReal z = XsMath_zero)
		: m_w(w)
		, m_x(x)
		, m_y(y)
		, m_z(z)
	{}

	//! \brief Construct a quaternion with the supplied values from the \a other Quaternion
	inline XsQuaternion(const XsQuaternion& other)
		: m_w(other.m_w)
		, m_x(other.m_x)
		, m_y(other.m_y)
		, m_z(other.m_z)
	{}

	//! \brief Construct a quaternion by converting from an XsEuler object
	inline explicit XsQuaternion(const XsEuler& euler)
	{
		XsQuaternion_fromEulerAngles(this, &euler);
	}

	//! \brief Construct a quaternion by converting from an XsMatrix rotation matrix object
	inline explicit XsQuaternion(const XsMatrix& ori)
	{
		XsQuaternion_fromRotationMatrix(this, &ori);
	}

	//! \brief Assigns the \a other quaternion to this quaternion
	inline XsQuaternion& operator=(const XsQuaternion& other)
	{
		m_w = other.m_w;
		m_x = other.m_x;
		m_y = other.m_y;
		m_z = other.m_z;
		return *this;
	}

	/*! \brief Set the Quaternion to these specific values
	*/
	inline void assign(XsReal w, XsReal x, XsReal y, XsReal z)
	{
		m_w = w;
		m_x = x;
		m_y = y;
		m_z = z;
	}

	/*! \brief Set the Quaternion to the specific values in the supplied array.
		\param values An array that contains at least 4 items. The first four will be interpreted as w,x,y,z
	*/
	inline void assign(const XsReal* values)
	{
		for (int i = 0; i < 4; ++i)
			m_data[i] = values[i];
	}

	//! \brief Returns a reference to the \a index'th component of the quaternion
	inline XsReal& operator[](XsSize index)
	{
		assert(index < 4);
		return m_data[index];
	}

	//! \brief Returns the \a index'th component of the quaternion
	inline XsReal const& operator[](XsSize index) const
	{
		assert(index < 4);
		return m_data[index];
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_data;
	}

	//! \brief \copybrief XsQuaternion_inverse \returns The inverse/conjugate of the quaternion
	inline XsQuaternion inverse() const
	{
		XsQuaternion tmp;
		XsQuaternion_inverse(this, &tmp);
		return tmp;
	}

	//! \brief \copybrief XsQuaternion_inverse \returns The inverse/conjugate of the quaternion
	inline XsQuaternion conjugate() const
	{
		XsQuaternion tmp;
		XsQuaternion_inverse(this, &tmp);
		return tmp;
	}

	//! \brief Return a normalized version of the quaternion \sa XsQuaternion_normalized \returns The normalized quaternion
	XsQuaternion normalized() const
	{
		XsQuaternion tmp;
		XsQuaternion_normalized(this, &tmp);
		return tmp;
	}

	//! \brief Normalized the quaternion \sa XsQuaternion_normalized \returns The cartesian length of the quaternion before normalization
	inline XsReal normalize()
	{
		return XsQuaternion_normalized(this, this);
	}

	//! \brief \copybrief XsQuaternion_empty
	inline bool empty() const
	{
		return 0 != XsQuaternion_empty(this);
	}

	//! \brief \copybrief XsQuaternion_fromEulerAngles
	inline XsQuaternion& fromEulerAngles(const XsEuler& src)
	{
		XsQuaternion_fromEulerAngles(this, &src);
		return *this;
	}

	//! \brief \copybrief XsQuaternion_fromRotationMatrix
	inline XsQuaternion& fromRotationMatrix(const XsMatrix& ori)
	{
		XsQuaternion_fromRotationMatrix(this, &ori);
		return *this;
	}

	//! \brief \copybrief XsQuaternion_identity
	inline static const XsQuaternion& identity()
	{
		return *XsQuaternion_identity();
	}

	/*! \brief In-place multiplication of this quaternion with \a other quaternion
		\param other The other quaternion to multiply with (right side of multiplication)
		\sa XsQuaternion_multiply()
	*/
	inline void operator*=(const XsQuaternion& other)
	{
		XsQuaternion_multiply(this, &other, this);
	}

	//! \brief Return the w component of the quaternion
	inline XsReal w() const { return m_w; }
	//! \brief Return the x component of the quaternion
	inline XsReal x() const { return m_x; }
	//! \brief Return the y component of the quaternion
	inline XsReal y() const { return m_y; }
	//! \brief Return the z component of the quaternion
	inline XsReal z() const { return m_z; }
	//! \brief Return a reference to the w component of the quaternion
	inline XsReal& w() { return m_w; }
	//! \brief Return a reference to the x component of the quaternion
	inline XsReal& x() { return m_x; }
	//! \brief Return a reference to the y component of the quaternion
	inline XsReal& y() { return m_y; }
	//! \brief Return a reference to the z component of the quaternion
	inline XsReal& z() { return m_z; }

	//! \brief Swap the contents with \a other
	inline void swap(XsQuaternion& other)
	{
		XsQuaternion_swap(this, &other);
	}

	//! \brief Returns true if \a other is numerically identical to this
	inline bool operator ==(const XsQuaternion& other) const
	{
		return	m_w == other.m_w &&
				m_x == other.m_x &&
				m_y == other.m_y &&
				m_z == other.m_z;
	}

#endif
};

#ifdef __cplusplus
//! \brief Return the negated version of the Quaternion \a q (w,-x,-y,-z)
inline XsQuaternion operator-(const XsQuaternion &q)
{
	return XsQuaternion(q.w(), -q.x(), -q.y(), -q.z());
}

//! \brief Multiply \a lhs by \a rhs and return the result
inline XsQuaternion operator *(const XsQuaternion& lhs, const XsQuaternion &rhs)
{
	XsQuaternion tmp(lhs);
	tmp *= rhs;
	return tmp;
}
#endif

#endif // file guard
