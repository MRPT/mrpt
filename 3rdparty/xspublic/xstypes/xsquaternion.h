
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

#ifndef XSQUATERNION_H
#define XSQUATERNION_H

#include "xsmath.h"

struct XsEuler;
struct XsMatrix;
struct XsVector;
struct XsQuaternion;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
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
XSTYPES_DLL_API int XsQuaternion_compare(XsQuaternion const* thisPtr, XsQuaternion const* other, XsReal tolerance);
XSTYPES_DLL_API XsReal XsQuaternion_dotProduct(XsQuaternion const* thisPtr, XsQuaternion const* other);

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
	inline explicit XsQuaternion(XsReal w_ = XsMath_zero, XsReal x_ = XsMath_zero, XsReal y_ = XsMath_zero, XsReal z_ = XsMath_zero)
		: m_w(w_)
		, m_x(x_)
		, m_y(y_)
		, m_z(z_)
	{
	}

	//! \brief Construct a quaternion with the supplied values, or zero's by default and optionally normalize
	inline explicit XsQuaternion(XsReal w_, XsReal x_, XsReal y_, XsReal z_, bool normalize_)
		: m_w(w_)
		, m_x(x_)
		, m_y(y_)
		, m_z(z_)
	{
		if (normalize_)
			normalize();
	}

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

	//! \brief Destructor, intentionally empty
	inline ~XsQuaternion()
	{
	}

	//! \brief Assigns the \a other quaternion to this quaternion
	inline XsQuaternion& operator=(const XsQuaternion& other)
	{
		if (this != &other)
		{
			m_w = other.m_w;
			m_x = other.m_x;
			m_y = other.m_y;
			m_z = other.m_z;
		}
		return *this;
	}

	/*! \brief Set the Quaternion to these specific values
	*/
	inline void assign(XsReal w_, XsReal x_, XsReal y_, XsReal z_)
	{
		m_w = w_;
		m_x = x_;
		m_y = y_;
		m_z = z_;
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
	inline XsReal operator[](XsSize index) const
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

	//! \brief Normalize the quaternion \sa XsQuaternion_normalized \returns The cartesian length of the quaternion before normalization
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

	//! \brief Returns true if the fields of this and \a other are within \a tolerance of each otehr
	inline bool isEqual(const XsQuaternion& other, XsReal tolerance) const
	{
		return !!XsQuaternion_compare(this, &other, tolerance);
	}

	//! \brief Returns the dot product of this with \a other
	inline XsReal dotProduct(const XsQuaternion& other) const
	{
		return XsQuaternion_dotProduct(this, &other);
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

#endif
