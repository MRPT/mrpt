/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSEULER_H
#define XSEULER_H

#include "xsmath.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSEULER_INITIALIZER { { { XsMath_zero,XsMath_zero,XsMath_zero } } }
#endif

struct XsEuler;
struct XsQuaternion;
struct XsMatrix;

XSTYPES_DLL_API void XsEuler_destruct(struct XsEuler* thisPtr);
XSTYPES_DLL_API int XsEuler_empty(const struct XsEuler* thisPtr);
XSTYPES_DLL_API void XsEuler_fromQuaternion(struct XsEuler* thisPtr, const struct XsQuaternion* quat);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsEuler {
#ifdef __cplusplus
	//! \brief Constructor that creates an Euler object with all angles 0
	inline XsEuler() : m_x(XsMath_zero), m_y(XsMath_zero), m_z(XsMath_zero) {}
	//! \brief Constructor that creates an Euler object the specified angles
	inline XsEuler(XsReal x_, XsReal y_, XsReal z_) : m_x(x_), m_y(y_), m_z(z_) {}
	//! \brief Constructor that creates an Euler object from \a other
	inline XsEuler(const XsEuler& other) : m_x(other.m_x), m_y(other.m_y), m_z(other.m_z) {}

	//! \brief \copybrief XsEuler_fromQuaternion
	inline explicit XsEuler(const XsQuaternion& q)
	{
		XsEuler_fromQuaternion(this, &q);
	}

	//! \brief Assigns the \a other XsEuler object to this one
	inline XsEuler& operator=(const XsEuler& other)
	{
		m_x = other.m_x;
		m_y = other.m_y;
		m_z = other.m_z;
		return *this;
	}

	//! \brief Returns the \a index'th euler angle in the object
	inline XsReal operator[](XsSize index) const
	{
		assert (index <= 2);
		return m_data[index];
	}

	//! \brief Returns a reference to the \a index'th euler angle in the object
	inline XsReal &operator[](XsSize index)
	{
		assert (index <= 2);
		return m_data[index];
	}

	//! \brief Returns true if all angles in this object are zero
	inline bool empty() const
	{
		return m_x == XsMath_zero && m_y == XsMath_zero && m_z == XsMath_zero;
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_data;
	}
	
	//! \brief \copybrief XsEuler_fromQuaternion
	inline XsEuler& fromQuaternion(const XsQuaternion& quat)
	{
		XsEuler_fromQuaternion(this, &quat);
		return *this;
	}

	/*! \brief Returns true if the values in \a other are exactly equal to this
	*/
	inline bool operator == (const XsEuler& other) const
	{
		return m_roll == other.m_roll && m_pitch == other.m_pitch && m_yaw == other.m_yaw;
	}

	/*! \brief Returns true if the values in \a other are different from this
	*/
	inline bool operator != (const XsEuler& other) const
	{
		return m_roll != other.m_roll || m_pitch != other.m_pitch || m_yaw != other.m_yaw;
	}

	//! \brief Returns the roll or x value
	inline XsReal roll() const { return m_roll; }
	//! \brief Returns the pitch or y value
	inline XsReal pitch() const { return m_pitch; }
	//! \brief Returns the yaw or z value
	inline XsReal yaw() const { return m_yaw; }

	//! \brief Returns the x or roll value
	inline XsReal x() const { return m_x; }
	//! \brief Returns the y or pitch value
	inline XsReal y() const { return m_y; }
	//! \brief Returns the z or yaw value
	inline XsReal z() const { return m_z; }

private:
#endif

	union {
		struct {
			XsReal m_x;		//!< Stores the x component of the euler triplet
			XsReal m_y;		//!< Stores the y component of the euler triplet
			XsReal m_z;		//!< Stores the z component of the euler triplet
		};
		struct {
			XsReal m_roll;		//!< Stores the roll component of the euler triplet
			XsReal m_pitch;		//!< Stores the pitch component of the euler triplet
			XsReal m_yaw;		//!< Stores the yaw component of the euler triplet
		};
		XsReal m_data[3];	//!< Stores the euler triplet in an array of three elements
	};
};

typedef struct XsEuler XsEuler;

#endif // file guard
