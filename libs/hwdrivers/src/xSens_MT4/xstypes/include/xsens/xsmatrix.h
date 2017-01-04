/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSMATRIX_H
#define XSMATRIX_H

#include "xsmath.h"
#include <stddef.h>

struct XsMatrix;
struct XsEuler;
struct XsQuaternion;

#ifdef __cplusplus
extern "C" {
#else
#define XSMATRIX_INITIALIZER	{ NULL, 0, 0, 0, XSDF_Managed }
typedef struct XsMatrix XsMatrix;
#endif

XSTYPES_DLL_API void XsMatrix_ref(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, XsReal* buffer, XsDataFlags flags);
XSTYPES_DLL_API void XsMatrix_construct(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride);
XSTYPES_DLL_API void XsMatrix_assign(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride);
XSTYPES_DLL_API void XsMatrix_destruct(XsMatrix* thisPtr);
XSTYPES_DLL_API void XsMatrix_copy(XsMatrix* copy, XsMatrix const* src);
XSTYPES_DLL_API void XsMatrix_setZero(XsMatrix* thisPtr);
XSTYPES_DLL_API int XsMatrix_empty(const XsMatrix* thisPtr);
XSTYPES_DLL_API void XsMatrix_multiplyScalar(const XsMatrix* thisPtr, XsReal scalar, XsMatrix* dest);
XSTYPES_DLL_API XsSize XsMatrix_offset(const XsMatrix* thisPtr, XsSize row, XsSize column);
XSTYPES_DLL_API XsReal XsMatrix_value(const XsMatrix* thisPtr, XsSize row, XsSize column);
XSTYPES_DLL_API void XsMatrix_setValue(XsMatrix* thisPtr, XsSize row, XsSize column, XsReal value);
XSTYPES_DLL_API int XsMatrix_dimensionsMatch(const XsMatrix* thisPtr, XsSize rows, XsSize columns);
XSTYPES_DLL_API void XsMatrix_fromQuaternion(XsMatrix* thisPtr, const struct XsQuaternion* quat);
XSTYPES_DLL_API void XsMatrix_swap(XsMatrix* a, XsMatrix* b);

#define XsMatrix_offsetM(thisPtr, row, column)			(thisPtr->m_stride*row + column)

#ifdef __cplusplus
} // extern "C"
#endif
#ifndef XSENS_NO_PACK
#pragma pack(push, 1)
#endif
struct XsMatrix {
XSCPPPROTECTED
	XsReal* const m_data;		//!< Contained data
	const XsSize m_rows;		//!< Number of rows in the matrix
	const XsSize m_cols;		//!< Number of columns in the matrix
	const XsSize m_stride;		//!< Number of items per row in memory (usually equal to cols but not always)
	const int m_flags;			//!< Flags for data management

#ifdef __cplusplus
	//! \brief Return the data management flags of the matrix.
	inline int flags() { return m_flags; }
public:
	/*! \brief Initialize an XsMatrix object with the specified number of \a rows and \a cols */
	inline explicit XsMatrix(XsSize rows = 0, XsSize cols = 0, XsSize strde = 0, const XsReal* dat = 0)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		if (rows && cols)
			XsMatrix_construct(this, rows, cols, strde?strde:cols, dat, 0);
	}

	/*! \brief Initialize an XsMatrix object from the \a other XsMatrix */
	inline XsMatrix(const XsMatrix& other)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		XsMatrix_copy(this, &other);
	}

	/*! \brief Initialize an XsMatrix object that references the data passed in \a ref. \a rows, \a cols and \a stride can be used to specify the layout of the data */
	inline explicit XsMatrix(XsReal* ref, XsSize rows, XsSize cols, XsSize stride, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_rows(rows)
		, m_cols(cols)
		, m_stride(stride)
		, m_flags(flags)
	{
	}

	/*! \brief Initialize a copy of \a other in an XsMatrix object that references the data passed in \a ref. \a rows, \a cols and \a stride can be used to specify the layout of the data */
	inline explicit XsMatrix(const XsMatrix& other, XsReal* ref, XsSize rows, XsSize cols, XsSize stride, XsDataFlags flags = XSDF_None)
		: m_data(ref)
		, m_rows(rows)
		, m_cols(cols)
		, m_stride(stride)
		, m_flags(flags)
	{
		XsMatrix_copy(this, &other);
	}

	//! \brief \copybrief XsMatrix_fromQuaternion
	inline explicit XsMatrix(const XsQuaternion& quat)
		: m_data(0)
		, m_rows(0)
		, m_cols(0)
		, m_stride(0)
		, m_flags(0)
	{
		XsMatrix_fromQuaternion(this, &quat);
	}

	//! \copydoc XsMatrix_destruct
	inline ~XsMatrix()
	{
		XsMatrix_destruct(this);
	}

	/*! \brief Resize the matrix to the specified number of \a rows and \a cols, destroying its current contents */
	inline void setSize(XsSize rows, XsSize cols, XsSize stride = 0)
	{
		XsMatrix_assign(this, rows, cols, stride, 0, 0);
	}

	/*! \brief \copybrief XsMatrix_copy */
	inline XsMatrix& operator=(const XsMatrix& other)
	{
		XsMatrix_copy(this, &other);
		return *this;
	}

	//! \brief \copybrief XsMatrix_empty
	inline bool empty() const
	{
		return 0 != XsMatrix_empty(this);
	}

	//! \brief \copybrief XsMatrix_setZero
	inline void setZero()
	{
		XsMatrix_setZero(this);
	}
	
	//! \copydoc XsMatrix_offset */
	inline XsSize offset(XsSize row, XsSize column) const
	{
		return XsMatrix_offset(this, row, column);
	}

	/*! \brief Returns the value at \a row and \a column in the matrix */
	inline XsReal value(XsSize row, XsSize column) const
	{
		return m_data[XsMatrix_offset(this, row, column)];

	}

	/*! \brief Sets the \a value at \a row and \a column in the matrix */
	inline void setValue(XsSize row, XsSize column, XsReal value)
	{
		m_data[XsMatrix_offsetM(this, row, column)] = value;
	}

	/*! \brief Returns a pointer to the data in \a row */
	inline const XsReal* operator[](XsSize row) const
	{
		return &m_data[XsMatrix_offsetM(this, row, 0)];
	}

	/*! \brief Returns a reference to the data in \a row */
	inline XsReal* operator[](XsSize row) {
		return &m_data[XsMatrix_offsetM(this, row, 0)];
	}

	/*! \brief \copybrief XsMatrix_multiplyScalar */
	inline XsMatrix operator*(XsReal scalar) const
	{
		XsMatrix tmp(m_rows, m_cols);
		XsMatrix_multiplyScalar(this, scalar, &tmp);
		return tmp;
	}

	/*! \brief \copybrief XsMatrix_fromQuaternion */
	inline XsMatrix& fromQuaternion(const XsQuaternion& quat)
	{
		XsMatrix_fromQuaternion(this, &quat);
		return *this;
	}

	/*! \brief Fill the matrix with zeroes */
	inline void zero()
	{
		for (XsSize r = 0; r < m_rows; ++r)
			for (XsSize c = 0; c < m_cols; ++c)
				m_data[XsMatrix_offsetM(this, r, c)] = XsMath_zero;
	}

	/*! \brief Return the number of rows in the matrix */
	inline XsSize rows() const
	{
		return m_rows;
	}

	/*! \brief Return the number of columns in the matrix */
	inline XsSize cols() const
	{
		return m_cols;
	}

	/*! \brief Return the stride of the matrix.
		\details The stride of a matrix is for internal administration. It defines the number of items
		in a row in the data buffer. This is always greater than or equal to the number of columns.
		Especially for matrices that reference a part of another matrix this may differ from the
		cols() value.
		\returns The stride of the matrix.
	*/
	inline XsSize stride() const
	{
		return m_stride;
	}

	//! \brief Return a const pointer to the internal data
	inline const XsReal* data() const
	{
		return m_data;
	}

	//! \brief Returns true if \a other is numerically identical to this
	inline bool operator ==(const XsMatrix& other) const
	{
		if (this == &other)
			return true;
		if (m_rows != other.m_rows || m_cols != other.m_cols)
			return false;
		for (XsSize r = 0; r < m_rows; ++r)
			for (XsSize c = 0; c < m_cols; ++c)
				if (m_data[XsMatrix_offsetM(this, r, c)] != other.m_data[XsMatrix_offsetM((&other), r, c)])
					return false;
		return true;
	}

#endif
};
#ifndef XSENS_NO_PACK
#pragma pack(pop)
#endif

#ifdef __cplusplus
//! \brief Multiplies all values in the matrix \a m by \a scalar
inline XsMatrix operator *(XsReal scalar, const XsMatrix &m)
{
	return (m*scalar);
}
#endif

#endif // file guard
