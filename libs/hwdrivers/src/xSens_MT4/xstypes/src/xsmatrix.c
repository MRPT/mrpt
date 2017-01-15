/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsmatrix.h"
#include <stdlib.h>
#include <math.h>
#include "xsatomicint.h"
#include <string.h>
#include "xsmalloc.h"
#include "xsquaternion.h"

//lint -e123 -e40 -e522  the INC_ALLOC and INC_ALLOC(L) definitions confuse PCLint

XsAtomicInt XSTYPES_DLL_API XsMatrix_allocCount = XSATOMICINT_INITIALIZER;	//!< The number of times XsMatrix_ functions have allocated memory
XsAtomicInt XSTYPES_DLL_API XsMatrix_freeCount = XSATOMICINT_INITIALIZER;	//!< The number of times XsMatrix_ functions have freed memory

#ifdef XSENS_DEBUG
#define INC_ALLOC()		(void)XsAtomicInt_preIncrement(&XsMatrix_allocCount)
#define INC_FREE()		(void)XsAtomicInt_preIncrement(&XsMatrix_freeCount)
#else
#define INC_ALLOC()		((void)0)
#define INC_FREE()		((void)0)
#endif

//lint -e641 conversion from enum to int should not be a problem

/*! \class XsMatrix
	\brief A class that represents a matrix of real numbers
*/	

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsMatrix \brief Construct the %XsMatrix as a reference to data in \a buffer */
void XsMatrix_ref(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, XsReal* buffer, XsDataFlags flags)
{
	//assert(!(thisPtr->m_flags & XSDF_FixedSize));
	//XsMatrix_destruct(thisPtr);

	*((XsReal**) &thisPtr->m_data) = buffer;
	*((int*) &thisPtr->m_flags) = (int) flags;
	*((XsSize*) &thisPtr->m_rows) = rows;
	*((XsSize*) &thisPtr->m_cols) = cols;
	*((XsSize*) &thisPtr->m_stride) = stride;
}

/*! \relates XsMatrix \brief Init the %XsMatrix and copy the data from \a src into the matrix if \a src is not null */
void XsMatrix_construct(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride)
{
	XsSize r,c;
	XsSize size = rows*stride;

	if (stride == 0)
	{
		stride = cols;
		size = rows * stride;
	}
	if (size)
	{
		// init to size
		*((XsReal**) &thisPtr->m_data) = (XsReal*) xsMathMalloc(size*sizeof(XsReal));
		INC_ALLOC();
	}
	else
		*((XsReal**) &thisPtr->m_data) = 0;
	*((int*) &thisPtr->m_flags) = XSDF_Managed;
	*((XsSize*) &thisPtr->m_rows) = rows;
	*((XsSize*) &thisPtr->m_cols) = cols;
	*((XsSize*) &thisPtr->m_stride) = stride;

	if (src && size)
	{
		if (srcStride == 0 || srcStride == stride)
			memcpy(thisPtr->m_data, src, size*sizeof(XsReal));
		else
		{
			for (r = 0; r < rows; ++r)
				for (c = 0; c < cols; ++c)
					thisPtr->m_data[r*stride+c] = src[r*srcStride + c];
		}
	}
}

/*! \relates XsMatrix \brief Init the %XsMatrix and copy the data from \a src into the matrix if \a src is not null */
void XsMatrix_assign(XsMatrix* thisPtr, XsSize rows, XsSize cols, XsSize stride, const XsReal* src, XsSize srcStride)
{
	XsSize r,c;
	XsSize size = rows*stride;

	if (thisPtr->m_flags & XSDF_FixedSize)
	{
		if (rows == 0 && cols == 0)
		{
			*((int*) &thisPtr->m_flags) |= XSDF_Empty;
			return;
		}

		assert(thisPtr->m_rows == rows && thisPtr->m_cols == cols);
		stride = thisPtr->m_stride;
		size = thisPtr->m_rows * stride;
		*((int*) &thisPtr->m_flags) &= ~XSDF_Empty;
	}
	else
	{
		if (thisPtr->m_rows == rows && thisPtr->m_cols == cols &&
			(stride == 0 || stride == thisPtr->m_stride))
		{
			stride = thisPtr->m_stride;
			size = rows * stride;
		}
		else
		{
			if (stride == 0)
			{
				stride = cols;
				size = rows * stride;
			}
			if (size > thisPtr->m_rows*thisPtr->m_stride || thisPtr->m_rows == 0)
			{
				XsMatrix_destruct(thisPtr);
				if (size)
				{
					// init to size
					*((XsReal**) &thisPtr->m_data) = (XsReal*) xsMathMalloc(size*sizeof(XsReal));
					*((int*) &thisPtr->m_flags) = XSDF_Managed;
					INC_ALLOC();
				}
			}
			*((XsSize*) &thisPtr->m_rows) = rows;
			*((XsSize*) &thisPtr->m_cols) = cols;
			*((XsSize*) &thisPtr->m_stride) = stride;
		}
	}
	if (src && size)
	{
		if (srcStride == 0 || srcStride == stride)
			memcpy(thisPtr->m_data, src, size*sizeof(XsReal));
		else
		{
			for (r = 0; r < rows; ++r)
				for (c = 0; c < cols; ++c)
					thisPtr->m_data[r*stride+c] = src[r*srcStride + c];
		}
	}
}

/*! \relates XsMatrix \brief Clear the XsMatrix and release allocated resources */
void XsMatrix_destruct(XsMatrix* thisPtr)
{
	if (thisPtr->m_data && (thisPtr->m_flags & XSDF_Managed))
	{
		// clear contents
		xsMathFree((void*) thisPtr->m_data);
		INC_FREE();
	}
	// init to 0
	if (!(thisPtr->m_flags & XSDF_FixedSize))
	{
		*((XsReal**) &thisPtr->m_data) = 0;
		*((XsSize*) &thisPtr->m_rows) = 0;
		*((XsSize*) &thisPtr->m_cols) = 0;
		*((XsSize*) &thisPtr->m_stride) = 0;
		*((int*) &thisPtr->m_flags) = 0;
	}
	else
		*((int*) &thisPtr->m_flags) |= XSDF_Empty;
}

/*! \relates XsMatrix \brief Copy the contents of \a copy to the %XsMatrix */
void XsMatrix_copy(XsMatrix* copy, XsMatrix const* src)
{
	if (copy == src)
	{
		return;
	}
	XsMatrix_assign(copy, src->m_rows, src->m_cols, 0, src->m_data, src->m_stride);
}

/*! \relates XsMatrix \brief Set all the values in the matrix to zero */
void XsMatrix_setZero(XsMatrix* thisPtr)
{
	XsSize r,c,stride = thisPtr->m_stride;
	for (r = 0; r < thisPtr->m_rows; ++r)
		for (c = 0; c < thisPtr->m_cols; ++c)
			thisPtr->m_data[r*stride+c] = XsMath_zero;
}

/*! \relates XsMatrix \brief Returns not zero if the matrix contains no values */
int XsMatrix_empty(const XsMatrix* thisPtr)
{
	return (thisPtr->m_rows == 0) || (thisPtr->m_cols == 0) || (thisPtr->m_flags & XSDF_Empty);
}

/*! \relates XsMatrix \brief Multiplies all values in this XsMatrix by \a scalar
	\param scalar : Value to multiply by
	\param dest : The XsMatrix to store the result in
*/
void XsMatrix_multiplyScalar(const XsMatrix* thisPtr, XsReal scalar, XsMatrix* dest)
{
	XsSize r,c,stride = thisPtr->m_stride, stride2;
	XsMatrix_assign(dest, thisPtr->m_rows, thisPtr->m_cols, 0, 0, 0);
	stride2 = dest->m_stride;
	for (r = 0; r < thisPtr->m_rows; ++r)
		for (c = 0; c < thisPtr->m_cols; ++c)
			dest->m_data[r*stride2+c] = thisPtr->m_data[r*stride+c] * scalar;
}

/*! \relates XsMatrix
	\brief Returns the offset in the data for accessing the value at \a row and \a column 
	\param row The row of the value
	\param column The column of the value
	\returns The offset of the requested item in the internal buffer
*/
XsSize XsMatrix_offset(const XsMatrix* thisPtr, XsSize row, XsSize column)
{
	return XsMatrix_offsetM(thisPtr, row, column);
}

//! \relates XsMatrix \brief Returns the data value at \a row and \a column
XsReal XsMatrix_value(const XsMatrix* thisPtr, XsSize row, XsSize column)
{
	return thisPtr->m_data[XsMatrix_offsetM(thisPtr, row, column)];
}


//! \relates XsMatrix \brief Sets the data \a value at \a row and \a column
void XsMatrix_setValue(XsMatrix* thisPtr, XsSize row, XsSize column, XsReal value)
{
	thisPtr->m_data[XsMatrix_offsetM(thisPtr, row, column)] = value;
}

//! \relates XsMatrix \brief Returns not zero if the dimensions of the %XsMatrix are equal to \a rows and \a columns
int XsMatrix_dimensionsMatch(const XsMatrix* thisPtr, XsSize rows, XsSize columns)
{
	return thisPtr->m_rows == rows && thisPtr->m_cols == columns;
}

/*! \relates XsMatrix \brief Get an orientation matrix representation of the quaternion. */
void XsMatrix_fromQuaternion(XsMatrix* thisPtr, const XsQuaternion* quat)
{
	XsReal q00, q11, q22, q33, q01, q02, q03, q12, q13, q23;

	if (XsQuaternion_empty(quat))
	{
		XsMatrix_destruct(thisPtr);
		return;
	}

	q00 = quat->m_w*quat->m_w;
	q11 = quat->m_x*quat->m_x;
	q22 = quat->m_y*quat->m_y;
	q33 = quat->m_z*quat->m_z;

	q01 = quat->m_w*quat->m_x;
	q02 = quat->m_w*quat->m_y;
	q03 = quat->m_w*quat->m_z;

	q12 = quat->m_x*quat->m_y;
	q13 = quat->m_x*quat->m_z;
	q23 = quat->m_y*quat->m_z;

	XsMatrix_assign(thisPtr, 3, 3, 3, 0, 0);
	
	XsMatrix_setValue(thisPtr, 0, 0, q00 + q11 - q22 - q33);
	XsMatrix_setValue(thisPtr, 0, 1, (q12 - q03) * XsMath_two);
	XsMatrix_setValue(thisPtr, 0, 2, (q13 + q02) * XsMath_two);

	XsMatrix_setValue(thisPtr, 1, 0, (q12 + q03) * XsMath_two);
	XsMatrix_setValue(thisPtr, 1, 1, q00 - q11 + q22 - q33);
	XsMatrix_setValue(thisPtr, 1, 2, (q23 - q01) * XsMath_two);

	XsMatrix_setValue(thisPtr, 2, 0, (q13 - q02) * XsMath_two);
	XsMatrix_setValue(thisPtr, 2, 1, (q23 + q01) * XsMath_two);
	XsMatrix_setValue(thisPtr, 2, 2, q00 - q11 - q22 + q33);
}

/*! \relates XsMatrix \brief Swap the contents of \a a and \a b
	\details This function swaps the internal buffers so no actual data is moved around.
	This won't work for unmanaged data such as fixed size matrices (XsMatrix3x3)
	\param a Object whose contents will be placed in \a b
	\param b Object whose contents will be placed in \a a
*/
void XsMatrix_swap(XsMatrix* a, XsMatrix* b)
{
	XsMatrix tmp;
	assert((!a->m_data || (a->m_flags & XSDF_Managed)) && (!b->m_data || (b->m_flags & XSDF_Managed)));

	*((XsReal**) &tmp.m_data) = a->m_data;
	*((XsSize*) &tmp.m_rows) = a->m_rows;
	*((XsSize*) &tmp.m_cols) = a->m_cols;
	*((XsSize*) &tmp.m_stride) = a->m_stride;
	*((int*) &tmp.m_flags) = a->m_flags;

	*((XsReal**) &a->m_data) = b->m_data;
	*((XsSize*) &a->m_rows) = b->m_rows;
	*((XsSize*) &a->m_cols) = b->m_cols;
	*((XsSize*) &a->m_stride) = b->m_stride;
	*((int*) &a->m_flags) = b->m_flags;

	*((XsReal**) &b->m_data) = tmp.m_data;
	*((XsSize*) &b->m_rows) = tmp.m_rows;
	*((XsSize*) &b->m_cols) = tmp.m_cols;
	*((XsSize*) &b->m_stride) = tmp.m_stride;
	*((int*) &b->m_flags) = tmp.m_flags;
}

/*! @} */

//lint +e123 +e40 +e522
