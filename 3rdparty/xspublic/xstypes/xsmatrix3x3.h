
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

#ifndef XSMATRIX3X3_H
#define XSMATRIX3X3_H

#include "xsmatrix.h"

struct XsMatrix3x3;
#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
typedef struct XsMatrix3x3 XsMatrix3x3;
#endif


XSTYPES_DLL_API void XsMatrix3x3_construct(XsMatrix3x3* thisPtr);
XSTYPES_DLL_API void XsMatrix3x3_assign(XsMatrix3x3* thisPtr, const XsReal* src, XsSize srcStride);
XSTYPES_DLL_API void XsMatrix3x3_destruct(XsMatrix3x3* thisPtr);
XSTYPES_DLL_API void XsMatrix3x3_copy(XsMatrix* copy, XsMatrix3x3 const* src);

#ifdef __cplusplus
} // extern "C"

  /* This is allowed since the C standard says that no padding appears before the first member of a struct.
	Basically we're defining a union between a C++ inherited type and a C encapsulated type.
*/
struct XsMatrix3x3 : public XsMatrix {
XSCPPPROTECTED
#endif
#ifndef __cplusplus
struct XsMatrix3x3 {
	struct XsMatrix m_matrix;	//!< The underlying XsMatrix
#endif
	XsReal XSCCONST m_fixedData[9];			//!< Fixed storage for the elements of the matrix

#ifdef __cplusplus
public:
	//! \brief Constructs an XsMatrix3x3
	XsMatrix3x3() : XsMatrix(m_fixedData, 3, 3, 3, XSDF_FixedSize)
	{
	}

	//! \brief Constructs an XsMatrix3x3 from an \a other XsMatrix
	XsMatrix3x3(const XsMatrix& other) : XsMatrix(other, m_fixedData, 3, 3, 3, XSDF_FixedSize)
	{
	}

	//! \brief Constructs an XsMatrix3x3 from an \a other XsMatrix
	XsMatrix3x3(const XsMatrix3x3& other) : XsMatrix(other, m_fixedData, 3, 3, 3, XSDF_FixedSize)
	{
	}

	//! \brief Constructs an XsMatrix3x3 from a set of values
	XsMatrix3x3(XsReal r1c1, XsReal r1c2, XsReal r1c3,
				XsReal r2c1, XsReal r2c2, XsReal r2c3,
				XsReal r3c1, XsReal r3c2, XsReal r3c3) : XsMatrix(m_fixedData, 3, 3, 3, XSDF_FixedSize)
	{
		m_fixedData[0] = r1c1;
		m_fixedData[1] = r1c2;
		m_fixedData[2] = r1c3;
		m_fixedData[3] = r2c1;
		m_fixedData[4] = r2c2;
		m_fixedData[5] = r2c3;
		m_fixedData[6] = r3c1;
		m_fixedData[7] = r3c2;
		m_fixedData[8] = r3c3;
	}
//	using XsMatrix::operator=;
//	using XsMatrix::operator[];
#endif
};
#if 0
}	// for Qt lupdate parser
#endif

#endif
