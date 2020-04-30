
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

#ifndef XSVECTOR3_H
#define XSVECTOR3_H

#include "xsvector.h"

struct XsVector3;
#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
typedef struct XsVector3 XsVector3;
#endif

XSTYPES_DLL_API void XsVector3_construct(XsVector3* thisPtr, const XsReal* src);
XSTYPES_DLL_API void XsVector3_assign(XsVector3* thisPtr, const XsReal* src);
XSTYPES_DLL_API void XsVector3_destruct(XsVector3* thisPtr);
XSTYPES_DLL_API void XsVector3_copy(XsVector* copy, XsVector3 const* src);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
/* This is allowed since the C standard says that no padding appears before the first member of a struct.
	Basically we're defining a union between a C++ inherited type and a C encapsulated type.
*/
struct XsVector3 : public XsVector {
XSCPPPROTECTED
#else
struct XsVector3 {
	XsVector m_vector;		//!< The underlying vector
#endif
	XsReal XSCCONST m_fixedData[3];				//!< Fixed size storage for the components in the vector

#ifdef __cplusplus
public:
	//! \brief Constructs an empty vector3
	XsVector3()	: XsVector(m_fixedData, 3, XSDF_FixedSize)
	{
		//XsVector3_construct(this, 0);
	}

	//! \brief Constructs a vector3 from an \a other XsVector
	XsVector3(XsVector3 const& other) : XsVector(other, m_fixedData, 3, XSDF_FixedSize)
	{
	}

	//! \brief Constructs a vector3 from an \a other XsVector
	XsVector3(XsVector const& other) : XsVector(other, m_fixedData, 3, XSDF_FixedSize)
	{
	}

	//! \brief Constructs a vector3 using the values \a x, \a y, \a z
	XsVector3(XsReal x, XsReal y, XsReal z)	: XsVector(m_fixedData, 3, XSDF_FixedSize)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
	}

	//! \brief Return a 3-element zero vector
	static XsVector3 const& zero3()
	{
		static const XsVector3 rv(XsMath_zero, XsMath_zero, XsMath_zero);
		return rv;
	}

	using XsVector::operator=;
	//! \brief Assignment operator. Copies from \a other into this
	inline XsVector3& operator=(const XsVector3& other)
	{
		XsVector_copy(this, &other);
		return *this;
	}

	//	using XsVector::operator[];
#endif
};

#endif
