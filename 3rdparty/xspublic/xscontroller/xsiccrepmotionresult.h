
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

#ifndef XSICCREPMOTIONRESULT_H
#define XSICCREPMOTIONRESULT_H

#include <xstypes/pstdint.h>

/*! \brief Contains the result of the representative motion processed by ICC
*/
struct XsIccRepMotionResult
{
	float m_ddtAccuracy;	//!< The ddtAccuracy of the In-Run Compass Calibration
	uint8_t m_dimension;	//!< The dimension of the In-Run Compass Calibration
	uint8_t m_status;		//!< The status of the In-Run Compass Calibration

#ifdef __cplusplus
	XsIccRepMotionResult() : m_ddtAccuracy(0.0), m_dimension(0), m_status(0)
	{

	}

	/*! \brief Copy constructor for a filter profile object
		\param other the filter profile object to construct a copy of
	*/
	XsIccRepMotionResult(const XsIccRepMotionResult& other)
		: m_ddtAccuracy(other.m_ddtAccuracy)
		, m_dimension(other.m_dimension)
		, m_status(other.m_status)
	{
	}

	//! \returns the ddtAccuracy
	inline float ddtAccuracy() const
	{
		return m_ddtAccuracy;
	}

	//! \returns the dimension
	inline uint8_t dimension() const
	{
		return m_dimension;
	}

	//! \returns the status
	inline uint8_t status() const
	{
		return m_status;
	}
#endif
};

typedef struct XsIccRepMotionResult XsIccRepMotionResult;

#endif
