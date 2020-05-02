
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

#ifndef XSSYNCSETTINGS_H
#define XSSYNCSETTINGS_H

#include <string.h>

#include "pstdint.h"
#include "xstypesconfig.h"
#include "xssyncline.h"
#include "xssyncfunction.h"
#include "xssyncpolarity.h"

struct XsSyncSetting;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSSYNCSETTINGS_INITIALIZER		{ XSL_Invalid, XSF_Invalid, XSP_None, 1, 0, 0, 0, 0, 0, 0 }
#endif

XSTYPES_DLL_API int XsSyncSetting_isInput(const struct XsSyncSetting* thisPtr);
XSTYPES_DLL_API int XsSyncSetting_isOutput(const struct XsSyncSetting* thisPtr);
XSTYPES_DLL_API void XsSyncSetting_swap(struct XsSyncSetting* a, struct XsSyncSetting* b);
XSTYPES_DLL_API int XsSyncSetting_compare(const struct XsSyncSetting* a, const struct XsSyncSetting* b);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief A structure for storing all xsens sync settings */
struct XsSyncSetting {
	XsSyncLine		m_line;						/*!< The sync lines enabled. \see XsSyncLine. */
	XsSyncFunction	m_function;					/*!< The action to be performed, when an input sync line changes \see XsSyncFunction. */
	XsSyncPolarity	m_polarity;					/*!< The edge on which the action is performed, \see XsSyncPolarity. */
	uint32_t		m_pulseWidth;				/*!< The time to keep the line polarity before toggling back, in microseconds. */
	int32_t			m_offset;					/*!< The time between reception of a line change and the execution of the sync action, in microseconds. */
	uint16_t		m_skipFirst;				/*!< The number of frames to skip before executing the action. */
	uint16_t		m_skipFactor;				/*!< The number of frames to skip between 2 actions. */
	uint16_t		m_clockPeriod;				/*!< The frequency of the external clock in milliseconds, only valid when function is XSF_ClockBiasEstimation. */
	uint8_t			m_triggerOnce;				/*!< Whether the action is repeated for each frame. */
	uint8_t			m_padding;					/*!< Padding to get at a 4 byte boundary so memcpy/memcmp works with sizeof(XsSyncSetting) */
#ifdef __cplusplus
	//! \brief Default constructor, initializes to the given (default) settings
	explicit XsSyncSetting(XsSyncLine line = XSL_Invalid
				, XsSyncFunction function = XSF_Invalid
				, XsSyncPolarity polarity = XSP_RisingEdge
				, uint32_t pulseWidth = 1000
				, int32_t offset = 0
				, uint16_t skipFirst = 0
				, uint16_t skipFactor = 0
				, uint16_t clockPeriod = 0
				, uint8_t triggerOnce = 0)
		: m_line(line)
		, m_function(function)
		, m_polarity(polarity)
		, m_pulseWidth(pulseWidth)
		, m_offset(offset)
		, m_skipFirst(skipFirst)
		, m_skipFactor(skipFactor)
		, m_clockPeriod(clockPeriod)
		, m_triggerOnce(triggerOnce)
		, m_padding(0)
	{
	}

	//! \brief Construct a XsSyncSetting as a copy of \a other.
	XsSyncSetting(const XsSyncSetting& other)
	{
		memcpy(this, &other, sizeof(XsSyncSetting));
	}

	//! \brief Copy values of \a other into this.
	const XsSyncSetting& operator =(const XsSyncSetting& other)
	{
		if (this != &other)
			memcpy(this, &other, sizeof(XsSyncSetting));
		return *this;
	}

	//! \brief \copybrief XsSyncSetting_isInput
	inline bool isInput() const
	{
		return 0 != XsSyncSetting_isInput(this);
	}

	//! \brief \copybrief XsSyncSetting_isOutput
	inline bool isOutput() const
	{
		return 0 != XsSyncSetting_isOutput(this);
	}

	/*! \brief Swap the contents with \a other
	*/
	inline void swap(XsSyncSetting& other)
	{
		XsSyncSetting_swap(this, &other);
	}

	/*! \brief Return true if \a other is identical to this
	*/
	inline bool operator == (const XsSyncSetting& other) const
	{
		return (this == &other) || XsSyncSetting_compare(this, &other) == 0;
	}

	/*! \brief Return true if \a other is not identical to this
	*/
	inline bool operator != (const XsSyncSetting& other) const
	{
		return (this != &other) && XsSyncSetting_compare(this, &other) != 0;
	}

	/*! \brief Return true if \a other is less than this
	*/
	inline bool operator < (const XsSyncSetting& other) const
	{
		return (this != &other) && XsSyncSetting_compare(this, &other) < 0;
	}
#endif
};

typedef struct XsSyncSetting XsSyncSetting;

#endif
