
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

#include "xssyncsetting.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Returns whether the selected line is configured as an input line
*/
int XsSyncSetting_isInput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_In1:
	case XSL_In2:
	case XSL_Bi1In:
	case XSL_ClockIn:
	case XSL_CtsIn:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Returns whether the selected line is configured as an output line
*/
int XsSyncSetting_isOutput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_Out1:
	case XSL_Out2:
	case XSL_Bi1Out:
	case XSL_RtsOut:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Swap the contents of \a a with \a b
*/
void XsSyncSetting_swap(XsSyncSetting* a, XsSyncSetting* b)
{
	XsSyncSetting tmp;
	memcpy(&tmp, a, sizeof(XsSyncSetting));
	memcpy(a, b, sizeof(XsSyncSetting));
	memcpy(b, &tmp, sizeof(XsSyncSetting));
}

/*! \brief Compares \a a with \a b
	\returns 0 if \a a equals \a b, negative value if \a a < \a b, positive value if \a a > \a b
	\param[in] a Sync setting a
	\param[in] b Sync setting b
*/
int XsSyncSetting_compare(const struct XsSyncSetting* a, const struct XsSyncSetting* b)
{
	assert(a && b);

	if (a->m_line < b->m_line) return -1;
	if (b->m_line < a->m_line) return 1;
	if (a->m_function < b->m_function) return -1;
	if (b->m_function < a->m_function) return 1;
	if (a->m_polarity < b->m_polarity) return -1;
	if (b->m_polarity < a->m_polarity) return 1;
	if (XsSyncSetting_isOutput(a))
	{
		// only relevant for output triggers, ignored for inputs since inputs always trigger on an edge
		if (a->m_pulseWidth < b->m_pulseWidth) return -1;
		if (b->m_pulseWidth < a->m_pulseWidth) return 1;
	}
	if (a->m_offset < b->m_offset) return -1;
	if (b->m_offset < a->m_offset) return 1;
	if (a->m_skipFirst < b->m_skipFirst) return -1;
	if (b->m_skipFirst < a->m_skipFirst) return 1;
	if (a->m_skipFactor < b->m_skipFactor) return -1;
	if (b->m_skipFactor < a->m_skipFactor) return 1;
	if (a->m_function == XSF_ClockBiasEstimation)
	{
		if (a->m_clockPeriod < b->m_clockPeriod) return -1;
		if (b->m_clockPeriod < a->m_clockPeriod) return 1;
	}
	if (a->m_triggerOnce < b->m_triggerOnce) return -1;
	if (b->m_triggerOnce < a->m_triggerOnce) return 1;

	return 0;
}

/*! @} */
