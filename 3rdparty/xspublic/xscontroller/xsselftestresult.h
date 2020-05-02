
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

#ifndef XSSELFTESTRESULT_H
#define XSSELFTESTRESULT_H

#include <xstypes/pstdint.h>

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Enumeration of bits that describe whether the various self-tests succeeded
	\see XsSelfTestResult
*/
enum XsSelfTestFlag {
	 XSTF_X		= 0x01
	,XSTF_Y		= 0x02
	,XSTF_Z		= 0x04
	,XSTF_AccShift = 0
	,XSTF_AccX	= XSTF_X << XSTF_AccShift
	,XSTF_AccY	= XSTF_Y << XSTF_AccShift
	,XSTF_AccZ	= XSTF_Z << XSTF_AccShift
	,XSTF_GyrShift = 3
	,XSTF_GyrX	= XSTF_X << XSTF_GyrShift
	,XSTF_GyrY	= XSTF_Y << XSTF_GyrShift
	,XSTF_GyrZ	= XSTF_Z << XSTF_GyrShift
	,XSTF_MagShift = 6
	,XSTF_MagX	= XSTF_X << XSTF_MagShift
	,XSTF_MagY	= XSTF_Y << XSTF_MagShift
	,XSTF_MagZ	= XSTF_Z << XSTF_MagShift
	,XSTF_Baro	= 1<<9
	,XSTF_Gnss	= 1<<10
};
/*! @} */
typedef enum XsSelfTestFlag XsSelfTestFlag;

/*! \brief Contains the results of a self-test performed by an Xsens device
*/
struct XsSelfTestResult {
	uint16_t m_flags;		//!< Flags that specify which tests have passed

#ifdef __cplusplus
	//! \brief Create a new %XsSelfTestResult from supplied flags
	static inline XsSelfTestResult create(uint16_t resultFlags)
	{
		XsSelfTestResult tmp = { resultFlags };
		return tmp;
	}

	//! \brief Returns whether the accelerometer x-axis passed (true) or failed (false) its self-test
	inline bool accX() const
	{
		return (m_flags & XSTF_AccX) != 0;
	}

	//! \brief Returns whether the accelerometer y-axis passed (true) or failed (false) its self-test
	inline bool accY() const
	{
		return (m_flags & XSTF_AccY) != 0;
	}

	//! \brief Returns whether the accelerometer z-axis passed (true) or failed (false) its self-test
	inline bool accZ() const
	{
		return (m_flags & XSTF_AccZ) != 0;
	}

	//! \brief Returns whether the gyroscope x-axis passed (true) or failed (false) its self-test
	inline bool gyrX() const
	{
		return (m_flags & XSTF_GyrX) != 0;
	}

	//! \brief Returns whether the gyroscope y-axis passed (true) or failed (false) its self-test
	inline bool gyrY() const
	{
		return (m_flags & XSTF_GyrY) != 0;
	}

	//! \brief Returns whether the gyroscope z-axis passed (true) or failed (false) its self-test
	inline bool gyrZ() const
	{
		return (m_flags & XSTF_GyrZ) != 0;
	}

	//! \brief Returns whether the magnetometer x-axis passed (true) or failed (false) its self-test
	inline bool magX() const
	{
		return (m_flags & XSTF_MagX) != 0;
	}

	//! \brief Returns whether the magnetometer y-axis passed (true) or failed (false) its self-test
	inline bool magY() const
	{
		return (m_flags & XSTF_MagY) != 0;
	}

	//! \brief Returns whether the magnetometer z-axis passed (true) or failed (false) its self-test
	inline bool magZ() const
	{
		return (m_flags & XSTF_MagZ) != 0;
	}

	/*!	\brief Returns whether the barometer passed (true) or failed (false) its self-test
		\details Only valid for MTi-7
		\returns True if the baro selftest has passed
	 */
	inline bool baro() const
	{
		return (m_flags & XSTF_Baro) != 0;
	}

	/*! \brief Returns whether the gnss passed (true) or failed (false) its self-test
		\details Only valid for MTi-7
		\returns True if the gnss selftest has passed
	 */
	inline bool gnss() const
	{
		return (m_flags & XSTF_Gnss) != 0;
	}
#endif
};

typedef struct XsSelfTestResult XsSelfTestResult;

#endif	// file guard
