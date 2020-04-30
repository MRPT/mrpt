
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

#ifndef XSCANOUTPUTCONFIGURATION_H
#define XSCANOUTPUTCONFIGURATION_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xscandataidentifier.h"
#include "xscanframeformat.h"

#define XS_MAX_CANOUTPUTCONFIGURATIONS			(16)

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSCANOUTPUTCONFIGURATION_INITIALIZER		{ XCDI_None, 0 }
#endif

struct XsCanOutputConfiguration;

XSTYPES_DLL_API void XsCanOutputConfiguration_swap(struct XsCanOutputConfiguration* a, struct XsCanOutputConfiguration* b);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \brief Single data type CAN output configuration
	\details This structure contains a single data type and the frequency at which it should be produced.
	If m_frequency is 0xFFFF and the %XsCanOutputConfiguration is used for input, the device will configure
	itself to its maximum frequency for the data type. If it is 0xFFFF and reported by the device,
	the data has no maximum frequency, but is sent along with appropriate packets (e.g. packet counter)
*/
struct XsCanOutputConfiguration {
	XsCanFrameFormat m_frameFormat;			//!< The frame format of the CAN message
	XsCanDataIdentifier m_dataIdentifier;	//!< The data identifier
	uint32_t m_id;							//!< The 11 or 29 bit ID identifier
	uint16_t m_frequency;					//!< The frequency

#ifdef __cplusplus
	//! Constructor, initializes to an empty object
	XsCanOutputConfiguration()
		: m_frameFormat(XCFF_11Bit_Identifier)
		, m_dataIdentifier(XCDI_Invalid)
		, m_id(0)
		, m_frequency(0)
	{}

	//! Constructor, initializes to specified values
	XsCanOutputConfiguration(XsCanFrameFormat il, XsCanDataIdentifier di, uint32_t id, uint16_t freq)
		: m_frameFormat(il)
		, m_dataIdentifier(di)
		, m_id(id)
		, m_frequency(freq)
	{}

	//! Comparison operator
	bool operator == (const XsCanOutputConfiguration& other) const
	{
		return (m_frameFormat == other.m_frameFormat &&
				m_dataIdentifier == other.m_dataIdentifier &&
				m_id == other.m_id &&
				m_frequency == other.m_frequency);
	}
#endif
};
typedef struct XsCanOutputConfiguration XsCanOutputConfiguration;

#endif
