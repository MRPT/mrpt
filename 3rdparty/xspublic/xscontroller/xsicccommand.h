
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

#ifndef XSICCCOMMAND_H
#define XSICCCOMMAND_H

#include <xstypes/pstdint.h>

/*! \brief ICC (Inrun Compass Calibration) commands.
	\details To be used inside XMID_IccCommand and XMID_IccCommandAck messages
*/
enum XsIccCommand
{
	XIC_StartRepMotion		= 0x00,	//!< Indicate to ICC the start of representative motion
	XIC_StopRepMotion		= 0x01,	//!< Indicate to ICC the end of representative motion
	XIC_StoreResults		= 0x02,	//!< Update the stored magnetometer calibration using the ICC estimated calibration values
	XIC_RepMotionState		= 0x03,	//!< Retrieve the current state of the representative motion
	XIC_Status				= 0x04	//!< Retrieve the current ICC status
};
typedef enum XsIccCommand XsIccCommand;

/*! \brief ICC status flag
	\details Used for status fields in XMID_IccCommand and XMID_IccCommandAck messages
*/
enum XsIccStatusFlag
{
	XISF_ddtWarning		= 0x01,	//!< Indicates magnetic disturbance
	XISF_notEnoughData	= 0x02,	//!< Indicates data during representative motion does not have enough observability for an estimate
	XISF_OutputStable	= 0x10,	//!< Indicates the ICC output is stable and used by the filter
	XISF_RepMoActive	= 0x20	//!< Indicates ICC is recording a representative motion
};
typedef enum XsIccStatusFlag XsIccStatusFlag;

#endif
