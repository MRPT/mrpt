
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

#ifndef XSSYNCLINE_H
#define XSSYNCLINE_H

/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xstypes {
/*! \brief Synchronization line identifiers */
enum XsSyncLine
{
	XSL_Inputs,							/*!< \brief Value for checking if a line is an input, any item equal to or greater than XSL_Inputs and less than XSL_Outputs is an input */
	XSL_In1 = XSL_Inputs,				/*!< \brief Sync In 1 \remark Applies to Awinda Station and Mt */
	XSL_In2,							/*!< \brief Sync In 2 \remark Applies to Awinda Station */
	XSL_Bi1In,							/*!< \brief Bidirectional Sync 1 In */
	XSL_ClockIn,						/*!< \brief Clock synchronisation input \remark Applies to Mk4 */
	XSL_CtsIn,							/*!< \brief RS232 CTS sync in */
	XSL_GnssClockIn,					/*!< \brief Clock synchronisation input line attached to internal GNSS unit \remark Applies to Mk4 */
	XSL_ExtTimepulseIn,					/*!< \brief External time pulse input (e.g. for external GNSS unit) \remark Applies to Mk4 with external device */
	XSL_ReqData,						/*!< \brief Serial data sync option, use \a XMID_ReqData message id for this \remark Applies to Mk4*/
	XSL_Gnss1Pps,						/*!< \brief GNSS 1PPS pulse sync line \remark Applies to MTi-7*/

	XSL_Outputs,						/*!< \brief Value for checking if a line is output. Values equal to or greater than this are outputs */
	XSL_Out1 = XSL_Outputs,				/*!< \brief Sync Out 1 \remark Applies to Awinda Station and Mt */
	XSL_Out2,							/*!< \brief Sync Out 2 \remark Applies to Awinda Station */
	XSL_Bi1Out,							/*!< \brief Bidirectional Sync 1 Out */
	XSL_RtsOut,							/*!< \brief RS232 RTS sync out */

	XSL_Invalid							/*!< \brief Invalid sync setting. Used if no sync line is set */
};
/*! @} */
typedef enum XsSyncLine XsSyncLine;
//AUTO }

#endif
