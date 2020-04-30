
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

#ifndef XSRESETMETHOD_H
#define XSRESETMETHOD_H

/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xstypes {
/*! \brief Orientation reset type. */
enum XsResetMethod {
	XRM_StoreAlignmentMatrix	= 0,				//!< \brief Store the current object alignment matrix to persistent memory
	XRM_Heading					= 1,				//!< \brief Reset the heading (yaw)
	XRM_Object					= 3,				//!< \brief Reset the attitude (roll, pitch), same as XRM_Inclination
	XRM_Inclination				= XRM_Object,		//!< \brief Reset the inclination (roll, pitch), same as XRM_Object
	XRM_Alignment				= 4,				//!< \brief Reset heading and attitude \details This effectively combines the XRM_Heading and XRM_Object
	XRM_Global					= XRM_Alignment,	//!< \brief Reset the full orientation of the device \details Obsolete. Use XRM_Alignment instead.
	XRM_DefaultHeading			= 5,				//!< \brief Revert to default behaviour for heading, undoes XRM_Heading
	XRM_DefaultInclination		= 6,				//!< \brief Revert to default behaviour for inclination, undoes XRM_Inclination
	XRM_DefaultAlignment		= 7,				//!< \brief Revert to default behaviour for heading and inclination, undoes any reset
	XRM_None										//!< \brief No reset planned
};
/*! @} */
typedef enum XsResetMethod XsResetMethod;
//AUTO }

#endif
