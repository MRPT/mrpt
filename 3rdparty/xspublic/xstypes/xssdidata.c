
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

#include "xssdidata.h"

/*! \class XsSdiData
	\brief Contains StrapDown Integration (SDI) data.
	\details SDI data consists of a rotation and an acceleration, expressed as an orientation increment
	(also known as deltaQ) and a velocity increment (also known as deltaV).
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsSdiData
	\brief Initialize an %XsSdiData object with the optional arguments.
	\param orientationIncrement The orientation increment to initialize the object with, may be 0
	\param velocityIncrement The velocity increment to initialize the object with, may be 0
*/
void XsSdiData_construct(XsSdiData* thisPtr, const XsReal* orientationIncrement, const XsReal* velocityIncrement)
{
	if (orientationIncrement)
	{
		thisPtr->m_orientationIncrement.m_data[0] = orientationIncrement[0];
		thisPtr->m_orientationIncrement.m_data[1] = orientationIncrement[1];
		thisPtr->m_orientationIncrement.m_data[2] = orientationIncrement[2];
		thisPtr->m_orientationIncrement.m_data[3] = orientationIncrement[3];
	}
	else
		XsQuaternion_destruct(&thisPtr->m_orientationIncrement);

	if (velocityIncrement)
		XsVector3_assign(&thisPtr->m_velocityIncrement, velocityIncrement);
	else
		XsVector3_destruct(&thisPtr->m_velocityIncrement);
}

/*! \relates XsSdiData
	\brief Destruct the object, makes the fields invalid
*/
void XsSdiData_destruct(XsSdiData* thisPtr)
{
	XsQuaternion_destruct(&thisPtr->m_orientationIncrement);
	XsVector3_destruct(&thisPtr->m_velocityIncrement);
}

/*! @} */
