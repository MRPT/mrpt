
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

#ifndef XSCANDATAIDENTIFIER_H
#define XSCANDATAIDENTIFIER_H

//! \brief Max frequency values for can interface
#define XCDI_MAX_FREQUENCY_VAL	0x07FF
#define XCDI_MAX_FREQUENCY		((uint16_t) XCDI_MAX_FREQUENCY_VAL)

//////////////////////////////////////////////////////////////////////////////////////////
/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xstypes {
/*!	\enum XsCanDataIdentifier
	\brief Defines the data identifiers for CAN messages

*/
enum XsCanDataIdentifier
{
	XCDI_Invalid			= 0x00,

	/* Group Information & Timestamp Messages */
	XCDI_Error				= 0x01,
	XCDI_Warning			= 0x02,

	XCDI_SampleTime			= 0x05,	//!< Sample Time in us
	XCDI_GroupCounter		= 0x06,
	XCDI_UtcTime			= 0x07,

	/* Group Status Messages */
	XCDI_StatusWord 		= 0x11,

	/* Group Quaternion Messages */
	XCDI_Quaternion			= 0x21,
	XCDI_EulerAngles		= 0x22,
	XCDI_RotationMatrix		= 0x23,

	/* Group Inertial Data Messages */
	XCDI_DeltaV				= 0x31,	//!< DeltaV SDI data output
	XCDI_RateOfTurn			= 0x32,
	XCDI_DeltaQ				= 0x33,	//!< DeltaQ SDI data
	XCDI_Acceleration		= 0x34,
	XCDI_FreeAcceleration	= 0x35,

	/* Group Magnetic Field */
	XCDI_MagneticField		= 0x41,	//!< Magnetic field data in a.u.

	/* Group Temperature & Pressure Messages */
	XCDI_Temperature		= 0x51,	//!< Temperature
	XCDI_BaroPressure		= 0x52,	//!< Pressure output recorded from the barometer

	/* Group High-Rate Data Messages */
	XCDI_RateOfTurnHR		= 0x61,
	XCDI_AccelerationHR		= 0x62,

	/* Group Position & Velocity Messages */
	XCDI_LatLong			= 0x71,
	XCDI_AltitudeEllipsoid	= 0x72,
	XCDI_PositionEcef_X		= 0x73,
	XCDI_PositionEcef_Y		= 0x74,
	XCDI_PositionEcef_Z		= 0x75,
	XCDI_Velocity			= 0x76,
	XCDI_Latitude			= 0x77,
	XCDI_Longitude			= 0x78,
	XCDI_GnssSatInfo 		= 0x79,

	XCDI_EndOfGroup, //Keep this entry second to last.
	XCDI_HighestIdentifier, //Keep this entry last. Don't assign IDs with a higher value than this.
};
/*! @} */

typedef enum XsCanDataIdentifier XsCanDataIdentifier;

#endif
