
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

/*! \file
	\brief	Macros and types for use in the Xsens communication protocol and Xsens Device API classes

	This file contains useful macros for the Xsens Device API classes. These include
	Message Ids, sizes of messages, result/error codes, etc.

	The macros and enumerators in this file are either used on multiple levels (XsResultValue)
	or so numerous (XMID_...) that it would not be justifiable to put them in a header
	file of a particular level.
*/

#ifndef XSDEF_H
#define XSDEF_H

#include "xscontrollerconfig.h"

//////////////////////////////////////////////////////////////////////////////////////////
// Field message indices
#define XS_IND_PREAMBLE				0
#define XS_IND_BID						1
#define XS_IND_MID						2
#define XS_IND_LEN						3
#define XS_IND_DATA0					4
#define XS_IND_LENEXTH					4
#define XS_IND_LENEXTL					5
#define XS_IND_DATAEXT0				6

#define XS_SELFTEST_OK					0x1FF

// message data lengths
#define XS_LEN_TRANSPORTMODE			1
#define XS_LEN_DEVICEID				4
#define XS_LEN_INITBUSRESULTS			4
#define XS_LEN_PERIOD					2
#define XS_LEN_BUSPWR					2
#define XS_LEN_DATALENGTH				2
#define XS_LEN_CONFIGURATION			118
#define XS_LEN_FIRMWAREREV				3
#define XS_LEN_BTDISABLE				1
#define XS_LEN_OPMODE					1
#define XS_LEN_BAUDRATE				1
#define XS_LEN_SYNCMODE				1
#define XS_LEN_PRODUCTCODE				20
#define XS_LEN_PROCESSINGFLAGS			2
#define XS_LEN_XMPWROFF				0
#define XS_LEN_OUTPUTMODE		 		2
#define XS_LEN_OUTPUTSETTINGS		 	4
#define XS_LEN_OUTPUTSKIPFACTOR		2
#define XS_LEN_SYNCINMODE				2
#define XS_LEN_SYNCINSKIPFACTOR		2
#define XS_LEN_SYNCINOFFSET			4
#define XS_LEN_SYNCOUTMODE				2
#define XS_LEN_SYNCOUTSKIPFACTOR		2
#define XS_LEN_SYNCOUTOFFSET			4
#define XS_LEN_SYNCOUTPULSEWIDTH		4
#define XS_LEN_ERRORMODE				2
#define XS_LEN_TRANSMITDELAY			2
#define XS_LEN_OBJECTALIGNMENT			36
#define XS_LEN_ALIGNMENTROTATION		(4*4)
#define XS_LEN_XMERRORMODE				2
#define XS_LEN_BUFFERSIZE				2
#define XS_LEN_HEADING		 			4
#define XS_LEN_MAGNETICFIELD		 	12
#define XS_LEN_LOCATIONID				2
#define XS_LEN_EXTOUTPUTMODE			2
#define XS_LEN_INITTRACKMODE			2
#define XS_LEN_STOREFILTERSTATE			0
#define XS_LEN_UTCTIME					12
#define XS_LEN_FILTERPROFILELABEL		20
#define XS_LEN_FILTERPROFILEFULL		(1+1+XS_LEN_FILTERPROFILELABEL)
#define XS_LEN_AVAILABLEFILTERPROFILES	(XS_MAX_FILTERPROFILES_IN_MT*XS_LEN_FILTERPROFILEFULL)
#define XS_LEN_REQFILTERPROFILEACK		2
#define XS_LEN_SETFILTERPROFILE			2
#define XS_LEN_GRAVITYMAGNITUDE		4
#define XS_LEN_GPSLEVERARM				12
#define XS_LEN_LATLONALT				18
#define XS_LEN_SETNOROTATION			2
#define XS_LEN_FILTERSETTINGS			4
#define XS_LEN_AMD						2
#define XS_LEN_RESETORIENTATION		2
#define XS_LEN_GPSSTATUS				(1+5*16)
#define XS_LEN_CLIENTUSAGE				1
#define XS_LEN_CLIENTPRIORITY			1
#define XS_LEN_WIRELESSCONFIG			4
#define XS_LEN_INFOREQUEST				1
#define XS_LEN_SETOUTPUTTRIGGER		10
#define XS_LEN_SETINPUTTRIGGER			10

// MTData defines
// Length of data blocks in bytes
#define XS_LEN_RAWDATA					20
#define XS_LEN_CALIBDATA				36
#define XS_LEN_CALIB_ACCDATA			12
#define XS_LEN_CALIB_GYRDATA			12
#define XS_LEN_CALIB_MAGDATA			12
#define XS_LEN_ORIENT_QUATDATA			16
#define XS_LEN_ORIENT_EULERDATA		12
#define XS_LEN_ORIENT_MATRIXSTA		36
#define XS_LEN_SAMPLECNT				2
#define XS_LEN_TEMPDATA				4

// Length of data blocks in floats
#define XS_LEN_CALIBDATA_FLT			9
#define XS_LEN_TEMPDATA_FLT			1
#define XS_LEN_ORIENT_QUATDATA_FLT		4
#define XS_LEN_ORIENT_EULERDATA_FLT	3
#define XS_LEN_ORIENT_MATRIXSTA_FLT	9

#define XS_INVALIDSETTINGVALUE			0xFFFFFFFF



// Configuration message defines
#define XS_CONF_MASTERDID				0
#define XS_CONF_PERIOD					4
#define XS_CONF_OUTPUTSKIPFACTOR		6
#define XS_CONF_SYNCIN_MODE			8
#define XS_CONF_SYNCIN_SKIPFACTOR		10
#define XS_CONF_SYNCIN_OFFSET			12
#define XS_CONF_DATE					16
#define XS_CONF_TIME					24
#define XS_CONF_NUMDEVICES				96
// Configuration sensor block properties
#define XS_CONF_DID					98
#define XS_CONF_DATALENGTH				102
#define XS_CONF_OUTPUTMODE				104
#define XS_CONF_OUTPUTSETTINGS			106
#define XS_CONF_BLOCKLEN				20
// To calculate the offset in data field for output mode of sensor #2 use
//		CONF_OUTPUTMODE + 1*CONF_BLOCKLEN
#define XS_CONF_MASTERDIDLEN			4
#define XS_CONF_PERIODLEN				2
#define XS_CONF_OUTPUTSKIPFACTORLEN	2
#define XS_CONF_SYNCIN_MODELEN			2
#define XS_CONF_SYNCIN_SKIPFACTORLEN	2
#define XS_CONF_SYNCIN_OFFSETLEN		4
#define XS_CONF_DATELEN				8
#define XS_CONF_TIMELEN				8
#define XS_CONF_RESERVED_CLIENTLEN		32
#define XS_CONF_RESERVED_HOSTLEN		32
#define XS_CONF_NUMDEVICESLEN			2
// Configuration sensor block properties
#define XS_CONF_DIDLEN					4
#define XS_CONF_DATALENGTHLEN			2
#define XS_CONF_OUTPUTMODELEN			2
#define XS_CONF_OUTPUTSETTINGSLEN		4

// Clock frequency for offset & pulse width
#define XS_SYNC_CLOCKFREQMHZ			29.4912
#define XS_SYNC_CLOCK_NS_TO_TICKS		(XS_SYNC_CLOCKFREQMHZ * 1.0e-3)
#define XS_SYNC_CLOCK_TICKS_TO_NS		(1.0e3 / XS_SYNC_CLOCKFREQMHZ)
#define XS_SYNC_CLOCK_US_TO_TICKS		(XS_SYNC_CLOCKFREQMHZ * 1.0)
#define XS_SYNC_CLOCK_TICKS_TO_US		(1.0 / XS_SYNC_CLOCKFREQMHZ)

// SyncIn params
#define XS_PARAM_SYNCIN_MODE			0x00
#define XS_PARAM_SYNCIN_SKIPFACTOR		0x01
#define XS_PARAM_SYNCIN_OFFSET			0x02

// SyncIn mode
#define XS_SYNCIN_DISABLED				0x0000
#define XS_SYNCIN_EDGE_RISING			0x0001
#define XS_SYNCIN_EDGE_FALLING			0x0002
#define XS_SYNCIN_EDGE_BOTH			0x0003
#define XS_SYNCIN_EDGE_MASK			0x0003
#define XS_SYNCIN_TYPE_DOSAMPLING		0x0000
#define XS_SYNCIN_TYPE_SENDLASTDATA	0x0004
#define XS_SYNCIN_TYPE_CLOCK			0x0010
#define XS_SYNCIN_TYPE_MASK			0x001C

// SyncOut params
#define XS_PARAM_SYNCOUT_MODE			0x00
#define XS_PARAM_SYNCOUT_SKIPFACTOR	0x01
#define XS_PARAM_SYNCOUT_OFFSET		0x02
#define XS_PARAM_SYNCOUT_PULSEWIDTH	0x03

// SyncOut mode
#define XS_SYNCOUT_DISABLED		0x0000
#define XS_SYNCOUT_TYPE_TOGGLE		0x0001
#define XS_SYNCOUT_TYPE_PULSE		0x0002
#define XS_SYNCOUT_POL_NEG			0x0000
#define XS_SYNCOUT_POL_POS			0x0010
#define XS_SYNCOUT_TYPE_MASK		0x000F
#define XS_SYNCOUT_POL_MASK		0x0010

// Initial tracking mode (SetInitTrackMode)
#define XS_INITTRACKMODE_DISABLED		0x0000
#define XS_INITTRACKMODE_ENABLED		0x0001

// Filter settings params
#define XS_PARAM_FILTER_GAIN			0x00
#define XS_PARAM_FILTER_RHO			0x01
#define XS_DONOTSTORE					0x00
#define XS_STORE						0x01

// AMDSetting (SetAMD)
#define XS_AMDSETTING_DISABLED			0x0000
#define XS_AMDSETTING_ENABLED			0x0001

#define XS_PARAM_ROTSENSOR				0x00
#define XS_PARAM_ROTLOCAL				0x01

// Send raw string mode
#define XS_SENDRAWSTRING_INIT			0
#define XS_SENDRAWSTRING_DEFAULT		1
#define XS_SENDRAWSTRING_SEND			2

// Timeouts
#define XS_TO_DEFAULT					500
#define XS_TO_INIT						250
#define XS_TO_RETRY					50

#define	XS_PERIOD_10HZ				11520		// invalid with gps pulse time correction
#define	XS_PERIOD_12HZ				9600
#define	XS_PERIOD_15HZ				7680		// invalid with gps pulse time correction
#define	XS_PERIOD_16HZ				7200
#define	XS_PERIOD_18HZ				6400		// invalid with gps pulse time correction
#define	XS_PERIOD_20HZ				5760
#define	XS_PERIOD_24HZ				4800
#define	XS_PERIOD_25HZ				4608		// invalid with gps pulse time correction
#define	XS_PERIOD_30HZ				3840
#define	XS_PERIOD_32HZ				3600
#define	XS_PERIOD_36HZ				3200
#define	XS_PERIOD_40HZ				2880
#define	XS_PERIOD_45HZ				2560		// invalid with gps pulse time correction
#define	XS_PERIOD_48HZ				2400
#define	XS_PERIOD_50HZ				2304		// invalid with gps pulse time correction
#define	XS_PERIOD_60HZ				1920
#define	XS_PERIOD_64HZ				1800
#define	XS_PERIOD_72HZ				1600
#define	XS_PERIOD_75HZ				1536		// invalid with gps pulse time correction
#define	XS_PERIOD_80HZ				1440
#define	XS_PERIOD_90HZ				1280		// invalid with gps pulse time correction
#define	XS_PERIOD_96HZ				1200

#define	XS_PERIOD_100HZ			1152
#define	XS_PERIOD_120HZ			960
#define	XS_PERIOD_128HZ			900
#define	XS_PERIOD_144HZ			800
#define	XS_PERIOD_150HZ			768			// invalid with gps pulse time correction
#define	XS_PERIOD_160HZ			720
#define	XS_PERIOD_180HZ			640
#define	XS_PERIOD_192HZ			600
#define	XS_PERIOD_200HZ			576
#define	XS_PERIOD_225HZ			512			// invalid with gps pulse time correction
#define	XS_PERIOD_240HZ			480
#define	XS_PERIOD_256HZ			450
#define	XS_PERIOD_288HZ			400
#define	XS_PERIOD_300HZ			384
#define	XS_PERIOD_320HZ			360
#define	XS_PERIOD_360HZ			320
#define	XS_PERIOD_384HZ			300
#define	XS_PERIOD_400HZ			288
#define	XS_PERIOD_450HZ			256			// invalid with gps pulse time correction
#define	XS_PERIOD_480HZ			240
#define	XS_PERIOD_512HZ			225

#ifndef NOT_FOR_PUBLIC_RELEASE
#define	XS_PERIOD_576HZ			200
#define	XS_PERIOD_600HZ			192
#define	XS_PERIOD_640HZ			180
#define	XS_PERIOD_720HZ			160
#define	XS_PERIOD_768HZ			150
#define	XS_PERIOD_800HZ			144
#define	XS_PERIOD_900HZ			128
#define	XS_PERIOD_960HZ			120
#define	XS_PERIOD_1152HZ			100
#define	XS_PERIOD_1200HZ			96
#define	XS_PERIOD_1280HZ			90
#define	XS_PERIOD_1440HZ			80
#define	XS_PERIOD_1536HZ			75
#define	XS_PERIOD_1600HZ			72
#define	XS_PERIOD_1800HZ			64
#define	XS_PERIOD_1920HZ			60
#define	XS_PERIOD_2304HZ			50
#define	XS_PERIOD_2400HZ			48
#define	XS_PERIOD_2560HZ			45
#define	XS_PERIOD_2880HZ			40
#define	XS_PERIOD_3200HZ			36
#define	XS_PERIOD_3600HZ			32
#define	XS_PERIOD_3840HZ			30
#define	XS_PERIOD_4608HZ			25
#define	XS_PERIOD_4800HZ			24
#define	XS_PERIOD_5760HZ			20
#define	XS_PERIOD_6400HZ			18
#define	XS_PERIOD_7200HZ			16
#define	XS_PERIOD_7680HZ			15
#define	XS_PERIOD_9600HZ			12
#define	XS_PERIOD_11520HZ			10
#endif

/*! \cond XS_INTERNAL */

int XDA_DLL_API xsScaleBatteryLevel(int batteryLevel);

#define XS_AUTO_SAVE_FRAMES	5000
#define XS_FILE_LAST_FRAME	0xFFFFFFFF

#define XEMTS_SIZE			1320

#define XS_MAX_VPORTNAME_LEN	32

/*! \endcond */ // XS_INTERNAL
#endif	// file guard
