/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _CMTDEF_H_2006_05_01
#define _CMTDEF_H_2006_05_01

#include <mrpt/utils/mrpt_stdint.h>
#include <mrpt/utils/mrpt_macros.h>
#include <time.h>

#ifdef _WIN32
#else
#	include <termios.h>
#	include <string.h>
#	ifndef	B460800
#		undef	B230400
#		define	B230400	0010003
#		define  B460800 0010004
#		define  B921600 0010007
#	endif
// required for gnu c++ compiler due to difference in attribute declarations
#   define __cdecl   // __attribute__((cdecl))   // JLBC @ MRPT: GCC warns about ignored attribute
#   define __stdcall // __attribute__((stdcall)) // JLBC @ MRPT: GCC warns about ignored attribute
#endif

#ifndef _XSENS_STD_H_2006_09_11
#	include "xsens_std.h"
#endif
#if !defined(_XSENS_TIME_2006_09_12) && !defined(_CMT_DLL_IMPORT)
#	include "xsens_time.h"
#endif

#ifndef _CMT_FILE_DEF_H_2007_01_24
#	include "cmtf.h"
#endif

#ifdef _CMT_STATIC_LIB
//namespace xsens {
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Field message indices
#define CMT_IND_PREAMBLE				0
#define CMT_IND_BID						1
#define CMT_IND_MID						2
#define CMT_IND_LEN						3
#define CMT_IND_DATA0					4
#define CMT_IND_LENEXTH					4
#define CMT_IND_LENEXTL					5
#define CMT_IND_DATAEXT0				6

#define CMT_PREAMBLE					0xFA
#define CMT_BID_MASTER					0xFF
#define CMT_BID_MT						0x01
#define CMT_EXTLENCODE					0xFF

#define CMT_LEN_MSGHEADER				4
#define CMT_LEN_MSGEXTHEADER			6
#define CMT_LEN_MSGHEADERCS				5
#define CMT_LEN_MSGEXTHEADERCS			7
#define CMT_LEN_CHECKSUM				1
#define CMT_LEN_UNSIGSHORT				2
#define CMT_LEN_UNSIGINT				4
#define CMT_LEN_FLOAT					4

// Maximum message/data length
#define CMT_MAXDATALEN					8192
#define CMT_MAXSHORTDATALEN				254
#define CMT_MAXMSGLEN					(CMT_MAXDATALEN+7)
#define CMT_MAXSHORTMSGLEN				(CMT_MAXSHORTDATALEN+5)
#define CMT_MAXGARBAGE					(CMT_MAXMSGLEN+1)


// DID Type (high nibble)
#define CMT_DID_TYPEH_MASK				0x00F00000
#define CMT_DID_TYPEH_MT				0x00000000
#define CMT_DID_TYPEH_XM				0x00100000
#define CMT_DID_TYPEH_MTI_MTX			0x00300000
#define CMT_DID_TYPEH_MTIG				0x00500000

// All Message identifiers
// WakeUp state messages
#define CMT_MID_WAKEUP					0x3E
#define CMT_MID_WAKEUPACK				0x3F

// Config state messages
#define CMT_MID_REQDID					0x00
#define CMT_MID_DEVICEID				0x01
#define CMT_LEN_DEVICEID				4
#define CMT_MID_INITBUS					0x02
#define CMT_MID_INITBUSRESULTS			0x03
#define CMT_LEN_INITBUSRESULTS			4
#define CMT_MID_REQPERIOD				0x04
#define CMT_MID_REQPERIODACK			0x05
#define CMT_LEN_PERIOD					2
#define CMT_MID_SETPERIOD				0x04
#define CMT_MID_SETPERIODACK			0x05
// XbusMaster
#define CMT_MID_SETBID					0x06
#define CMT_MID_SETBIDACK				0x07
#define CMT_MID_AUTOSTART				0x06
#define CMT_MID_AUTOSTARTACK			0x07
#define CMT_MID_BUSPWR					0x08
#define CMT_LEN_BUSPWR					2
#define CMT_MID_BUSPWRACK				0x09
// End XbusMaster
#define CMT_MID_REQDATALENGTH			0x0A
#define CMT_MID_DATALENGTH				0x0B
#define CMT_LEN_DATALENGTH				2
#define CMT_MID_REQCONFIGURATION		0x0C
#define CMT_MID_CONFIGURATION			0x0D
#define CMT_LEN_CONFIGURATION			118
#define CMT_MID_RESTOREFACTORYDEF		0x0E
#define CMT_MID_RESTOREFACTORYDEFACK	0x0F

#define CMT_MID_GOTOMEASUREMENT			0x10
#define CMT_MID_GOTOMEASUREMENTACK		0x11
#define CMT_MID_REQFWREV				0x12
#define CMT_MID_FIRMWAREREV				0x13
#define CMT_LEN_FIRMWAREREV				3
// XbusMaster
#define CMT_MID_REQBTDISABLE			0x14
#define CMT_MID_REQBTDISABLEACK			0x15
#define CMT_LEN_BTDISABLE				1
#define CMT_MID_DISABLEBT				0x14
#define CMT_MID_DISABLEBTACK			0x15
#define CMT_MID_REQOPMODE				0x16
#define CMT_MID_REQOPMODEACK			0x17
#define CMT_LEN_OPMODE					1
#define CMT_MID_SETOPMODE				0x16
#define CMT_MID_SETOPMODEACK			0x17
// End XbusMaster
#define CMT_MID_REQBAUDRATE				0x18
#define CMT_MID_REQBAUDRATEACK			0x19
#define CMT_LEN_BAUDRATE				1
#define CMT_MID_SETBAUDRATE				0x18
#define CMT_MID_SETBAUDRATEACK			0x19
// XbusMaster
#define CMT_MID_REQSYNCMODE				0x1A
#define CMT_MID_REQSYNCMODEACK			0x1B
#define CMT_LEN_SYNCMODE				1
#define CMT_MID_SETSYNCMODE				0x1A
#define CMT_MID_SETSYNCMODEACK			0x1B
// End XbusMaster
#define CMT_MID_REQPRODUCTCODE			0x1C
#define CMT_MID_PRODUCTCODE				0x1D
#define CMT_LEN_PRODUCTCODE				20

// XbusMaster
#define CMT_MID_XMPWROFF				0x44
#define CMT_LEN_XMPWROFF				0
// End XbusMaster

#define CMT_MID_REQOUTPUTMODE			0xD0
#define CMT_MID_REQOUTPUTMODEACK		0xD1
#define CMT_LEN_OUTPUTMODE		 		2
#define CMT_MID_SETOUTPUTMODE			0xD0
#define CMT_MID_SETOUTPUTMODEACK		0xD1

#define CMT_MID_REQOUTPUTSETTINGS		0xD2
#define CMT_MID_REQOUTPUTSETTINGSACK	0xD3
#define CMT_LEN_OUTPUTSETTINGS		 	4
#define CMT_MID_SETOUTPUTSETTINGS		0xD2
#define CMT_MID_SETOUTPUTSETTINGSACK	0xD3

#define CMT_MID_REQOUTPUTSKIPFACTOR		0xD4
#define CMT_MID_REQOUTPUTSKIPFACTORACK	0xD5
#define CMT_LEN_OUTPUTSKIPFACTOR		2
#define CMT_MID_SETOUTPUTSKIPFACTOR		0xD4
#define CMT_MID_SETOUTPUTSKIPFACTORACK	0xD5

#define CMT_MID_REQSYNCINSETTINGS		0xD6
#define CMT_MID_REQSYNCINSETTINGSACK	0xD7
#define CMT_LEN_SYNCINMODE				2
#define CMT_LEN_SYNCINSKIPFACTOR		2
#define CMT_LEN_SYNCINOFFSET			4
#define CMT_MID_SETSYNCINSETTINGS		0xD6
#define CMT_MID_SETSYNCINSETTINGSACK	0xD7

#define CMT_MID_REQSYNCOUTSETTINGS		0xD8
#define CMT_MID_REQSYNCOUTSETTINGSACK	0xD9
#define CMT_LEN_SYNCOUTMODE				2
#define CMT_LEN_SYNCOUTSKIPFACTOR		2
#define CMT_LEN_SYNCOUTOFFSET			4
#define CMT_LEN_SYNCOUTPULSEWIDTH		4
#define CMT_MID_SETSYNCOUTSETTINGS		0xD8
#define CMT_MID_SETSYNCOUTSETTINGSACK	0xD9

#define CMT_MID_REQERRORMODE			0xDA
#define CMT_MID_REQERRORMODEACK			0xDB
#define CMT_LEN_ERRORMODE				2
#define CMT_MID_SETERRORMODE			0xDA
#define CMT_MID_SETERRORMODEACK			0xDB

#define CMT_MID_REQTRANSMITDELAY		0xDC
#define CMT_MID_REQTRANSMITDELAYACK		0xDD
#define CMT_LEN_TRANSMITDELAY			2
#define CMT_MID_SETTRANSMITDELAY		0xDC
#define CMT_MID_SETTRANSMITDELAYACK		0xDD

// Xbus Master
#define CMT_MID_REQXMERRORMODE			0x82
#define CMT_MID_REQXMERRORMODEACK		0x83
#define CMT_LEN_XMERRORMODE				2
#define CMT_MID_SETXMERRORMODE			0x82
#define CMT_MID_SETXMERRORMODEACK		0x83

#define CMT_MID_REQBUFFERSIZE			0x84
#define CMT_MID_REQBUFFERSIZEACK		0x85
#define CMT_LEN_BUFFERSIZE				2
#define CMT_MID_SETBUFFERSIZE			0x84
#define CMT_MID_SETBUFFERSIZEACK		0x85
// End Xbus Master

#define CMT_MID_REQHEADING				0x82
#define CMT_MID_REQHEADINGACK			0x83
#define CMT_LEN_HEADING		 			4
#define CMT_MID_SETHEADING				0x82
#define CMT_MID_SETHEADINGACK			0x83

#define CMT_MID_REQMAGNETICDECLINATION		0x6A
#define CMT_MID_REQMAGNETICDECLINATIONACK	0x6B
#define CMT_LEN_MAGNETICDECLINATION		 	4
#define CMT_MID_SETMAGNETICDECLINATION		0x6A
#define CMT_MID_SETMAGNETICDECLINATIONACK	0x6B

#define CMT_MID_REQLOCATIONID			0x84
#define CMT_MID_REQLOCATIONIDACK		0x85
#define CMT_LEN_LOCATIONID				2
#define CMT_MID_SETLOCATIONID			0x84
#define CMT_MID_SETLOCATIONIDACK		0x85

#define CMT_MID_REQEXTOUTPUTMODE		0x86
#define CMT_MID_REQEXTOUTPUTMODEACK		0x87
#define CMT_LEN_EXTOUTPUTMODE			2
#define CMT_MID_SETEXTOUTPUTMODE		0x86
#define CMT_MID_SETEXTOUTPUTMODEACK		0x87

// XbusMaster
#define CMT_MID_REQBATLEVEL				0x88
#define CMT_MID_BATLEVEL				0x89
// End XbusMaster

#define CMT_MID_REQINITTRACKMODE		0x88
#define CMT_MID_REQINITTRACKMODEACK		0x89
#define CMT_LEN_INITTRACKMODE			2
#define CMT_MID_SETINITTRACKMODE		0x88
#define CMT_MID_SETINITTRACKMODEACK		0x89

// obsolete
//#define CMT_MID_STOREFILTERSTATE		0x8A
//#define CMT_LEN_STOREFILTERSTATE		0
//#define CMT_MID_STOREFILTERSTATEACK	0x8B
// new definition
#define CMT_MID_STOREXKFSTATE			0x8A
#define CMT_LEN_STOREXKFSTATE			0
#define CMT_MID_STOREXKFSTATEACK		0x8B

#define CMT_MID_REQUTCTIME				0x60
#define CMT_MID_UTCTIME					0x61
#define CMT_LEN_UTCTIME					12

#define CMT_LEN_SCENARIOLABEL			20
#define CMT_LEN_SCENARIOFULL			(1+1+CMT_LEN_SCENARIOLABEL)
#define	CMT_MID_REQAVAILABLESCENARIOS	0x62
#define CMT_MID_AVAILABLESCENARIOS		0x63
#define CMT_LEN_AVAILABLESCENARIOS		(CMT_MAX_SCENARIOS_IN_MT*CMT_LEN_SCENARIOFULL)

#define CMT_MID_REQSCENARIO				0x64
#define CMT_MID_REQSCENARIOACK			0x65
#define CMT_LEN_REQSCENARIOACK			2
#define CMT_MID_SETSCENARIO				0x64
#define CMT_MID_SETSCENARIOACK			0x65
#define CMT_LEN_SETSCENARIO				1

#define CMT_MID_REQGRAVITYMAGNITUDE		0x66
#define CMT_MID_REQGRAVITYMAGNITUDEACK	0x67
#define CMT_MID_SETGRAVITYMAGNITUDE		0x66
#define CMT_MID_SETGRAVITYMAGNITUDEACK	0x67
#define CMT_LEN_GRAVITYMAGNITUDE		4

#define CMT_MID_REQGPSLEVERARM			0x68
#define CMT_MID_REQGPSLEVERARMACK		0x69
#define CMT_MID_SETGPSLEVERARM			0x68
#define CMT_MID_SETGPSLEVERARMACK		0x69
#define CMT_LEN_GPSLEVERARM				12

// Measurement state
#define CMT_MID_GOTOCONFIG				0x30
#define CMT_MID_GOTOCONFIGACK			0x31
#define CMT_MID_BUSDATA					0x32
#define CMT_MID_MTDATA					0x32

// Manual
#define CMT_MID_PREPAREDATA				0x32
#define CMT_MID_REQDATA					0x34
#define CMT_MID_REQDATAACK				0x35

// MTData defines
// Length of data blocks in bytes
#define CMT_LEN_RAWDATA					20
#define CMT_LEN_CALIBDATA				36
#define CMT_LEN_CALIB_ACCDATA			12
#define CMT_LEN_CALIB_GYRDATA			12
#define CMT_LEN_CALIB_MAGDATA			12
#define CMT_LEN_ORIENT_QUATDATA			16
#define CMT_LEN_ORIENT_EULERDATA		12
#define CMT_LEN_ORIENT_MATRIXDATA		36
#define CMT_LEN_SAMPLECNT				2
#define CMT_LEN_TEMPDATA				4

// Length of data blocks in floats
#define CMT_LEN_CALIBDATA_FLT			9
#define CMT_LEN_ORIENT_QUATDATA_FLT		4
#define CMT_LEN_ORIENT_EULERDATA_FLT	3
#define CMT_LEN_ORIENT_MATRIXDATA_FLT	9

// Indices of fields in DATA field of MTData message in bytes
// use in combination with LEN_CALIB etc
// Un-calibrated raw data
#define CMT_IND_RAW_ACCX				0
#define CMT_IND_RAW_ACCY				2
#define CMT_IND_RAW_ACCZ				4
#define CMT_IND_RAW_GYRX				6
#define CMT_IND_RAW_GYRY				8
#define CMT_IND_RAW_GYRZ				10
#define CMT_IND_RAW_MAGX				12
#define CMT_IND_RAW_MAGY				14
#define CMT_IND_RAW_MAGZ				16
#define CMT_IND_RAW_TEMP				18
// Calibrated data
#define CMT_IND_CALIB_ACCX				0
#define CMT_IND_CALIB_ACCY				4
#define CMT_IND_CALIB_ACCZ				8
#define CMT_IND_CALIB_GYRX				12
#define CMT_IND_CALIB_GYRY				16
#define CMT_IND_CALIB_GYRZ				20
#define CMT_IND_CALIB_MAGX				24
#define CMT_IND_CALIB_MAGY				28
#define CMT_IND_CALIB_MAGZ				32
// Orientation data - quat
#define CMT_IND_ORIENT_Q0				0
#define CMT_IND_ORIENT_Q1				4
#define CMT_IND_ORIENT_Q2				8
#define CMT_IND_ORIENT_Q3				12
// Orientation data - euler
#define CMT_IND_ORIENT_ROLL				0
#define CMT_IND_ORIENT_PITCH			4
#define CMT_IND_ORIENT_YAW				8
// Orientation data - matrix
#define CMT_IND_ORIENT_A				0
#define CMT_IND_ORIENT_B				4
#define CMT_IND_ORIENT_C				8
#define CMT_IND_ORIENT_D				12
#define CMT_IND_ORIENT_E				16
#define CMT_IND_ORIENT_F				20
#define CMT_IND_ORIENT_G				24
#define CMT_IND_ORIENT_H				28
#define CMT_IND_ORIENT_I				32
// Orientation data - euler
#define CMT_IND_SAMPLECNTH				0
#define CMT_IND_SAMPLECNTL				1

// Indices of fields in DATA field of MTData message
// Un-calibrated raw data
#define CMT_FLDNUM_RAW_ACCX				0
#define CMT_FLDNUM_RAW_ACCY				1
#define CMT_FLDNUM_RAW_ACCZ				2
#define CMT_FLDNUM_RAW_GYRX				3
#define CMT_FLDNUM_RAW_GYRY				4
#define CMT_FLDNUM_RAW_GYRZ				5
#define CMT_FLDNUM_RAW_MAGX				6
#define CMT_FLDNUM_RAW_MAGY				7
#define CMT_FLDNUM_RAW_MAGZ				8
#define CMT_FLDNUM_RAW_TEMP				9
// Calibrated data
#define CMT_FLDNUM_CALIB_ACCX			0
#define CMT_FLDNUM_CALIB_ACCY			1
#define CMT_FLDNUM_CALIB_ACCZ			2
#define CMT_FLDNUM_CALIB_GYRX			3
#define CMT_FLDNUM_CALIB_GYRY			4
#define CMT_FLDNUM_CALIB_GYRZ			5
#define CMT_FLDNUM_CALIB_MAGX			6
#define CMT_FLDNUM_CALIB_MAGY			7
#define CMT_FLDNUM_CALIB_MAGZ			8
// Orientation data - quat
#define CMT_FLDNUM_ORIENT_Q0			0
#define CMT_FLDNUM_ORIENT_Q1			1
#define CMT_FLDNUM_ORIENT_Q2			2
#define CMT_FLDNUM_ORIENT_Q3			3
// Orientation data - euler
#define CMT_FLDNUM_ORIENT_ROLL			0
#define CMT_FLDNUM_ORIENT_PITCH			1
#define CMT_FLDNUM_ORIENT_YAW			2
// Orientation data - matrix
#define CMT_FLDNUM_ORIENT_A				0
#define CMT_FLDNUM_ORIENT_B				1
#define CMT_FLDNUM_ORIENT_C				2
#define CMT_FLDNUM_ORIENT_D				3
#define CMT_FLDNUM_ORIENT_E				4
#define CMT_FLDNUM_ORIENT_F				5
#define CMT_FLDNUM_ORIENT_G				6
#define CMT_FLDNUM_ORIENT_H				7
#define CMT_FLDNUM_ORIENT_I				8
// Length
// Uncalibrated raw data
#define CMT_LEN_RAW_ACCX				2
#define CMT_LEN_RAW_ACCY				2
#define CMT_LEN_RAW_ACCZ				2
#define CMT_LEN_RAW_GYRX				2
#define CMT_LEN_RAW_GYRY				2
#define CMT_LEN_RAW_GYRZ				2
#define CMT_LEN_RAW_MAGX				2
#define CMT_LEN_RAW_MAGY				2
#define CMT_LEN_RAW_MAGZ				2
#define CMT_LEN_RAW_TEMP				2
// Calibrated data
#define CMT_LEN_CALIB_ACCX				4
#define CMT_LEN_CALIB_ACCY				4
#define CMT_LEN_CALIB_ACCZ				4
#define CMT_LEN_CALIB_GYRX				4
#define CMT_LEN_CALIB_GYRY				4
#define CMT_LEN_CALIB_GYRZ				4
#define CMT_LEN_CALIB_MAGX				4
#define CMT_LEN_CALIB_MAGY				4
#define CMT_LEN_CALIB_MAGZ				4
// Orientation data - quat
#define CMT_LEN_ORIENT_Q0				4
#define CMT_LEN_ORIENT_Q1				4
#define CMT_LEN_ORIENT_Q2				4
#define CMT_LEN_ORIENT_Q3				4
// Orientation data - euler
#define CMT_LEN_ORIENT_ROLL				4
#define CMT_LEN_ORIENT_PITCH			4
#define CMT_LEN_ORIENT_YAW				4
// Orientation data - matrix
#define CMT_LEN_ORIENT_A				4
#define CMT_LEN_ORIENT_B				4
#define CMT_LEN_ORIENT_C				4
#define CMT_LEN_ORIENT_D				4
#define CMT_LEN_ORIENT_E				4
#define CMT_LEN_ORIENT_F				4
#define CMT_LEN_ORIENT_G				4
#define CMT_LEN_ORIENT_H				4
#define CMT_LEN_ORIENT_I				4

// Defines for getDataValue
#define CMT_VALUE_RAW_ACC				0
#define CMT_VALUE_RAW_GYR				1
#define CMT_VALUE_RAW_MAG				2
#define CMT_VALUE_RAW_TEMP				3
#define CMT_VALUE_CALIB_ACC				4
#define CMT_VALUE_CALIB_GYR				5
#define CMT_VALUE_CALIB_MAG				6
#define CMT_VALUE_ORIENT_QUAT			7
#define CMT_VALUE_ORIENT_EULER			8
#define CMT_VALUE_ORIENT_MATRIX			9
#define CMT_VALUE_SAMPLECNT				10
#define CMT_VALUE_TEMP					11

#define CMT_INVALIDSETTINGVALUE			0xFFFFFFFF


// Valid in all states
#define CMT_MID_RESET					0x40
#define CMT_MID_RESETACK				0x41
#define CMT_MID_ERROR					0x42
//#define CMT_LEN_ERROR					1
// XbusMaster
#define CMT_MID_XMPWROFF				0x44
// End XbusMaster

#define CMT_MID_REQFILTERSETTINGS		0xA0
#define CMT_MID_REQFILTERSETTINGSACK	0xA1
#define CMT_LEN_FILTERSETTINGS			4
#define CMT_MID_SETFILTERSETTINGS		0xA0
#define CMT_MID_SETFILTERSETTINGSACK	0xA1
#define CMT_MID_REQAMD					0xA2
#define CMT_MID_REQAMDACK				0xA3
#define CMT_LEN_AMD						2
#define CMT_MID_SETAMD					0xA2
#define CMT_MID_SETAMDACK				0xA3
#define CMT_MID_RESETORIENTATION		0xA4
#define CMT_MID_RESETORIENTATIONACK		0xA5
#define CMT_LEN_RESETORIENTATION		2

#define CMT_MID_REQGPSSTATUS			0xA6
#define CMT_MID_GPSSTATUS				0xA7
#define CMT_LEN_GPSSTATUS				(1+5*16)

// Baudrate defines for SetBaudrate message
#define CMT_BAUDCODE_9K6				0x09
//	#define CMT_BAUDCODE_14K4				0x08
#define CMT_BAUDCODE_19K2				0x07
//	#define CMT_BAUDCODE_28K8				0x06
#define CMT_BAUDCODE_38K4				0x05
#define CMT_BAUDCODE_57K6				0x04
//	#define CMT_BAUDCODE_76K8				0x03
#define CMT_BAUDCODE_115K2				0x02
#define CMT_BAUDCODE_230K4				0x01
#define CMT_BAUDCODE_460K8				0x00
#define CMT_BAUDCODE_921K6				0x80

// Xbus protocol error codes (Error)
#define CMT_ERROR_NOBUSCOMM				0x01
#define CMT_ERROR_BUSNOTREADY			0x02
#define CMT_ERROR_PERIODINVALID			0x03
#define CMT_ERROR_MESSAGEINVALID		0x04
#define CMT_ERROR_INITOFBUSFAILED1		0x10
#define CMT_ERROR_INITOFBUSFAILED2		0x11
#define CMT_ERROR_INITOFBUSFAILED3		0x12
#define CMT_ERROR_SETBIDPROCFAILED1		0x14
#define CMT_ERROR_SETBIDPROCFAILED2		0x15
#define CMT_ERROR_MEASUREMENTFAILED1	0x18
#define CMT_ERROR_MEASUREMENTFAILED2	0x19
#define CMT_ERROR_MEASUREMENTFAILED3	0x1A
#define CMT_ERROR_MEASUREMENTFAILED4	0x1B
#define CMT_ERROR_MEASUREMENTFAILED5	0x1C
#define CMT_ERROR_MEASUREMENTFAILED6	0x1D
#define CMT_ERROR_TIMEROVERFLOW			0x1E
#define CMT_ERROR_BAUDRATEINVALID		0x20
#define CMT_ERROR_PARAMETERINVALID		0x21
#define CMT_ERROR_MEASUREMENTFAILED7	0x23

// Error modes (SetErrorMode)
#define CMT_ERRORMODE_IGNORE					0x0000
#define CMT_ERRORMODE_INCSAMPLECNT				0x0001
#define CMT_ERRORMODE_INCSAMPLECNT_SENDERROR	0x0002
#define CMT_ERRORMODE_SENDERROR_GOTOCONFIG		0x0003

// Configuration message defines
#define CMT_CONF_MASTERDID				0
#define CMT_CONF_PERIOD					4
#define CMT_CONF_OUTPUTSKIPFACTOR		6
#define CMT_CONF_SYNCIN_MODE			8
#define CMT_CONF_SYNCIN_SKIPFACTOR		10
#define CMT_CONF_SYNCIN_OFFSET			12
#define CMT_CONF_DATE					16
#define CMT_CONF_TIME					24
#define CMT_CONF_NUMDEVICES				96
// Configuration sensor block properties
#define CMT_CONF_DID					98
#define CMT_CONF_DATALENGTH				102
#define CMT_CONF_OUTPUTMODE				104
#define CMT_CONF_OUTPUTSETTINGS			106
#define CMT_CONF_BLOCKLEN				20
// To calculate the offset in data field for output mode of sensor #2 use
//		CONF_OUTPUTMODE + 1*CONF_BLOCKLEN
#define CMT_CONF_MASTERDIDLEN			4
#define CMT_CONF_PERIODLEN				2
#define CMT_CONF_OUTPUTSKIPFACTORLEN	2
#define CMT_CONF_SYNCIN_MODELEN			2
#define CMT_CONF_SYNCIN_SKIPFACTORLEN	2
#define CMT_CONF_SYNCIN_OFFSETLEN		4
#define CMT_CONF_DATELEN				8
#define CMT_CONF_TIMELEN				8
#define CMT_CONF_RESERVED_CLIENTLEN		32
#define CMT_CONF_RESERVED_HOSTLEN		32
#define CMT_CONF_NUMDEVICESLEN			2
// Configuration sensor block properties
#define CMT_CONF_DIDLEN					4
#define CMT_CONF_DATALENGTHLEN			2
#define CMT_CONF_OUTPUTMODELEN			2
#define CMT_CONF_OUTPUTSETTINGSLEN		4

// Clock frequency for offset & pulse width
#define CMT_SYNC_CLOCKFREQMHZ			29.4912
#define CMT_SYNC_CLOCK_NS_TO_TICKS		(CMT_SYNC_CLOCKFREQMHZ * 1.0e-3)
#define CMT_SYNC_CLOCK_TICKS_TO_NS		(1.0e3 / CMT_SYNC_CLOCKFREQMHZ)

// SyncIn params
#define CMT_PARAM_SYNCIN_MODE			0x00
#define CMT_PARAM_SYNCIN_SKIPFACTOR		0x01
#define CMT_PARAM_SYNCIN_OFFSET			0x02

// SyncIn mode
#define CMT_SYNCIN_DISABLED				0x0000
#define CMT_SYNCIN_EDGE_RISING			0x0001
#define CMT_SYNCIN_EDGE_FALLING			0x0002
#define CMT_SYNCIN_EDGE_BOTH			0x0003
#define CMT_SYNCIN_EDGE_MASK			0x0003
#define CMT_SYNCIN_TYPE_DOSAMPLING		0x0000
#define CMT_SYNCIN_TYPE_SENDLASTDATA	0x0004
#define CMT_SYNCIN_TYPE_CLOCK			0x0010
#define CMT_SYNCIN_TYPE_MASK			0x001C

// SyncOut params
#define CMT_PARAM_SYNCOUT_MODE			0x00
#define CMT_PARAM_SYNCOUT_SKIPFACTOR	0x01
#define CMT_PARAM_SYNCOUT_OFFSET		0x02
#define CMT_PARAM_SYNCOUT_PULSEWIDTH	0x03

// SyncOut mode
#define CMT_SYNCOUT_DISABLED		0x0000
#define CMT_SYNCOUT_TYPE_TOGGLE		0x0001
#define CMT_SYNCOUT_TYPE_PULSE		0x0002
#define CMT_SYNCOUT_POL_NEG			0x0000
#define CMT_SYNCOUT_POL_POS			0x0010
#define CMT_SYNCOUT_TYPE_MASK		0x000F
#define CMT_SYNCOUT_POL_MASK		0x0010

// OutputModes
#define CMT_OUTPUTMODE_MT9				0x8000
#define CMT_OUTPUTMODE_XM				0x0000
#define CMT_OUTPUTMODE_RAW				0x4000
#define CMT_OUTPUTMODE_RAWGPSPRINT		0x1000
#define CMT_OUTPUTMODE_TEMP				0x0001
#define CMT_OUTPUTMODE_CALIB			0x0002
#define CMT_OUTPUTMODE_ORIENT			0x0004
#define CMT_OUTPUTMODE_AUXILIARY		0x0008
#define CMT_OUTPUTMODE_POSITION			0x0010
#define CMT_OUTPUTMODE_VELOCITY			0x0020
#define CMT_OUTPUTMODE_STATUS			0x0800


// CmtOutputSettings
#define CMT_OUTPUTSETTINGS_XM						0x00000001
#define CMT_OUTPUTSETTINGS_TIMESTAMP_NONE			0x00000000
#define CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT		0x00000001
#define CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION	0x00000000
#define CMT_OUTPUTSETTINGS_ORIENTMODE_EULER			0x00000004
#define CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX		0x00000008
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG		0x00000000
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACC			0x00000060
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYR			0x00000040
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACCMAG			0x00000020
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYR			0x00000050
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYRMAG			0x00000010
#define CMT_OUTPUTSETTINGS_CALIBMODE_MAG			0x00000030
#define CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT			0x00000000
#define CMT_OUTPUTSETTINGS_DATAFORMAT_F1220			0x00000100
#define CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632		0x00000200
#define CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE		0x00000300
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1		0x00000800
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2		0x00000400
#define CMT_OUTPUTSETTINGS_POSITIONMODE_LLA_WGS84	0x00000000
#define CMT_OUTPUTSETTINGS_VELOCITYMODE_NED			0x00000000
#define CMT_OUTPUTSETTINGS_UNCERTAINTY_ORIENT		0x00100000
#define CMT_OUTPUTSETTINGS_UNCERTAINTY_POS			0x00200000
#define CMT_OUTPUTSETTINGS_UNCERTAINTY_VEL			0x00400000
#define CMT_OUTPUTSETTINGS_TIMESTAMP_MASK			0x00000003
#define CMT_OUTPUTSETTINGS_ORIENTMODE_MASK			0x0000000C
#define CMT_OUTPUTSETTINGS_CALIBMODE_ACC_MASK		0x00000010
#define CMT_OUTPUTSETTINGS_CALIBMODE_GYR_MASK		0x00000020
#define CMT_OUTPUTSETTINGS_CALIBMODE_MAG_MASK		0x00000040
#define CMT_OUTPUTSETTINGS_CALIBMODE_MASK			0x00000070
#define CMT_OUTPUTSETTINGS_DATAFORMAT_MASK			0x00000300
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1_MASK	0x00000400
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2_MASK	0x00000800
#define CMT_OUTPUTSETTINGS_AUXILIARYMODE_MASK		0x00000C00
#define CMT_OUTPUTSETTINGS_POSITIONMODE_MASK		0x0001C000
#define CMT_OUTPUTSETTINGS_VELOCITYMODE_MASK		0x00060000
#define CMT_OUTPUTSETTINGS_UNCERTAINTY_MASK			0x00F00000
#define CMT_OUTPUTSETTINGS_COORDINATES_NED			0x80000000

//#define CMT_OUTPUTMODE_XKF3_ACCG					0x100000000

// Extended (analog) Output Modes
#define CMT_EXTOUTPUTMODE_DISABLED			0x0000
#define CMT_EXTOUTPUTMODE_EULER				0x0001

// Factory Output Mode
#define CMT_FACTORYOUTPUTMODE_DISABLE		0x0000
#define CMT_FACTORYOUTPUTMODE_DEFAULT		0x0001
#define CMT_FACTORYOUTPUTMODE_CUSTOM		0x0002

// Initial tracking mode (SetInitTrackMode)
#define CMT_INITTRACKMODE_DISABLED		0x0000
#define CMT_INITTRACKMODE_ENABLED		0x0001

// Filter settings params
#define CMT_PARAM_FILTER_GAIN			0x00
#define CMT_PARAM_FILTER_RHO			0x01
#define CMT_DONOTSTORE					0x00
#define CMT_STORE						0x01

// AMDSetting (SetAMD)
#define CMT_AMDSETTING_DISABLED			0x0000
#define CMT_AMDSETTING_ENABLED			0x0001

// Send raw string mode
#define CMT_SENDRAWSTRING_INIT			0
#define CMT_SENDRAWSTRING_DEFAULT		1
#define CMT_SENDRAWSTRING_SEND			2

// Timeouts
#define CMT_TO_DEFAULT					500
#define CMT_TO_INIT						250
#define CMT_TO_RETRY					50

#define	CMT_PERIOD_10HZ				11520		// invalid with gps pulse time correction
#define	CMT_PERIOD_12HZ				9600
#define	CMT_PERIOD_15HZ				7680		// invalid with gps pulse time correction
#define	CMT_PERIOD_16HZ				7200
#define	CMT_PERIOD_18HZ				6400		// invalid with gps pulse time correction
#define	CMT_PERIOD_20HZ				5760
#define	CMT_PERIOD_24HZ				4800
#define	CMT_PERIOD_25HZ				4608		// invalid with gps pulse time correction
#define	CMT_PERIOD_30HZ				3840
#define	CMT_PERIOD_32HZ				3600
#define	CMT_PERIOD_36HZ				3200
#define	CMT_PERIOD_40HZ				2880
#define	CMT_PERIOD_45HZ				2560		// invalid with gps pulse time correction
#define	CMT_PERIOD_48HZ				2400
#define	CMT_PERIOD_50HZ				2304		// invalid with gps pulse time correction
#define	CMT_PERIOD_60HZ				1920
#define	CMT_PERIOD_64HZ				1800
#define	CMT_PERIOD_72HZ				1600
#define	CMT_PERIOD_75HZ				1536		// invalid with gps pulse time correction
#define	CMT_PERIOD_80HZ				1440
#define	CMT_PERIOD_90HZ				1280		// invalid with gps pulse time correction
#define	CMT_PERIOD_96HZ				1200

#define	CMT_PERIOD_100HZ			1152
#define	CMT_PERIOD_120HZ			960
#define	CMT_PERIOD_128HZ			900
#define	CMT_PERIOD_144HZ			800
#define	CMT_PERIOD_150HZ			768			// invalid with gps pulse time correction
#define	CMT_PERIOD_160HZ			720
#define	CMT_PERIOD_180HZ			640
#define	CMT_PERIOD_192HZ			600
#define	CMT_PERIOD_200HZ			576
#define	CMT_PERIOD_225HZ			512			// invalid with gps pulse time correction
#define	CMT_PERIOD_240HZ			480
#define	CMT_PERIOD_256HZ			450
#define	CMT_PERIOD_288HZ			400
#define	CMT_PERIOD_300HZ			384
#define	CMT_PERIOD_320HZ			360
#define	CMT_PERIOD_360HZ			320
#define	CMT_PERIOD_384HZ			300
#define	CMT_PERIOD_400HZ			288
#define	CMT_PERIOD_450HZ			256			// invalid with gps pulse time correction
#define	CMT_PERIOD_480HZ			240
#define	CMT_PERIOD_512HZ			225

// openPort baudrates
#ifdef _WIN32
	#define CMT_BAUD_RATE_9600					CBR_9600
//	#define CMT_BAUD_RATE_14K4					CBR_14400
	#define CMT_BAUD_RATE_19K2					CBR_19200
//	#define CMT_BAUD_RATE_28K8					28800
	#define CMT_BAUD_RATE_38K4					CBR_38400
	#define CMT_BAUD_RATE_57K6					CBR_57600
	#define CMT_BAUD_RATE_115K2					CBR_115200
	#define CMT_BAUD_RATE_230K4					230400
	#define CMT_BAUD_RATE_460K8					460800
	#define CMT_BAUD_RATE_921K6					921600
#else
	#define CMT_BAUD_RATE_9600					B9600
//	#define CMT_BAUD_RATE_14K4					B14400
	#define CMT_BAUD_RATE_19K2					B19200
//	#define CMT_BAUD_RATE_28K8					B28800
	#define CMT_BAUD_RATE_38K4					B38400
	#define CMT_BAUD_RATE_57K6					B57600
	#define CMT_BAUD_RATE_115K2					B115200
	#define CMT_BAUD_RATE_230K4					B230400
	#define CMT_BAUD_RATE_460K8					B460800
	#define CMT_BAUD_RATE_921K6					B921600
#endif

#define CMT_DEFAULT_OUTPUT_MODE			CMT_OUTPUTMODE_ORIENT
#define CMT_DEFAULT_OUTPUT_SETTINGS		(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION |\
										 CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT)

#define CMT_GOTO_CONFIG_TRIES			3	// 50

#define CMT_MAX_DEVICES_PER_PORT		11
#define CMT_DEFAULT_SAMPLE_FREQUENCY	100
#define CMT_DEFAULT_PERIOD				1152
#define CMT_DEFAULT_SKIP				0
#define CMT_DEFAULT_FILTER_GAIN			1.0f
#define CMT_DEFAULT_FILTER_WEIGHTING	1.0f
#define CMT_SYNCOUT_DEFAULT_PULSE_WIDTH	1000000		// 1ms = 1M ns


#define CMT_MAX_PORTS_PER_THREAD	20
#define CMT_MAX_FILES_PER_THREAD	20
#if CMT_MAX_PORTS_PER_THREAD >= CMT_MAX_FILES_PER_THREAD
#	define CMT_MAX_DEVICES			(CMT_MAX_PORTS_PER_THREAD * CMT_MAX_DEVICES_PER_PORT)
#else
#	define CMT_MAX_DEVICES			(CMT_MAX_FILES_PER_THREAD * CMT_MAX_DEVICES_PER_PORT)
#endif
#define CMT_MAX_PACKETS_PER_BUNDLE	CMT_MAX_PORTS_PER_THREAD
#define CMT_BUNDLE_QUEUE_SIZE		256
#define CMT_MAX_ITEMS_IN_PACKET		10

	//! The default size of the serial read buffer in bytes
#define CMT_DEFAULT_READ_BUFFER_SIZE		(CMT_MAXDATALEN*8)
	//! The default size of the serial write buffer in bytes
#define CMT_DEFAULT_WRITE_BUFFER_SIZE		CMT_MAXDATALEN
	//! The default baud rate of the Cmt1s serial communication
#define CMT_DEFAULT_BAUD_RATE				CMT_BAUD_RATE_115K2
	//! The size of the L2 data-message fifo-queue
#define CMT2_DATA_QUEUE_SIZE				60

	//! The timeout value for "goto config"-message acknowledgement
#define CMT3_CONFIG_TIMEOUT				100
	//! The default timeout value for blocking CMT1s operations in ms
#define CMT1_DEFAULT_TIMEOUT			10
	//! Timeout in ms for level 2
#define CMT2_DEFAULT_TIMEOUT			50
	//! The default timeout value for L3 data reading
#define CMT3_DEFAULT_TIMEOUT_MEAS		16
	//! The default timeout value for L3 configuration settings
#define CMT3_DEFAULT_TIMEOUT_CONF		3000
	//! The default timeout value for L4 data reading at L2 and 3
#define CMT4_DEFAULT_TIMEOUT_MEAS		1
	//! The default timeout value for L4 configuration settings
#define CMT4_DEFAULT_TIMEOUT_CONF		CMT3_DEFAULT_TIMEOUT_CONF
	//! The timeout to use for requests during measurement mode
#define CMT4_MEASUREMENT_REQ_TIMEOUT	100
	//! The standard timeout to use for data receipt in measurement mode
#define CMT4_DEFAULT_TIMEOUT_DATA		3000

enum CmtControlLine {
	CMT_CONTROL_DCD		= 0x0001,		// pin 1: Carrier Detect
	CMT_CONTROL_RD		= 0x0002,		// pin 2: Received Data
	CMT_CONTROL_TD		= 0x0004,		// pin 3: Transmitted Data
	CMT_CONTROL_DTR		= 0x0008,		// pin 4: Data Terminal Ready
	CMT_CONTROL_GND		= 0x0010,		// pin 5: Common Ground
	CMT_CONTROL_DSR		= 0x0020,		// pin 6: Data Set Ready
	CMT_CONTROL_RTS		= 0x0040,		// pin 7: Request To Send
	CMT_CONTROL_CTS		= 0x0080,		// pin 8: Clear To Send
	CMT_CONTROL_RI		= 0x0100		// pin 9: Ring Indicator
};

// Reset orientation message type
enum CmtResetMethod {
	CMT_RESETORIENTATION_STORE = 0,
	CMT_RESETORIENTATION_HEADING,
	CMT_RESETORIENTATION_GLOBAL,
	CMT_RESETORIENTATION_OBJECT,
	CMT_RESETORIENTATION_ALIGN
};

enum CmtXmSyncMode {
	CMT_XM_SYNC_OFF				= 0x00,
	CMT_XM_SYNC_PWM				= 0x01,
	CMT_XM_SYNC_MASTER			= 0x10,
	CMT_XM_SYNC_SLAVE			= 0x20,
	CMT_XM_SYNC_TOGGLE			= 0x40
};

//! The type of a Device Id
typedef uint32_t	CmtDeviceId;

/* different alignment commands for gcc / MSVS, the structure needs to be 2-byte aligned
	because the deviceId field is 4 bytes long on offset 98.
*/
#ifdef _MSC_VER
	#pragma pack(push, 2)
#endif
//! Structure containing a full device configuration as returned by the ReqConfig message.
struct CmtDeviceConfiguration {
	uint32_t	m_masterDeviceId;
	uint16_t	m_samplingPeriod;
	uint16_t	m_outputSkipFactor;
	uint16_t	m_syncinMode;
	uint16_t	m_syncinSkipFactor;
	uint32_t	m_syncinOffset;
	uint8_t	m_date[8];
	uint8_t	m_time[8];
	uint8_t	m_reservedForHost[32];
	uint8_t	m_reservedForClient[32];
	uint16_t	m_numberOfDevices;
	struct _devInfo {
		uint32_t	m_deviceId;
		uint16_t	m_dataLength;
		uint16_t	m_outputMode;
		uint32_t	m_outputSettings;
		uint8_t	m_reserved[8];
	} m_deviceInfo[CMT_MAX_DEVICES_PER_PORT];

#ifndef _CMT_DLL_IMPORT
	void readFromMessage(const void* message);
#endif
}
// different alignment commands for gcc / MSVS
#ifdef _MSC_VER
	;
	#pragma pack(pop)
#else
	/*! \cond NODOXYGEN */	__attribute__((__packed__))	/*! \endcond */;
#endif

//////////////////////////////////////////////////////////////////////////////////////////

	//! An output mode bit-field
typedef uint32_t CmtOutputMode;
	//! An output settings bit-field
typedef uint64_t CmtOutputSettings;

	//! A structure for storing data formats
struct CmtDataFormat {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;

#ifndef _CMT_DLL_IMPORT
	//! default constructor, initializes to the given (default) MT settings
	CmtDataFormat(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
					const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS)
					: m_outputMode(mode), m_outputSettings(settings) {}
#endif
};

	//! An MT timestamp (sample count)
typedef uint16_t CmtMtTimeStamp;

#define CMT_MAX_FILENAME_LENGTH	512

#define CMT_DID_BROADCAST				0x80000000
#define CMT_DID_MASTER					0

#ifdef _CMT_ADD_PRIVATE
#	include "cmtprivate.h"
#endif

//! A structure for storing the firmware version
struct CmtVersion {
	uint8_t m_major;
	uint8_t m_minor;
	uint8_t m_revision;
};

//! A structure for storing sync in settings
struct CmtSyncInSettings {
	uint16_t	m_mode;
	uint16_t	m_skipFactor;
	uint32_t	m_offset;		//!< Offset in ns

#ifndef _CMT_DLL_IMPORT
	//! default constructor, initializes to the given (default) MT settings
	CmtSyncInSettings(	const uint16_t mode = 0,
						const uint16_t skip = 0,
						const uint32_t offset = 0)
						: m_mode(mode), m_skipFactor(skip), m_offset(offset) {}
#endif
};

//! A structure for storing sync out settings
struct CmtSyncOutSettings {
	uint16_t	m_mode;
	uint16_t	m_skipFactor;
	uint32_t	m_offset;		//!< Offset in ns
	uint32_t	m_pulseWidth;	//!< Pulse width in ns

#ifndef _CMT_DLL_IMPORT
	//! default constructor, initializes to the given (default) MT settings
	CmtSyncOutSettings( const uint16_t mode = 0,
						const uint16_t skip = 0,
						const uint32_t offset = 0,
						const uint32_t width = CMT_SYNCOUT_DEFAULT_PULSE_WIDTH) :
						m_mode(mode), m_skipFactor(skip), m_offset(offset),
						m_pulseWidth(width) {}
#endif
};

//! A structure for storing UTC Time values
struct CmtUtcTime
{
	uint32_t	m_nano;
	uint16_t	m_year;
	uint8_t	m_month;
	uint8_t	m_day;
	uint8_t	m_hour;
	uint8_t	m_minute;
	uint8_t	m_second;
	uint8_t	m_valid;	//!< When set to 1, the time is valid, when set to 2, the time part is valid, but the date may not be valid. when set to 0, the time is invalid and should be ignored.
};

//! A structure for storing device modes
struct CmtDeviceMode {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;
	uint16_t		m_sampleFrequency;

#ifndef _CMT_DLL_IMPORT
	//! default constructor, initializes to the given (default) MT settings
	CmtDeviceMode(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
					const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS,
					const uint16_t frequency = CMT_DEFAULT_SAMPLE_FREQUENCY) :
						m_outputMode(mode), m_outputSettings(settings),
						m_sampleFrequency(frequency) {}

	/*! \brief Compute the period and skip factor.

		This function computes the period and skipFactor fields from the stored
		m_sampleFrequency field. The maximum error in the frequency is approximately
		0.4%, which occurs at 510Hz (= actually 512 Hz). In general, the higher
		frequencies are harder to set up exactly.
	*/
	void getPeriodAndSkipFactor(uint16_t& period, uint16_t& skip) const;
	/*! \brief Return the real sample frequency in Hz.

		This may be up to 2Hz different from the value that is set.
	*/
	double getRealSampleFrequency(void) const;
	/*! \brief Compute the sample frequency from a period and skip factor.

		This function does the reverse of the getPeriodAndSkipFactor function, storing the
		value in the m_sampleFrequency field.
	*/
	void setPeriodAndSkipFactor(uint16_t period, uint16_t skip);

	//! Check if all fields of the two structures are equal
	bool operator == (const CmtDeviceMode& dev) const;
#endif
};

//! A structure for storing device modes using period and skip factor (new default)
struct CmtDeviceMode2 {
	CmtOutputMode		m_outputMode;
	CmtOutputSettings	m_outputSettings;
	uint16_t		m_period;
	uint16_t		m_skip;

#ifndef _CMT_DLL_IMPORT
	//! default constructor, initializes to the given (default) MT settings
	CmtDeviceMode2(	const CmtOutputMode mode = CMT_DEFAULT_OUTPUT_MODE,
		const CmtOutputSettings settings = CMT_DEFAULT_OUTPUT_SETTINGS,
		const uint16_t period = CMT_DEFAULT_PERIOD,
		const uint16_t skip = CMT_DEFAULT_SKIP) :
	m_outputMode(mode), m_outputSettings(settings),
		m_period(period), m_skip(skip) {}

	/*! \brief Return the real sample frequency in Hz.

	This may be up to 2Hz different from the value that is set.
	*/
	double getRealSampleFrequency(void) const;
	/*! \brief Return the sample frequency in Hz.

	This may be up to 2Hz different from the value that is set.
	*/
	uint16_t getSampleFrequency(void) const;
	/*! \brief Compute the period and skip factor from a sample frequency.

	This function does the reverse of the getPeriodAndSkipFactor function, storing the
	value in the m_period and m_skip field.
	*/
	void setSampleFrequency(uint16_t freq);

	//! Check if all fields of the two structures are equal
	bool operator == (const CmtDeviceMode2& dev) const;
#endif
};

#define CMT_MAX_SCENARIOS_IN_MT		5
#define CMT_MAX_SCENARIOS			254

//! A structure for storing scenario information
struct CmtScenario {
	uint8_t m_type;		//!< The type of the scenario. When set to 255 in an operation, the 'current' scenario is used.
	uint8_t m_version;	//!< The version of the scenario.
	char m_label[CMT_LEN_SCENARIOLABEL+1];		//!< The label of the scenario.
	//bool m_inSensor;							//!< When set to true, the scenario is available in the sensor
	char m_filterType;							//!< The type of filter this scenario is intended for '3': XKF-3, '6': XKF-6. \note The value is a character, so XKF-3 is '3', which is hex 0x33
};

#define CMT_MAX_OBJECTS		20

enum CmtCallbackSelector {
	CMT_CALLBACK_ONMEASUREMENTPREPARE	= 0,	//!< Callback function, called right before sending a GotoMeasurement message
	CMT_CALLBACK_ONMEASUREMENTSTART		= 1,	//!< Callback function, called right after successfully switching to Measurement mode
	CMT_CALLBACK_ONMEASUREMENTSTOP		= 2,	//!< Callback function, called right before switching from Measurement mode to Config mode
	CMT_CALLBACK_ONPOSTPROCESS			= 3,	//!< Callback function, called when a full data bundle is available and has been processed by the CMT. The first void* parameter supplied to this function can be handed as the bundle parameter in cmtData... functions to manipulate the newly received bundle.
	CMT_CALLBACK_ONBYTESRECEIVED		= 4,	//!< Callback function, called when bytes have been read from a port
	CMT_CALLBACK_ONMESSAGERECEIVED		= 5,	//!< Callback function, called when a full message has been received from a port
	CMT_CALLBACK_ONMESSAGESENT			= 6		//!< Callback function, called when a full message has been sent by a port
};

enum CmtQueueMode {
	CMT_QM_FIFO	= 0,
	CMT_QM_LAST = 1,
	CMT_QM_RAW  = 2
};

struct CmtBinaryData {
	int32_t m_size;
	uint8_t m_data[CMT_MAXMSGLEN];
	uint8_t m_portNr;
};

//#ifdef _WIN32
	typedef XsensResultValue (__cdecl * CmtCallbackFunction)(int32_t, CmtCallbackSelector, void*, void*);
//#else
//    typedef __attribute__((cdecl)) XsensResultValue (* CmtCallbackFunction)(int32_t, CmtCallbackSelector, void*, void*);
//#endif

//! \brief Structure for storing information about a serial port
struct CmtPortInfo {
	uint32_t m_baudrate;	//!< The baudrate at which an Xsens device was detected
	uint32_t m_deviceId;	//!< The device Id of the detected Xsens device
	uint8_t m_portNr;		//!< The port number
	char m_portName[32];		//!< The port name

	CmtPortInfo() : m_baudrate(0), m_deviceId(0), m_portNr(0)
	{ }
#ifdef __cplusplus
#	ifdef _WIN32
		//! greater than operator, used for sorting the list
	bool operator > (const CmtPortInfo& p) const { return m_portNr > p.m_portNr; }
		//! less than operator, used for sorting the list
	bool operator < (const CmtPortInfo& p) const { return m_portNr < p.m_portNr; }
		//! equality operator, used for finding items in a list
	bool operator == (const CmtPortInfo& p) const { return m_portNr == p.m_portNr; }
		//! equality operator, used for finding items in a list
	bool operator == (const uint8_t port) const { return m_portNr == port; }
#	else	// Linux
		//! greater than operator, used for sorting the list
	bool operator > (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) > 0; }
		//! less than operator, used for sorting the list
	bool operator < (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) < 0; }
		//! equality operator, used for finding items in a list
	bool operator == (const CmtPortInfo& p) const { return strcmp(m_portName, p.m_portName) == 0; }
		//! equality operator, used for finding items in a list
	bool operator == (const uint8_t port) const {
		MRPT_UNUSED_PARAM(port);
		return false;
	}
#	endif // ifdef _WIN32/Linux
#endif
};

struct CmtShortVector {
	uint16_t m_data[3];
};
struct CmtRawData {
	CmtShortVector	m_acc,m_gyr,m_mag;
	uint16_t	m_temp;
};
struct CmtRawGpsData {
	uint16_t	m_pressure;
	uint8_t		m_pressureAge;
	uint32_t	m_itow;
	int32_t		m_latitude;
	int32_t		m_longitude;
	int32_t		m_height;
	int32_t		m_veln;
	int32_t		m_vele;
	int32_t		m_veld;
	uint32_t	m_hacc;
	uint32_t	m_vacc;
	uint32_t	m_sacc;
	uint8_t	m_gpsAge;
};
struct CmtAnalogInData
{
	CmtAnalogInData() : m_data(0) { }

	uint16_t m_data;
};
struct CmtVector {
	double m_data[3];
};
struct CmtCalData {
	CmtVector	m_acc,m_gyr,m_mag;
};
struct CmtQuat {
	double m_data[4];
};
struct CmtEuler {
	double m_roll;		//!< The roll (rotation around x-axis / back-front-line)
	double m_pitch;		//!< The pitch (rotation around y-axis / right-left-line)
	double m_yaw;		//!< The yaw (rotation around z-axis / down-up-line)
};
struct CmtMatrix {
	double m_data[3][3];
};

#define CMT_MAX_SVINFO		16
struct CmtGpsSatelliteInfo {
	uint8_t m_id;
	uint8_t m_navigationStatus;
	uint8_t m_signalQuality;
	uint8_t m_signalStrength;
};
struct CmtGpsStatus {
	CmtGpsSatelliteInfo m_svInfo[CMT_MAX_SVINFO];
};

typedef uint64_t CmtTimeStamp;

#define CMT_AUTO_SAVE_FRAMES	5000
#define CMT_FILE_LAST_FRAME		0xFFFFFFFF

#define CMT_BID_BROADCAST		(const uint8_t)0x00
#define CMT_BID_INVALID			(const uint8_t)0xFE
#define CMT_MID_REQEMTS			(const uint8_t)0x90
#define CMT_MID_EMTSDATA		(const uint8_t)0x91
#define CMT_EMTS_SIZE			1056

#endif
