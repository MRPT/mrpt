/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*! \cond NODOXYGEN */

#ifndef _XSENS_STD_H_2006_09_11
#define _XSENS_STD_H_2006_09_11

//////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WIN32) && !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x0502
#endif

#ifdef _WIN32
#include <windows.h>
#endif

/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////
//! Xsens return values
// NOTE!!! When adding a value to this list, also add its description in
// xsens_std.cpp
typedef enum /*! \cond NODOXYGEN */ _XsensResultValue_ /*! \endcond */
{
	// general OK
	/** Operation was performed successfully */
	XRV_OK = 0

	// communication protocol
	/** No bus communication possible */
	,
	XRV_NOBUS = 1
	/** InitBus and/or SetBID are not issued */
	,
	XRV_BUSNOTREADY = 2
	/** Period sent is invalid */
	,
	XRV_INVALIDPERIOD = 3
	/** The message is invalid or not implemented */
	,
	XRV_INVALIDMSG = 4
	/** A slave did not respond to WaitForSetBID */
	,
	XRV_INITBUSFAIL1 = 16
	/** An incorrect answer received after WaitForSetBID */
	,
	XRV_INITBUSFAIL2 = 17
	/** After four bus-scans still undetected Motion Trackers */
	,
	XRV_INITBUSFAIL3 = 18
	/** No reply to SetBID message during SetBID procedure */
	,
	XRV_SETBIDFAIL1 = 20
	/** Other than SetBIDAck received */
	,
	XRV_SETBIDFAIL2 = 21
	/** Timer overflow - period too short to collect all data from Motion
	   Trackers */
	,
	XRV_MEASUREMENTFAIL1 = 24
	/** Motion Tracker responds with other than SlaveData message */
	,
	XRV_MEASUREMENTFAIL2 = 25
	/** Total bytes of data of Motion Trackers including sample counter exceeds
	   255 bytes */
	,
	XRV_MEASUREMENTFAIL3 = 26
	/** Timer overflows during measurement */
	,
	XRV_MEASUREMENTFAIL4 = 27
	/** Timer overflows during measurement */
	,
	XRV_MEASUREMENTFAIL5 = 28
	/** No correct response from Motion Tracker during measurement */
	,
	XRV_MEASUREMENTFAIL6 = 29
	/** Timer overflows during measurement */
	,
	XRV_TIMEROVERFLOW = 30
	/** Baud rate does not comply with valid range */
	,
	XRV_BAUDRATEINVALID = 32
	/** An invalid parameter is supplied */
	,
	XRV_PARAMINVALID = 33
	/** An invalid parameter is supplied */
	,
	XRV_INVALIDPARAM = 33
	/** TX PC Buffer is full */
	,
	XRV_MEASUREMENTFAIL7 = 35
	/** TX PC Buffer overflow, cannot fit full message */
	,
	XRV_MEASUREMENTFAIL8 = 36

	// CMT / XME / etc
	/** A generic error occurred */
	,
	XRV_ERROR = 256
	/** Operation not implemented in this version (yet) */
	,
	XRV_NOTIMPLEMENTED = 257
	/** A timeout occurred */
	,
	XRV_TIMEOUT = 258
	/** Operation aborted because of no data read */
	,
	XRV_TIMEOUTNODATA = 259
	/** Checksum fault occured */
	,
	XRV_CHECKSUMFAULT = 260
	/** No internal memory available */
	,
	XRV_OUTOFMEMORY = 261
	/** The requested item was not found */
	,
	XRV_NOTFOUND = 262
	/** Unexpected message received (e.g. no acknowledge message received) */
	,
	XRV_UNEXPECTEDMSG = 263
	/** Invalid id supplied */
	,
	XRV_INVALIDID = 264
	/** Operation is invalid at this point */
	,
	XRV_INVALIDOPERATION = 265
	/** Insufficient buffer space available */
	,
	XRV_INSUFFICIENTSPACE = 266
	/** The specified i/o device can not be opened */
	,
	XRV_INPUTCANNOTBEOPENED = 267
	/** The specified i/o device can not be opened */
	,
	XRV_OUTPUTCANNOTBEOPENED = 268
	/** An I/O device is already opened with this object */
	,
	XRV_ALREADYOPEN = 269
	/** End of file is reached */
	,
	XRV_ENDOFFILE = 270
	/** A required settings file could not be opened or is missing some data */
	,
	XRV_COULDNOTREADSETTINGS = 271
	/** No data is available */
	,
	XRV_NODATA = 272
	/** Tried to change a read-only value */
	,
	XRV_READONLY = 273
	/** Tried to supply a nullptr value where it is not allowed */
	,
	XRV_NULLPTR = 274
	/** Insufficient data was supplied to a function */
	,
	XRV_INSUFFICIENTDATA = 275
	/** Busy processing, try again later */
	,
	XRV_BUSY = 276
	/** Invalid instance called */
	,
	XRV_INVALIDINSTANCE = 277
	/** A trusted data stream proves to contain corrupted data */
	,
	XRV_DATACORRUPT = 278

	/** Failure during read of settings */
	,
	XRV_READINITFAILED = 279
	/** Could not find any Moven-compatible hardware */
	,
	XRV_NOXMFOUND = 280
	/** Found only one responding Xbus Master */
	,
	XRV_ONLYONEXMFOUND = 281
	/** No sensors found */
	,
	XRV_MTCOUNTZERO = 282
	/** One or more sensors are not where they were expected */
	,
	XRV_MTLOCATIONINVALID = 283
	/** Not enough sensors were found */
	,
	XRV_INSUFFICIENTMTS = 284
	/** Failure during initialization of Fusion Engine */
	,
	XRV_INITFUSIONFAILED = 285
	/** Something else was received than was requested */
	,
	XRV_OTHER = 286

	/** No file opened for reading/writing */
	,
	XRV_NOFILEOPEN = 287
	/** No serial port opened for reading/writing */
	,
	XRV_NOPORTOPEN = 288
	/** No file or serial port opened for reading/writing */
	,
	XRV_NOFILEORPORTOPEN = 289
	/** A required port could not be found */
	,
	XRV_PORTNOTFOUND = 290
	/** The low-level port handler failed to initialize */
	,
	XRV_INITPORTFAILED = 291
	/** A calibration routine failed */
	,
	XRV_CALIBRATIONFAILED = 292

	/** The in-config check of the device failed */
	,
	XRV_CONFIGCHECKFAIL = 293
	/** The operation is once only and has already been performed */
	,
	XRV_ALREADYDONE = 294

	/** The single connected device is configured as a slave */
	,
	XRV_SYNC_SINGLE_SLAVE = 295
	/** More than one master was detected */
	,
	XRV_SYNC_SECOND_MASTER = 296
	/** A device was detected that was neither master nor slave */
	,
	XRV_SYNC_NO_SYNC = 297
	/** No master detected */
	,
	XRV_SYNC_NO_MASTER = 298
	/** A device is not sending enough data */
	,
	XRV_SYNC_DATA_MISSING = 299

	/** The version of the object is too low for the requested operation */
	,
	XRV_VERSION_TOO_LOW = 300
	/** The object has an unrecognized version, so it's not safe to perform the
	   operation */
	,
	XRV_VERSION_PROBLEM = 301
} XsensResultValue;

//////////////////////////////////////////////////////////////////////////////////////////

/*! \cond NODOXYGEN */
#ifndef DELNUL
//! This macro deletes a pointer and sets it to NULL
#define DELNUL(ptr)    \
	{                  \
		delete ptr;    \
		ptr = nullptr; \
	}
//! This macro deletes a pointer if it is not nullptr and then sets it to NULL
#define CHKDELNUL(ptr)      \
	{                       \
		if (ptr != nullptr) \
		{                   \
			delete ptr;     \
			ptr = nullptr;  \
		}                   \
	}
//! This macro deletes a multi-object pointer and sets it to NULL
#define LSTDELNUL(ptr) \
	{                  \
		delete[] ptr;  \
		ptr = nullptr; \
	}
//! This macro deletes a multi-object pointer if it is not nullptr and then sets
//! it to NULL
#define LSTCHKDELNUL(ptr)   \
	{                       \
		if (ptr != nullptr) \
		{                   \
			delete[] ptr;   \
			ptr = nullptr;  \
		}                   \
	}

//! This macro frees a pointer and then sets it to NULL
#define FREENUL(ptr)   \
	{                  \
		free(ptr);     \
		ptr = nullptr; \
	}
//! This macro frees a pointer if it is not nullptr and then sets it to NULL
#define CHKFREENUL(ptr)     \
	{                       \
		if (ptr != nullptr) \
		{                   \
			free(ptr);      \
			ptr = nullptr;  \
		}                   \
	}
#endif
/*! \endcond */

//////////////////////////////////////////////////////////////////////////////////////////

/* \brief Retrieve a string corresponding to the given result code.

	This function uses the CmtResultText list to return a string with the
   relevant result
	code in a textual format. If the supplied result code is invalid the
	"!!Invalid result code!!" string is returned.
*/
#if !defined(_CMT_DLL_IMPORT) && !defined(_XME_DLL_IMPORT)
const char* xsensResultText(const XsensResultValue result);
#endif

//////////////////////////////////////////////////////////////////////////////////////////

/** The maximum length of a label */
#define XSENS_LABEL_SIZE 64
/** The maximum length of a short string */
#define XSENS_SHORT_STRING_SIZE 256
/** The maximum length of a long string */
#define XSENS_LONG_STRING_SIZE 16384

//////////////////////////////////////////////////////////////////////////////////////////

#endif  // _XSENS_STD_H_2006_09_11
