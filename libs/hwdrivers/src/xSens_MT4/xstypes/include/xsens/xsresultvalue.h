/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef XSRESULTVALUE_H
#define XSRESULTVALUE_H

#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//////////////////////////////////////////////////////////////////////////////////////////
	/*!	\addtogroup enums Global enumerations
		@{
	*/
	/*!	\brief Xsens result values
		\details These values are used to signal success or specific failures of
	   functions
		\sa XsResultValue_toString
	*/
	enum XsResultValue
	{
		// general OK
		/** 0: Operation was performed successfully */
		XRV_OK = 0

		// communication protocol
		/** 1: No bus communication possible */
		,
		XRV_NOBUS = 1
		/** 2: InitBus and/or SetBID are not issued */
		,
		XRV_BUSNOTREADY = 2
		/** 3: Period sent is invalid */
		,
		XRV_INVALIDPERIOD = 3
		/** 4: The message is invalid or not implemented */
		,
		XRV_INVALIDMSG = 4
		/** 16: A slave did not respond to WaitForSetBID */
		,
		XRV_INITBUSFAIL1 = 16
		/** 17: An incorrect answer received after WaitForSetBID */
		,
		XRV_INITBUSFAIL2 = 17
		/** 18: After four bus-scans still undetected Motion Trackers */
		,
		XRV_INITBUSFAIL3 = 18
		/** 20: No reply to SetBID message during SetBID procedure */
		,
		XRV_SETBIDFAIL1 = 20
		/** 21: Other than SetBIDAck received */
		,
		XRV_SETBIDFAIL2 = 21
		/** 24: Timer overflow - period too short to collect all data from
		   Motion Trackers */
		,
		XRV_MEASUREMENTFAIL1 = 24
		/** 25: Motion Tracker responds with other than SlaveData message */
		,
		XRV_MEASUREMENTFAIL2 = 25
		/** 26: Total bytes of data of Motion Trackers including sample counter
		   exceeds 255 bytes */
		,
		XRV_MEASUREMENTFAIL3 = 26
		/** 27: Timer overflows during measurement */
		,
		XRV_MEASUREMENTFAIL4 = 27
		/** 28: Timer overflows during measurement */
		,
		XRV_MEASUREMENTFAIL5 = 28
		/** 29: No correct response from Motion Tracker during measurement */
		,
		XRV_MEASUREMENTFAIL6 = 29
		/** 30: Timer overflows during measurement */
		,
		XRV_TIMEROVERFLOW = 30
		/** 32: Baud rate does not comply with valid range */
		,
		XRV_BAUDRATEINVALID = 32
		/** 33: An invalid parameter is supplied */
		,
		XRV_INVALIDPARAM = 33
		/** 35: TX PC Buffer is full */
		,
		XRV_MEASUREMENTFAIL7 = 35
		/** 36: TX PC Buffer overflow, cannot fit full message */
		,
		XRV_MEASUREMENTFAIL8 = 36
		/** 40: The device generated an error, try updating the firmware */
		,
		XRV_DEVICEERROR = 40
		/** 41: The device generates more data than the bus communication can
		   handle (baud rate may be too low) */
		,
		XRV_DATAOVERFLOW = 41
		/** 42: The sample buffer of the device was full during a communication
		   outage */
		,
		XRV_BUFFEROVERFLOW = 42

		// CMT / XDA / XME / etc
		/** 256: A generic error occurred */
		,
		XRV_ERROR = 256
		/** 257: Operation not implemented in this version (yet) */
		,
		XRV_NOTIMPLEMENTED = 257
		/** 258: A timeout occurred */
		,
		XRV_TIMEOUT = 258
		/** 259: Operation aborted because of no data read */
		,
		XRV_TIMEOUTNODATA = 259
		/** 260: Checksum fault occurred */
		,
		XRV_CHECKSUMFAULT = 260
		/** 261: No internal memory available */
		,
		XRV_OUTOFMEMORY = 261
		/** 262: The requested item was not found */
		,
		XRV_NOTFOUND = 262
		/** 263: Unexpected message received (e.g. no acknowledge message
		 * received)
		 */
		,
		XRV_UNEXPECTEDMSG = 263
		/** 264: Invalid id supplied */
		,
		XRV_INVALIDID = 264
		/** 265: Operation is invalid at this point */
		,
		XRV_INVALIDOPERATION = 265
		/** 266: Insufficient buffer space available */
		,
		XRV_INSUFFICIENTSPACE = 266
		/** 267: The specified i/o device can not be opened */
		,
		XRV_INPUTCANNOTBEOPENED = 267
		/** 268: The specified i/o device can not be opened */
		,
		XRV_OUTPUTCANNOTBEOPENED = 268
		/** 269: An I/O device is already opened with this object */
		,
		XRV_ALREADYOPEN = 269
		/** 270: End of file is reached */
		,
		XRV_ENDOFFILE = 270
		/** 271: A required settings file could not be opened or is missing some
		   data */
		,
		XRV_COULDNOTREADSETTINGS = 271
		/** 272: No data is available */
		,
		XRV_NODATA = 272
		/** 273: Tried to change a read-only value */
		,
		XRV_READONLY = 273
		/** 274: Tried to supply a nullptr value where it is not allowed */
		,
		XRV_NULLPTR = 274
		/** 275: Insufficient data was supplied to a function */
		,
		XRV_INSUFFICIENTDATA = 275
		/** 276: Busy processing, try again later */
		,
		XRV_BUSY = 276
		/** 277: Invalid instance called */
		,
		XRV_INVALIDINSTANCE = 277
		/** 278: A trusted data stream proves to contain corrupted data */
		,
		XRV_DATACORRUPT = 278

		/** 279: Failure during read of settings */
		,
		XRV_READINITFAILED = 279
		/** 280: Could not find any MVN-compatible hardware */
		,
		XRV_NOXMFOUND = 280
		/** 281: Found only one responding Xbus Master */
		,
		XRV_ONLYONEXMFOUND = 281
		/** 282: No sensors found */
		,
		XRV_MTCOUNTZERO = 282
		/** 283: One or more sensors are not where they were expected */
		,
		XRV_MTLOCATIONINVALID = 283
		/** 284: Not enough sensors were found */
		,
		XRV_INSUFFICIENTMTS = 284
		/** 285: Failure during initialization of Fusion Engine */
		,
		XRV_INITFUSIONFAILED = 285
		/** 286: Something else was received than was requested */
		,
		XRV_OTHER = 286

		/** 287: No file opened for reading/writing */
		,
		XRV_NOFILEOPEN = 287
		/** 288: No serial port opened for reading/writing */
		,
		XRV_NOPORTOPEN = 288
		/** 289: No file or serial port opened for reading/writing */
		,
		XRV_NOFILEORPORTOPEN = 289
		/** 290: A required port could not be found */
		,
		XRV_PORTNOTFOUND = 290
		/** 291: The low-level port handler failed to initialize */
		,
		XRV_INITPORTFAILED = 291
		/** 292: A calibration routine failed */
		,
		XRV_CALIBRATIONFAILED = 292

		/** 293: The in-config check of the device failed */
		,
		XRV_CONFIGCHECKFAIL = 293
		/** 294: The operation is once only and has already been performed */
		,
		XRV_ALREADYDONE = 294

		/** 295: The single connected device is configured as a slave */
		,
		XRV_SYNC_SINGLE_SLAVE = 295
		/** 296: More than one master was detected */
		,
		XRV_SYNC_SECOND_MASTER = 296
		/** 297: A device was detected that was neither master nor slave */
		,
		XRV_SYNC_NO_SYNC = 297
		/** 298: No master detected */
		,
		XRV_SYNC_NO_MASTER = 298
		/** 299: A device is not sending enough data */
		,
		XRV_SYNC_DATA_MISSING = 299

		/** 300: The version of the object is too low for the requested
		   operation */
		,
		XRV_VERSION_TOO_LOW = 300
		/** 301: The object has an unrecognized version, so it's not safe to
		   perform the operation */
		,
		XRV_VERSION_PROBLEM = 301

		/** 302: The process was aborted by an external event, usually a user
		   action or process termination */
		,
		XRV_ABORTED = 302
		/** 303: The requested functionality is not supported by the device */
		,
		XRV_UNSUPPORTED = 303

		/** 304: A packet counter value was missed */
		,
		XRV_PACKETCOUNTERMISSED = 304

		/** 305: An error occured while trying to put the device in measurement
		 * mode
		 */
		,
		XRV_MEASUREMENTFAILED = 305
	};
	/*! @} */
	typedef enum XsResultValue XsResultValue;

	//! These enum values can be used to specify a device error (XRV_DEVICEERROR
	//! 0x28), i.e. a xbus message like [FA FF 42 05 28 XXXXXXXX CS]
	enum XsDeviceErrorType
	{
		XERR_Unknown = 0,
		XERR_ImcuTimeout = 1,
		XERR_ImcuSettingsInvalid = 2,
		XERR_ImcuSettingsFailure = 3,
		XERR_ImcuEmtsWriteFailure = 4,
		XERR_ImcuEmtsReadFailure = 5,
		XERR_DspCrashed = 6,
		XERR_DspBootingTimeout = 7
	};
	typedef enum XsDeviceErrorType XsDeviceErrorType;

	//////////////////////////////////////////////////////////////////////////////////////////

	/* \brief Retrieve a string corresponding to the given result code.

		This function uses the XsResultText list to return a string with the
	   relevant result
		code in a textual format. If the supplied result code is invalid the
		"!!Invalid result code!!" string is returned.
	*/
	XSTYPES_DLL_API const char* XsResultValue_toString(XsResultValue result);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // file guard
