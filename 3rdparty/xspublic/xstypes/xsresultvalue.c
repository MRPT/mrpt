
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

#include "xsresultvalue.h"

/*! \addtogroup cinterface C Interface
	@{
*/

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Retrieve a character string corresponding to the given result code.
	\param result The result code to convert
	\return A static string describing the result code
	\note In the Python interface this function is called XsResultValueToString
	\note In C/C++ do NOT delete the returned char* as it points to static memory.
*/
const char* XsResultValue_toString(XsResultValue result)
{
	switch(result)
	{
	case XRV_OK:					return "No error";

		// Communication protocol
	case XRV_NOBUS:					return "Bus has no power";
	case XRV_BUSNOTREADY:			return "Bus not ready for measurement";
	case XRV_INVALIDPERIOD:			return "Period is invalid";
	case XRV_INVALIDMSG:			return "Message is invalid";
	case XRV_INITBUSFAIL1:			return "A slave did not respond to WaitForSetBID";
	case XRV_INITBUSFAIL2:			return "An incorrect answer received after WaitForSetBID";
	case XRV_INITBUSFAIL3:			return "After four bus-scans still undetected Motion Trackers";
	case XRV_SETBIDFAIL1:			return "No reply to SetBID message during SetBID procedure";
	case XRV_SETBIDFAIL2:			return "Other than SetBIDAck received";
	case XRV_MEASUREMENTFAIL1:		return "Timer overflow - period too short to collect all data from Motion Trackers";
	case XRV_MEASUREMENTFAIL2:		return "Motion Tracker responds with other than SlaveData message";
	case XRV_MEASUREMENTFAIL3:		return "Total bytes of data of Motion Trackers incl sample counter exceeds 255 bytes";
	case XRV_MEASUREMENTFAIL4:		return "Timer overflow during measurement - MT does not respond within measurement period. Increase period (lower update rate)";
	case XRV_MEASUREMENTFAIL5:		return "Timer overflow during measurement - MT response was not received within measurement period. Increase period (lower update rate) or use fewer Motion Trackers";
	case XRV_MEASUREMENTFAIL6:		return "No correct response from Motion Tracker during measurement";
	case XRV_TIMEROVERFLOW:			return "Timer overflow during measurement";
	case XRV_BAUDRATEINVALID:		return "Baudrate does not comply with valid range";
	case XRV_INVALIDPARAM:			return "Invalid parameter supplied";
	case XRV_MEASUREMENTFAIL7:		return "TX PC Buffer is full";
	case XRV_MEASUREMENTFAIL8:		return "TX PC Buffer overflow, cannot fit full message";
	case XRV_WIRELESSFAIL:			return "Wireless communication system failed";
	case XRV_DEVICEERROR:			return "The device generated an error, try updating the firmware";
	case XRV_DATAOVERFLOW:			return "Data overflow";
	case XRV_BUFFEROVERFLOW:		return "Sample buffer overflow during communication outage";
	case XRV_EXTTRIGGERERROR:		return "The external trigger is not behaving as configured";
	case XRV_SAMPLESTREAMERROR:		return "Sample stream detected an error in ordering input data";
	case XRV_POWER_DIP:				return "A power dip has been detected and recovered from";
	case XRV_POWER_OVERCURRENT:		return "A current limiter has been activated";
	case XRV_OVERHEATING:			return "Device temperature exceded operational limits";
	case XRV_BATTERYLOW:			return "Battery level reached lower limit";
	case XRV_INVALIDFILTERPROFILE:	return "Specified filter profile ID is not available on the device or the user is trying to duplicate an existing filter profile type";
	case XRV_INVALIDSTOREDSETTINGS:	return "The settings stored in the device's non volatile memory are invalid";
	case XRV_ACCESSDENIED:			return "Request for control of the device was denied";
	case XRV_FILEERROR:				return "Failure reading, writing, opening or closing a file";
	case XRV_OUTPUTCONFIGERROR:		return "Erroneous output configuration, device can not go to measurement";

		// XDA / XME / etc
	case XRV_ERROR:					return "Generic error";
	case XRV_NOTIMPLEMENTED:		return "Operation not implemented";
	case XRV_TIMEOUT:				return "Timeout occurred, some data received";
	case XRV_TIMEOUTNODATA:			return "Timeout occurred, no data was received";
	case XRV_CHECKSUMFAULT:			return "Checksum fault";
	case XRV_OUTOFMEMORY:			return "Out of memory";
	case XRV_NOTFOUND:				return "Requested item was not found";
	case XRV_UNEXPECTEDMSG:			return "Unexpected message received";
	case XRV_INVALIDID:				return "Invalid id supplied";
	case XRV_INVALIDOPERATION:		return "Invalid operation";
	case XRV_INSUFFICIENTSPACE:		return "Insufficient buffer space available";
	case XRV_INPUTCANNOTBEOPENED:	return "Could not open input device";
	case XRV_OUTPUTCANNOTBEOPENED:	return "Could not open output device";
	case XRV_ALREADYOPEN:			return "A device is already open";
	case XRV_ENDOFFILE:				return "End of file reached";
	case XRV_COULDNOTREADSETTINGS:	return "A required settings file could not be opened or is missing some data";
	case XRV_NODATA:				return "No data available";
	case XRV_READONLY:				return "Tried to change a read-only value";
	case XRV_NULLPTR:				return "Invalid NULL pointer supplied";
	case XRV_INSUFFICIENTDATA:		return "Insufficient data supplied";
	case XRV_BUSY:					return "Busy processing, try again later";
	case XRV_INVALIDINSTANCE:		return "Invalid instance called, because of an invalid or missing license";
	case XRV_DATACORRUPT:			return "Data-source corrupt";

	case XRV_READINITFAILED:		return "Failure during reading of settings. File may be old or corrupt.";
	case XRV_NOXMFOUND:				return "Could not find any MVN-compatible hardware, check connections and LEDs";
	case XRV_DEVICECOUNTZERO:		return "No xsens devices found, check connections";
	case XRV_MTLOCATIONINVALID:		return "One or more sensors are not where they were expected, check locations";
	case XRV_INSUFFICIENTMTS:		return "Not enough sensors were found, check connections";
	case XRV_INITFUSIONFAILED:		return "Failure during initialization of Fusion Engine. Source file may be old or corrupt.";
	case XRV_OTHER:					return "Something else was received than was requested";

	case XRV_NOFILEOPEN:			return "No file open";
	case XRV_NOPORTOPEN:			return "No serial port open";
	case XRV_NOFILEORPORTOPEN:		return "No file or serial port open";
	case XRV_PORTNOTFOUND:			return "A required port could not be found";
	case XRV_INITPORTFAILED:		return "The low-level port handler failed to initialize";
	case XRV_CALIBRATIONFAILED:		return "A calibration routine failed";

	case XRV_CONFIGCHECKFAIL:		return "Failure during device configuration";
	case XRV_ALREADYDONE:			return "This once only operation has already been performed";

	case XRV_SYNC_SINGLE_SLAVE:		return "The single connected device is configured as a slave";
	case XRV_SYNC_SECOND_MASTER:	return "More than one master was detected";
	case XRV_SYNC_NO_SYNC:			return "A device was detected that was neither master nor slave";
	case XRV_SYNC_NO_MASTER:		return "No master detected";
	case XRV_SYNC_DATA_MISSING:		return "A device is not sending enough data, check synchronization cables";

	case XRV_VERSION_TOO_LOW:		return "The version of the object is too low for the requested operation";
	case XRV_VERSION_PROBLEM:		return "The object has an unrecognized version, so it's not safe to perform the operation";

	case XRV_ABORTED:				return "The process was aborted by an external event";
	case XRV_UNSUPPORTED:			return "The requested functionality is not supported by the device";

	case XRV_PACKETCOUNTERMISSED:	return "A packet counter value was missed";

	case XRV_MEASUREMENTFAILED:		return "Failed to start measurement";
	case XRV_STARTRECORDINGFAILED:	return "A device could not start recording";
	case XRV_STOPRECORDINGFAILED:	return "A device could not stop recording";

	case XRV_RADIO_CHANNEL_IN_USE:	return "Detected another system using the selected radio channel";
	case XRV_UNEXPECTED_DISCONNECT:	return "Motion tracker disconnected unexpectedly";
	case XRV_TOO_MANY_CONNECTED_TRACKERS:	return "Too many motion trackers connected";
	case XRV_GOTOCONFIGFAILED:		return "Failed to go to config mode";
	case XRV_OUTOFRANGE:			return "Device has gone out of range";
	case XRV_BACKINRANGE:			return "Device is back in range, resuming normal operation";
	case XRV_EXPECTED_DISCONNECT:	return "The device was disconnected";

	case XRV_RESTORE_COMMUNICATION_FAILED:	return "Restore communication failed";
	case XRV_RESTORE_COMMUNICATION_STOPPED:	return "Restore communication was stopped";

	case XRV_EXPECTED_CONNECT:		return "The device connected";

	case XRV_SHUTTINGDOWN:			return "The device is shutting down";
	case XRV_GNSSCONFIGURATIONERROR:return "A configuration item was refused by the GNSS module";
	case XRV_GNSSCOMMTIMEOUT:		return "The communication with the GNSS module timed out";
	case XRV_GNSSERROR:				return "Communication between the device and the GNSS module failed";
	case XRV_DEVICE_NOT_CALIBRATED:	return "The EMTS of the device does not contain calibration data";

	default:						return "!!Undefined Result Value!!";
	}
}

/*! @} */
