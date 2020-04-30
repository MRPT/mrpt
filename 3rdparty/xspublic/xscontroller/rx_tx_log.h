
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

#ifndef RX_TX_LOG_H
#define RX_TX_LOG_H

#if defined(XSENS_DEBUG)
	//// Debug or RelWithDeb build
	//#define LOG_RX_TX			// Lowest level byte receive and send (binary log)
	//#define LOG_RX_TX_UNIQUE	// Use unique file names for rx/tx logs
	//#define LOG_RX_TX_FLUSH		// Flush after each log operation (can cause hickups in timing, 300ms is not unheard of)
	//#define LOG_RX_TX_PER_STATE	// Use new unique log file after switching to a new state (config/measurement/operational/recording) for rx/tx logs, override LOG_RX_TX_UNIQUE, automatically disabled if LOG_RX_TX is disabled
#endif

#ifdef LOG_RX_TX
#if defined(LOG_RX_TX_PER_STATE) || defined(LOG_RX_TX_UNIQUE)
#include <xstypes/xstimestamp.h>
#include <xstypes/xsxbusmessageid.h>

/*! \brief Helper function for making filename of log file unique
*/
inline static void makeFilenameUnique(char* filename, char const* state)
{
#if defined(LOG_RX_TX_PER_STATE) || defined(LOG_RX_TX_UNIQUE)
	char basename[XS_MAX_FILENAME_LENGTH];
	strcpy(basename, filename);
	basename[strlen(basename) - 4] = 0;	// remove .log extension
	sprintf(filename, "%s_%" PRINTF_INT64_MODIFIER "u%s.log", basename, XsTimeStamp::nowMs(), state);
#else
	(void)filename;
#endif
}


#ifdef LOG_RX_TX_PER_STATE
inline static void checkStateRx(char* state, int length, XsByteArray const& data, XsFile& rx_log)
{
	state[0] = 0;
	if (length >= 4)
	{
		// find preamble
		int idx = 0;
		while (idx < length-4)
		{
			if (data[idx] == 0xFA && (data[idx+1] == 0xFF || data[idx+1] == 0) && data[idx+3] == 0)
			{
				switch (data[idx+2])
				{
				case XMID_GotoConfigAck:
					strcpy(state, "_Config");
					break;
				case XMID_GotoMeasurementAck:
					strcpy(state, "_Measurement");
					break;
				case XMID_GotoOperationalAck:
					strcpy(state, "_Operational");
					break;
				case XMID_StartRecordingAck:
					strcpy(state, "_Recording");
					break;
				case XMID_StopRecordingAck:
					strcpy(state, "_Flushing");
					break;
				default:
					++idx;
					continue;
				}
				break;
			}
			++idx;
		}
	}
	if (rx_log.isOpen() && state[0] != 0)
	{
		rx_log.flush();
		rx_log.close();
	}
}
inline static void checkStateRx(char* state, int length, void const* data, XsFile& rx_log)
{
	XsByteArray tmp((uint8_t*) data, length);
	checkStateRx(state, length, tmp, rx_log);
}

#define CHECK_STATE_RX(length, data, logfile)	\
	char state[16] = "";\
	checkStateRx(state, (int) (length), (data), (logfile))

inline static void checkStateTx(char* state, int length, XsByteArray const& data, XsFile& tx_log)
{
	state[0] = 0;
	if (length >= 4)
	{
		// find preamble
		int idx = 0;
		while (idx < length-4)
		{
			if (data[idx] == 0xFA && (data[idx+1] == 0xFF || data[idx+1] == 0) && data[idx+3] == 0)
			{
				switch (data[idx+2])
				{
				case XMID_GotoConfig:
					strcpy(state, "_Config");
					break;
				case XMID_GotoMeasurement:
					strcpy(state, "_Measurement");
					break;
				case XMID_GotoOperational:
					strcpy(state, "_Operational");
					break;
				case XMID_StartRecording:
					strcpy(state, "_Recording");
					break;
				case XMID_StopRecording:
					strcpy(state, "_Flushing");
					break;
				default:
					++idx;
					continue;
				}
				break;
			}
			++idx;
		}
	}
	if (tx_log.isOpen() && state[0] != 0)
	{
		tx_log.flush();
		tx_log.close();
	}
}
inline static void checkStateTx(char* state, int length, void const* data, XsFile& tx_log)
{
	XsByteArray tmp((uint8_t*) data, length);
	checkStateTx(state, length, tmp, tx_log);
}
#define CHECK_STATE_TX(length, data, logfile)	\
	char state[16] = "";\
	checkStateTx(state, (int) (length), (data), (logfile))

#else
	static const char state[1] = "";
	#define CHECK_STATE_RX(...)	((void)0)
	#define CHECK_STATE_TX(...)	((void)0)
#endif

#endif
#endif
#endif
