
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

#ifndef JOURNALLOGLEVEL_H
#define JOURNALLOGLEVEL_H

#ifdef ANDROID
	#include <android/log.h>

	#define JLL_TRACE	ANDROID_LOG_VERBOSE
	#define JLL_DEBUG	ANDROID_LOG_DEBUG
	#define JLL_ALERT	ANDROID_LOG_WARN
	#define JLL_ERROR	ANDROID_LOG_ERROR
	#define JLL_FATAL	ANDROID_LOG_FATAL
	#define JLL_WRITE	ANDROID_LOG_SILENT
	#define JLL_DISABLE	(JLL_WRITE+1)
#else
	#define JLL_TRACE	0
	#define JLL_DEBUG	1
	#define JLL_ALERT	2
	#define JLL_ERROR	3
	#define JLL_FATAL	4
	#define JLL_WRITE	5
	#define JLL_DISABLE	6
#endif

enum JournalLogLevel {
	  JLL_Trace = JLL_TRACE		//!< log all messages, including function entry/exit
	, JLL_Debug = JLL_DEBUG		//!< log all messages, except function entry/exit (trace)
	, JLL_Alert = JLL_ALERT		//!< only log fatal, error and alert messages
	, JLL_Error = JLL_ERROR		//!< only log fatal and error messages
	, JLL_Fatal = JLL_FATAL		//!< only log fatal messages
	, JLL_Write = JLL_WRITE		//!< only log 'write' messages
	, JLL_Diable= JLL_DISABLE	//!< don't log any messages
};

#ifdef NO_JOURNALLER
	#ifdef JLDEF_BUILD
		#undef JLDEF_BUILD
	#endif
	#define JLDEF_BUILD		JLL_DISABLE
#endif

#ifdef XSENS_DEBUG
	#ifndef XSENS_RELEASE
		// full debug
		#ifndef JLDEF_BUILD
			#define JLDEF_BUILD		JLL_DEBUG	// 'trace' needs to be enabled explicitly since it potentially has a huge impact on performance
		#endif
		#ifndef JLDEF_FILE
			#define JLDEF_FILE		JLL_Debug
		#endif
		#ifndef JLDEF_DEBUGGER
			#define JLDEF_DEBUGGER	JLL_Alert
		#endif
	#else
		// Release With Debug Info (non-optimized Release build)
		#ifndef JLDEF_BUILD
			#define JLDEF_BUILD		JLL_DEBUG
		#endif
		#ifndef JLDEF_FILE
			#define JLDEF_FILE		JLL_Debug
		#endif
		#ifndef JLDEF_DEBUGGER
			#define JLDEF_DEBUGGER	JLL_Error
		#endif
	#endif
#else
	// Full optimized Release build, logging should be reduced to a minimum
	#ifndef JLDEF_BUILD
		#define JLDEF_BUILD		JLL_ALERT
	#endif
	#ifndef JLDEF_FILE
		#define JLDEF_FILE		JLL_Alert
	#endif
	#ifndef JLDEF_DEBUGGER
		#define JLDEF_DEBUGGER	JLL_Fatal
	#endif
	#define JLNOLINEINFO
#endif

#endif
