
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

#ifndef COMMON_QDEBUG_H
#define COMMON_QDEBUG_H

#ifdef __cplusplus
#include <QDebug>

#define EXT_DEBUG_BODY(msg)  __FILE__ << "(" << __LINE__ << "):" << __FUNCTION__ << msg

#ifdef XSENS_RELEASE
	#define QTRACE(msg) ((void)0)
	#define QDEBUG(msg) ((void)0)
	#define QDEBUG_ACCURATE(msg) ((void)0)
#else
//	#ifdef NO_QTRACE
		#define QTRACE(msg) ((void)0)
//	#else
//		#define QTRACE(msg) qDebug() << __FUNCTION__ << msg
//	#endif
	#define QDEBUG(msg) qDebug() << __FUNCTION__ << msg

//	#include "xqtime.h"
//	#define QDEBUG_ACCURATE(msg) qDebug() << xqLogTime() <<  __FUNCTION__ << msg
#endif // XSENS_RELEASE

#define QWARNING(msg) qWarning() << EXT_DEBUG_BODY(msg)
#define QCRITICAL(msg) qCritical() << EXT_DEBUG_BODY(msg)
#define QFATAL(msg) do not use this macro! qFatal has weird side-effects!
#endif

#endif	// file guard
