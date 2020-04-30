
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

#ifdef CALLTRACER_H
XSENS_COMPILER_WARNING("X0008", "calltracer.h should only be included ONCE and NEVER in a header file")
#else
#define CALLTRACER_H
#endif

//#define TRACECALLS	// define before including this header file to enable semi-automatic calltracing
#ifdef TRACECALLS
#if JLDEF_BUILD <= JLL_TRACE
#include <xstypes/xstime.h>
#include "journaller.h"

/*! \brief The CallTracer class is used to log function entry and exit.
	\details Usually the class is instantiated through one of the TRACETHIS, TRACEGLOBAL or
	TRACETHIS2 macros, but it can be created manually if desired. The class writes a log-line
	during creation and another one when it gets destroyed, if a minimum time has passed since
	construction.
*/
class CallTracer {
public:
	Journaller* m_journal;
	std::string m_msg;

	/*! \brief Constructor, writes an entry logline and starts timing the function
		\param functionName The name of the function that contains the CallTracer object (or some other identifying string)
		\param object A pointer to an object, usually the this pointer of the containing function. Used to separate nested and concurrent calls to the same function.
		\param minTime The minimum time in ms to pass for an exit logline to be created. Set to 0 to always write an exit line.
	*/
	CallTracer(Journaller* journal, const std::string& functionName, const void* object)
		: m_journal(journal)
		, m_msg(functionName)
	{
		if (object)
		{
			char ptr[32];
			sprintf(ptr, " [%p]", object);
			m_msg += ptr;
		}
		JLTRACE_NODEC(m_journal, m_msg << " entry");
	}

	/*! \brief Destructor, writes an exit logline if at least m_minimumTime has passed
	*/
	~CallTracer()
	{
		JLTRACE_NODEC(m_journal, m_msg << " exit");
	}
};
#define TRACEFUNC(journal) CallTracer trAceR(journal, std::string(__FILE__) + ":" + __FUNCTION__, NULL)
#define TRACETHIS(journal) CallTracer trAceR(journal, std::string(__FILE__) + ":" + __FUNCTION__, this)
#else
#define TRACEFUNC(j)	((void) 0)
#define TRACETHIS(j)	((void) 0)
#endif
#else
#define TRACEFUNC(j)	((void) 0)
#define TRACETHIS(j)	((void) 0)
#endif
