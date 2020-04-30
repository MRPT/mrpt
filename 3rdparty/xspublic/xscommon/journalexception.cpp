
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

#include "journalstackwalker.h"
#include "journalexception.h"

/*! \brief A helper class that combines the StackWalker output into a std::string */
class JournalExceptionStackWalker : public StackWalker {
public:
	JournalExceptionStackWalker(JournalException* exc)
		: StackWalker()
		, m_exception(exc)
		{}

	JournalException* m_exception;

	virtual void OnOutput(LPCSTR szText)
	{
		m_exception->m_stack += szText;
		m_exception->m_stack += "\n";
	}
};

/*! \class JournalException
	\brief An exception class that automatically includes stack dump information
*/

/*! \brief Constructor, copies the \a message and creates a stack dump */
JournalException::JournalException(std::string const& message)
	: XsException(XRV_ERROR, message)
	, m_stack("************ Dump Begin ************\n")
{
	JournalExceptionStackWalker sw(this);
	sw.ShowCallstack();
	m_stack += "************* Dump End *************";
}

/*! \brief The message as supplied to the constructor */
const char* JournalException::msg() const
{
	return what();
}

/*! \brief The stack dump as it was at the time of object construction */
const std::string& JournalException::stack() const
{
	return m_stack;
}

#ifdef XSENS_DEBUG
#define GOTODEBDEF	true
#else
#define GOTODEBDEF	false
#endif

/*! When set to true, any caught exception at the C level will be passed on upward (ie to the debugger)
	When set to false, the exception handling stops at this level. This only applies to C level exceptions.
*/
bool gOnExceptionGotoDebugger = GOTODEBDEF;

#ifdef XSENS_WINDOWS
/*! \brief Exception filter for crash handler.

	Using this filter in __try __except() will generate a crash log with a call stack.
	\param pExPtrs pointers to exception struct, use GetExceptionInformation()
	\param journal The Journaller object to use for logging
*/
LONG journallerExceptionFilter(EXCEPTION_POINTERS* pExPtrs, Journaller* journal)
{
	journal->log(JLL_Fatal, "******* C level exception (CRASH) *******");
	journal->log(JLL_Fatal, "************ Dump Begin ************");
	JournalStackWalker sw(journal);
	sw.ShowCallstack(GetCurrentThread(), pExPtrs->ContextRecord);
	journal->log(JLL_Fatal, "************* Dump End *************");

	return gOnExceptionGotoDebugger ? EXCEPTION_CONTINUE_SEARCH : EXCEPTION_EXECUTE_HANDLER;
}
#endif
