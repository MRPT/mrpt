
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

#ifndef LASTRESULTMANAGER_H
#define LASTRESULTMANAGER_H

#include <xstypes/xsresultvalue.h>
#include <xstypes/xsstring.h>

/*!	\class LastResultManager
	\brief This class manages a result code with optional additional text
	\details It can be treated like a simple XsResultValue, but gives extra options
*/
class LastResultManager {
	XsResultValue m_lastResult;
	XsString m_lastResultText;
public:
	LastResultManager() : m_lastResult(XRV_OK) {}

	/*! \brief Sets the last results
		\param res : The result value to set
		\param text : The result text to set
	*/
	void set(XsResultValue res, XsString const& text)
	{
		m_lastResult = res;
		m_lastResultText = text;
	}

	/*! \returns The last result value
	*/
	XsResultValue lastResult() const { return m_lastResult; }

	/*! \returns The last result text
	*/
	XsString lastResultText() const
	{
		if (m_lastResultText.empty())
			return XsResultValue_toString(m_lastResult);
		return m_lastResultText;
	}

	/*! \brief Assignment operator, copies contents from the \a res result value
		\param res : The result value to copy from
		\returns The assigned result value
	*/
	XsResultValue operator = (XsResultValue res)
	{
		m_lastResult = res;
		m_lastResultText.clear();
		return res;
	}

	/*! \returns The const refernce to the last result
	*/
	operator XsResultValue const&() { return m_lastResult; }
};

#endif
