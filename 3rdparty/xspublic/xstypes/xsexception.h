
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

#ifdef __cplusplus
#ifndef XSEXCEPTION_H
#define XSEXCEPTION_H

#ifndef XSENS_NO_EXCEPTIONS

#include <exception>
#include "xsresultvalue.h"
#include "xsstring.h"

/*! \brief Exception class for Xsens public libraries. Inherits from std::exception
*/
class XsException : public std::exception {
public:
	//! \brief Copy constructor
	XsException(XsException const& e)
		: std::exception()
		, m_code(e.m_code)
		, m_description(e.m_description)
	{
	}

	/*! \brief Initializing constructor
		\details This constructor uses the value in \a err and the supplied \a description to create a full
		text for when the user requests what() or text()
		\param err The error code that the exception should report
		\param description A description of the error. The constructor prefixes this with a textual
							description of the error code unless prefix is false.
		\param prefix Whether to prefix the description with a textual description of the error code or not (default is yes)
	*/
	XsException(XsResultValue err, XsString const& description, bool prefix = true)
		: std::exception()
		, m_code(err)
		, m_description(description)
	{
		if (prefix && (m_code != XRV_OK))
		{
			char codeString[16];
			sprintf(codeString, "%d: ", (int) m_code);
			XsString rv(codeString);
			rv << XsResultValue_toString(m_code);
			if (!m_description.empty())
			{
				rv << ". ";
				rv.append(m_description);
			}
			m_description.swap(rv);
		}
	}

	/*! \brief Initializing constructor
		\param description A description of the error.
	*/
	explicit XsException(XsString const& description)
		: std::exception()
		, m_code(XRV_ERROR)
		, m_description(description)
	{
	}

	//! \brief Destructor
	virtual ~XsException() noexcept
	{
	}

	//! \brief Assignment operator, copies \a e to this
	XsException& operator = (XsException const& e)
	{
		if (this != &e)
		{
			m_code = e.m_code;
			m_description = e.m_description;
		}
		return *this;
	}

	//! \brief Returns the error value supplied during construction
	inline XsResultValue code() const noexcept
	{
		return m_code;
	}

	//! \brief Returns a description of the error that occurred as a char const*
	inline char const* what() const noexcept
	{
		return m_description.c_str();
	}

	//! \brief Returns a description of the error that occurred as a XsString
	inline XsString const& text() const noexcept
	{
		return m_description;
	}

private:
	XsResultValue m_code;	//!< The supplied error code
	XsString m_description;	//!< The supplied description, possibly prefixed with a description of the error code
};

#endif

#endif
#endif // __cplusplus guard
