/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifdef __cplusplus
#ifndef XSEXCEPTION_H
#define XSEXCEPTION_H

#include <exception>
#include "xsresultvalue.h"
#include "xsstring.h"

/*! \brief Exception class for Xsens public libraries. Inherits from std::exception
*/
class XsException : public std::exception {
public:
	//! \brief Copy constructor
	XsException(XsException const& e)
		: m_code(e.m_code)
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
		: m_code(err)
		, m_description(description)
	{
		if (prefix && (m_code != XRV_OK))
		{
			char codeString[16];
			sprintf(codeString, "%d: ", (int) m_code);
			XsString rv(codeString);
			rv << XsResultValue_toString(m_code);
			if (!m_description.empty())
				rv << "\tInfo: ";
			rv.append(m_description);
			m_description = rv;
		}
	}

	/*! \brief Initializing constructor
		\param description A description of the error.
	*/
	explicit XsException(XsString const& description)
		: m_code(XRV_ERROR)
		, m_description(description)
	{
	}

	//! \brief Destructor
	virtual ~XsException() throw()
	{
	}

	//! \brief Assignment operator, copies \a e to this
	XsException& operator = (XsException const& e)
	{
		m_code = e.m_code;
		m_description = e.m_description;
		return *this;
	}

	//! \brief Returns the error value supplied during construction
	inline XsResultValue code() const throw()
	{
		return m_code;
	}

	//! \brief Returns a description of the error that occurred as a char const*
	inline char const* what() const throw()
	{
		return m_description.c_str();
	}

	//! \brief Returns a description of the error that occurred as a XsString
	inline XsString const& text() const throw()
	{
		return m_description;
	}

	/*! \brief Throw/raise the exception object
		\note Override this in inheriting classes to make sure the proper type of exception is raised
	*/
	virtual void raise() const
	{
		throw *this;
	}

private:
	XsResultValue m_code;	//!< The supplied error code
	XsString m_description;	//!< The supplied description, prefixed with a description of the error code
};

#endif // file guard
#endif // __cplusplus guard
