/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CHYPOTHESISNOTFOUNDEXCEPTION_H
#define CHYPOTHESISNOTFOUNDEXCEPTION_H

#include <cstdint>
#include <mrpt/core/exceptions.h>

#include <stdexcept>
#include <iostream>
#include <string>
#include <sstream>

// TODO - include docstring, group etc.
class HypothesisNotFoundException : public std::runtime_error
{
   public:
	HypothesisNotFoundException(
		mrpt::graphs::TNodeID from, mrpt::graphs::TNodeID to);
	HypothesisNotFoundException(size_t id);
	~HypothesisNotFoundException() throw();
	void clear();
	std::string getErrorMsg() const throw();
	const char* what() const throw();

   private:
	mrpt::graphs::TNodeID m_from;
	mrpt::graphs::TNodeID m_to;

	/**\brief Hypothesis ID */
	size_t m_id;

	/**\brief Error message */
	std::string m_msg;
	static std::ostringstream m_cnvt;
};

#endif /* end of include guard: CHYPOTHESISNOTFOUNDEXCEPTION_H */
