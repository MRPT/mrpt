/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/mrpt_macros.h>

#include <stdexcept>
#include <iostream>
#include <string>
#include <sstream>

// TODO - include docstring, group etc.
class HypothesisNotFoundException: public std::runtime_error {
 	public:
		HypothesisNotFoundException(
				mrpt::utils::TNodeID from,
  			mrpt::utils::TNodeID to);
  	HypothesisNotFoundException(size_t id);
		~HypothesisNotFoundException();
		void clear();
		std::string getErrorMsg() const throw();
		const char* what() const throw();

 	private:
	mrpt::utils::TNodeID m_from;
	mrpt::utils::TNodeID m_to;

	/**\brief Hypothesis ID */
	size_t m_id;

	/**\brief Error message */
	std::string m_msg;
	static std::ostringstream m_cnvt;
};
