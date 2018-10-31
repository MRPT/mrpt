/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <mrpt/core/exceptions.h>
#include <mrpt/graphs/TNodeID.h>
#include <stdexcept>
#include <string>

namespace mrpt::graphs
{
// TODO - include docstring, group etc.
class HypothesisNotFoundException : public std::runtime_error
{
   public:
	HypothesisNotFoundException(
		mrpt::graphs::TNodeID from, mrpt::graphs::TNodeID to);
	HypothesisNotFoundException(size_t id);
	~HypothesisNotFoundException() noexcept override;
	void clear();
	std::string getErrorMsg() const noexcept;
	const char* what() const noexcept override;

   private:
	mrpt::graphs::TNodeID m_from{INVALID_NODEID}, m_to{INVALID_NODEID};

	/**\brief Hypothesis ID */
	size_t m_id{0};

	/**\brief Error message */
	std::string m_msg;
};
}  // namespace mrpt::graphs
