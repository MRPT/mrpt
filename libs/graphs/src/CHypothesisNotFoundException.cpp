/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphs-precomp.h"  // Precompiled headers

#include <mrpt/graphs/CHypothesisNotFoundException.h>

std::ostringstream HypothesisNotFoundException::m_cnvt;

HypothesisNotFoundException::HypothesisNotFoundException(
  	mrpt::utils::TNodeID from,
  	mrpt::utils::TNodeID to):
  runtime_error("Hypothesis between set of nodes was not found")
{
  using namespace mrpt;

	this->clear();
	m_to = to;
	m_from = from;

	// TODO - Do not allocate it on the stack
	// http://stackoverflow.com/a/23742555/2843583
  m_msg = std::runtime_error::what();
  m_msg += "- ";
  m_msg += format("[from] %lu ==> ", static_cast<unsigned long>(m_from)).c_str();
  m_msg += format("[to] %lu", static_cast<unsigned long>(m_to)).c_str();
}

HypothesisNotFoundException::HypothesisNotFoundException(size_t id):
  runtime_error("Hypothesis with the given ID was not found")
{
  using namespace mrpt;

  this->clear();
  m_id = id;

	m_msg = std::runtime_error::what();
	m_msg += format("- ID:%lu", static_cast<unsigned long>(id)).c_str();
}

void HypothesisNotFoundException::clear() {
  m_to = INVALID_NODEID;
  m_from = INVALID_NODEID;
  m_id = SIZE_MAX;
  m_msg.clear();
}

HypothesisNotFoundException::~HypothesisNotFoundException() throw() {}

std::string HypothesisNotFoundException::getErrorMsg() const throw() {
  return m_msg;
}

const char* HypothesisNotFoundException::what() const throw() {
  m_cnvt.str("");
  m_cnvt << getErrorMsg();
  return m_cnvt.str().c_str();
}

