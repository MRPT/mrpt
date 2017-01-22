/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#include <mrpt/graphslam/misc/CHypothesisNotFoundException.h>

std::ostringstream HypothesisNotFoundException::m_cnvt;

HypothesisNotFoundException::HypothesisNotFoundException(
  	mrpt::utils::TNodeID from,
  	mrpt::utils::TNodeID to):
  runtime_error("Hypothesis between set of nodes was not found") {

		this->clear();
		m_to = to;
		m_from = from;

		// TODO - Do not allocate it on the stack
		// http://stackoverflow.com/a/23742555/2843583
		std::stringstream ss;
    ss << std::runtime_error::what() << ":\t" <<
    	"From = " << m_from << " | " <<
    	"To = " << m_to << std::endl;
		m_msg = ss.str();
		std::cout << "m_msg: " << m_msg << std::endl;
  }
HypothesisNotFoundException::HypothesisNotFoundException(size_t id):
  runtime_error("Hypothesis with the given ID was not found") {

  	this->clear();
  	m_id = id;

		std::stringstream ss;
    ss << std::runtime_error::what() << ":\t" <<
    	"ID = " << m_id << std::endl;

		// TODO - When a HypohtesisNotFoundException is thrown, and what()
		// method is called from logFmt in grahpslam-engine catch statement, an
		// memory error is reported
		// `double free or corruption (!prev).`
		// what() output is not printed if I use the logger.logFmt method  but
		// IS printed if I used a printf call.
		MRPT_TODO("Double free or corruption error if HypothesisNotFoundException is raised.");
		std::cout << ss.str() << std::endl;
		m_msg = ss.str();
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

