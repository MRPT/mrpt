/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#include "graphslam-precomp.h"  // Precompiled headers
#include <mrpt/graphslam/CRosTopicMP.h>

namespace mrpt { namespace graphslam { namespace measurement_providers {

CRosTopicMP::CRosTopicMP():
	client_params(*this) {
	this->init();
}

CRosTopicMP::~CRosTopicMP() { 
	MRPT_LOG_DEBUG_STREAM << "In class Destructor";

	MRPT_LOG_DEBUG_STREAM << "Deleting client socket instance.";
	delete client;

}

void CRosTopicMP::init() {
	// configure the current provider
	m_class_name = "CRosTopicMP";
	m_ini_section_name = "CMeraurementProviderParameters";
	this->setLoggerName(m_class_name);
	rawlog_format = ACTION_OBSERVATIONS;
	run_online = true;
	provider_ready = false;

	client = NULL;
}

bool CRosTopicMP::getActionObservationPairOrObservation(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry ) {
	using namespace std;
	using namespace mrpt::obs;
	using namespace mrpt::utils;

	ASSERTMSG_(provider_ready, "getActionObservationPairOrObservation was called even though provider is not ready yet.");

	//is there any data available - if so return it, otherwise block and wait
	// TODO - maybe inform of the user if it takes too long to return from this
	// --> requires multithreading

	// Ask the server of data
	CMessage msg;
	bool did_receive = client->receiveMessage(msg,
			/*timeoutStart_ms =*/ 2000,
			/*timeoutBetween_ms =*/ 2000 );
	ASSERTMSG_(did_receive, "\nCould not successfully receive message from server! Exiting\n");

	stringstream ss("");
	ss <<  "Received message: Type: " << msg.type
		<< " | Length: " << msg.content.size();
	MRPT_LOG_DEBUG_STREAM << ss.str();

	// discard anything that might be in the rawlog
	action.clear_unique();
	observations.clear_unique();
	observation.clear_unique();

	bool success;
	if (msg.type == client_params.msg_types["FORMAT 1"]) {
		// in case of format #1 I need two objects:
		// - CActionCollection
		// - CSensoryFrame

		// TODO
		THROW_EXCEPTION("FORMAT 1 (action-observations) is not implemented yet.");

		//msg.deserializeIntoExistingObject(action.pointer());
	}
	else if (msg.type == client_params.msg_types["FORMAT 2"]) {
		msg.deserializeIntoExistingObject(observation.pointer());
		success=true;
	}
	else if (msg.type == client_params.msg_types["EXIT"]) {
		success = false;
	}
	else {
		THROW_EXCEPTION(mrpt::format("%s: Received measurement was not understood.", m_class_name.c_str()));
	}

	return success;
}

void CRosTopicMP::printParams() const {
	MRPT_START;

	// TODO - implement this.

	client_params.dumpToConsole();

	MRPT_END;
}
void CRosTopicMP::loadParams(const std::string& source_fname) {
	MRPT_START;
	using namespace mrpt::utils;

	client_params.loadFromConfigFileName(source_fname, m_ini_section_name);

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	// Minimum verbosity level of the logger
	int min_verbosity_level = source.read_int(
			m_ini_section_name,
			"class_verbosity",
			1, false);
	this->setMinLoggingLevel(VerbosityLevel(min_verbosity_level));
	this->logStr(LVL_DEBUG, "Successfully loaded parameters.");

	// initialize the client - connect to remote part...
	this->initClient(client);

	provider_ready = true;
	MRPT_END;
}

void CRosTopicMP::initClient(mrpt::utils::CClientTCPSocket* cl) {
	using namespace std;

	stringstream msg_ss;

	msg_ss << "Connecting to server side...\n";
	msg_ss << client_params.getAsString();
	MRPT_LOG_INFO_STREAM << msg_ss;

	client = new mrpt::utils::CClientTCPSocket();
	client->connect(
			client_params.server_addr,
			client_params.port_no,
			client_params.client_timeout_ms);

	MRPT_LOG_INFO_STREAM << "TCP Socket was successfully established." << endl;
}

// TClientParams
////////////////////////////////////////////////////////////////////////////////

CRosTopicMP::TClientParams::TClientParams(provider_t& p):
	provider(p)
{
	MRPT_START;

	msg_types["FORMAT 1"] = 1; /**< Transmit data in the 1st rawlog MRPT format */
	msg_types["FORMAT 2"] = 2; /**< Transmit data in the 2nd rawlog MRPT format */
	msg_types["EXIT"] = 99; /**< Code is returned when no more data is to be transmitted */


	MRPT_END;
}

CRosTopicMP::TClientParams::~TClientParams() { }

void CRosTopicMP::TClientParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("%s", this->getAsString().c_str());

	MRPT_END;
}
void CRosTopicMP::TClientParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
		const std::string &section) {
	MRPT_START;
	using namespace mrpt::utils;

	port_no = source.read_int(section           , "tcp_port_no"           , 68000       , false);
	server_addr = source.read_string(section    , "tcp_server_addr"       , "127.0.0.1" , false);
	client_timeout_ms = source.read_int(section , "tcp_client_timeout_ms" , 10000       , false);

	MRPT_END;
}

void CRosTopicMP::TClientParams::getAsString(std::string* params_out) const {
	using namespace std;

	stringstream ss("");
	ss << "------------------[ CRosTopicMP ]------------------\n";
	ss << "Port No.                           : " << port_no << endl;
	ss << "Server IP Address                  : " << server_addr.c_str() << endl;
	ss << "Client Time to wait for connection : " << client_timeout_ms << endl;

	*params_out = ss.str();
}

std::string CRosTopicMP::TClientParams::getAsString() const {
	std::string str;
	this->getAsString(&str);
	return str;
}

} } } // end of namespaces



