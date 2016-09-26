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
	delete m_client;

}

void CRosTopicMP::init() {
	MRPT_START;

	// configure the current provider
	m_class_name = "CRosTopicMP";
	m_ini_section_name = "CMeasurementProviderParameters";
	this->setLoggerName(m_class_name);
	run_online = true;
	provider_ready = false;

	m_client = NULL;

	MRPT_END;
}

bool CRosTopicMP::getActionObservationPairOrObservation(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry ) {
	MRPT_START;

	using namespace std;
	using namespace mrpt::obs;
	using namespace mrpt::utils;

	ASSERTMSG_(provider_ready,
			"getActionObservationPairOrObservation was called even though provider is not ready yet.");

	//is there any data available - if so return it, otherwise block and wait
	// TODO - maybe inform of the user if it takes too long to return from this
	// --> requires multithreading

	// Ask the server of data
	CMessage msg;
	bool did_receive = m_client->receiveMessage(msg,
			/*timeoutStart_ms =*/ 40000,
			/*timeoutBetween_ms =*/ 2000 );
	if (!did_receive) { // communication failed. What exception should I raise?
		if (client_params.has_transmitted_valid_data) {
 			// Overall, I have managed to transmit some data.
			// Assume that the data transmission is over.
			msg.type = client_params.msg_types["EXIT"];
		}
		else {
			THROW_EXCEPTION("\nMessage transmission stopped abruptly.\n");
		}
	}

	stringstream ss("");
	ss <<  "Received message: Type: " << msg.type
		<< " | Length: " << msg.content.size();
	MRPT_LOG_DEBUG_STREAM << ss.str();

	// discard anything that might be in the rawlog
	action.clear_unique();
	observations.clear_unique();
	observation.clear_unique();

	bool success;
	if (msg.type == client_params.msg_types["FORMAT_1"]) {
		// in case of format #1 I need two objects:
		// - CActionCollection
		// - CSensoryFrame
		// TODO
		THROW_EXCEPTION("FORMAT 1 (action-observations) is not implemented yet.");
	}
	else if (msg.type == client_params.msg_types["FORMAT_2"]) {
		observation = mrpt::obs::CObservation2DRangeScan::Create();
		msg.deserializeIntoExistingObject(observation.pointer());
		success=true;
	}
	else if (msg.type == client_params.msg_types["EXIT"]) {
		success = false;
	}
	else {
		THROW_EXCEPTION(
				mrpt::format("%s: Received measurement was not understood.",
					m_class_name.c_str()));
	}

	if (success) {
		client_params.has_transmitted_valid_data = true;
	}

	return success;
	MRPT_END;
}

bool CRosTopicMP::providerIsReady() {
	return provider_ready;
}
bool CRosTopicMP::providerRunsOnline() {
	return run_online;
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
	this->initClient(m_client);

	provider_ready = true;
	MRPT_END;
}

void CRosTopicMP::initClient(mrpt::utils::CClientTCPSocket* cl) {
	MRPT_START;
	using namespace std;

	stringstream msg_ss;

	msg_ss << "Connecting to server side...\n";
	msg_ss << client_params.getAsString();
	MRPT_LOG_INFO_STREAM << msg_ss;

	// Connect to the server side. Server counterpart may not be up when the
	// connection attempt is made Make multiple attempts.
	bool did_connect = false;
	int tries_thresh = 10;
	int curr_try = 1;
	m_client = new mrpt::utils::CClientTCPSocket();
	std::string error_msg = "Connection with remote server could not be established.";
	while (tries_thresh >= curr_try) {
		try {
			m_client->connect(
					client_params.server_addr,
					client_params.server_port_no,
					client_params.client_timeout_ms);
			did_connect = true;
			break;
		}
		catch(std::logic_error& e) {
			MRPT_LOG_WARN_STREAM << error_msg << "Retrying... "
				<< curr_try++ << "/" << tries_thresh << endl;
			mrpt::system::sleep(1000);
		}
	}
	error_msg = error_msg +
		"\nMake sure that the TCP server on the ROS side is up, otherwise contact the maintainer.";
	ASSERTMSG_(did_connect, mrpt::format("\n%s\n", error_msg.c_str()));

	MRPT_LOG_INFO_STREAM << "Connection with server was successfully established." << endl;

	MRPT_END;
}

// TClientParams
////////////////////////////////////////////////////////////////////////////////

CRosTopicMP::TClientParams::TClientParams(provider_t& p):
	provider(p) {
	MRPT_START;

	has_transmitted_valid_data = false;

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

	server_addr = source.read_string(section    , "tcp_server_addr"       , "127.0.0.1" , false);
	server_port_no = source.read_int(section    , "tcp_server_port_no"    , 6800        , false);
	client_timeout_ms = source.read_int(section , "tcp_client_timeout_ms" , 10000       , false);

	// reading the possilble formats that an incoming message may have
	msg_types["FORMAT_1"] = source.read_int(section , "MSG_TYPE_FORMAT_1" , 1  , false);
	msg_types["FORMAT_2"] = source.read_int(section , "MSG_TYPE_FORMAT_2" , 2  , false);
	msg_types["EXIT"] = source.read_int(section     , "MSG_TYPE_EXIT"     , 99 , false);

	MRPT_END;
}

void CRosTopicMP::TClientParams::getAsString(std::string* params_out) const {
	MRPT_START;

	using namespace std;

	stringstream ss("");
	ss << "------------------[ CRosTopicMP ]------------------\n";
	ss << "Port No.                           : " << server_port_no << endl;
	ss << "Server IP Address                  : " << server_addr.c_str() << endl;
	ss << "Client Time to wait for connection : " << client_timeout_ms << endl;
	ss << "Available message codes: " << endl;
	ss << "\tMessage code for FORMAT #1: "               << msg_types.find("FORMAT_1")->second << endl;
	ss << "\tMessage code for FORMAT #2: "               << msg_types.find("FORMAT_2")->second << endl;
	ss << "\tMessage code for end of data transmission " << msg_types.find("EXIT")->second     << endl;

	*params_out = ss.str();

	MRPT_END;
}

std::string CRosTopicMP::TClientParams::getAsString() const {
	MRPT_START;

	std::string str;
	this->getAsString(&str);
	return str;

	MRPT_END;
}

} } } // end of namespaces



