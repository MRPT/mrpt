#include "TGraphSlamHandler.h"

TGraphSlamHandler::TGraphSlamHandler() {
	using namespace mrpt::system;

	this->logger = NULL;
}

void TGraphSlamHandler::setOutputLoggerPtr(mrpt::utils::COutputLogger* logger) {
	MRPT_START;

	ASSERT_(logger);
	this->logger = logger;

	MRPT_END;
}

TGraphSlamHandler::~TGraphSlamHandler() {}

void TGraphSlamHandler::readConfigFname(const std::string& fname) {
	MRPT_START;
	using namespace mrpt::utils;

	ASSERTMSG_(mrpt::system::fileExists(fname),
			mrpt::format("\nConfiguration file not found: \n%s\n", fname.c_str()));

	if (logger) {
		logger->logStr(LVL_INFO, "Reading the .ini file... ");
	}

	CConfigFile cfg_file(fname);

	save_graph = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_graph",
			true, false);
	save_3DScene = cfg_file.read_bool(
			"GeneralConfiguration",
			"save_3DScene",
			true, false);
	save_graph_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_graph_fname",
			"output_graph.graph", false);
	save_3DScene_fname = cfg_file.read_string(
			"GeneralConfiguration",
			"save_3DScene_fname",
			"scene.3DScene", false);

	MRPT_END;
}

void TGraphSlamHandler::printParams() const {
	std::cout << this->getParamsAsString() << std::endl;

}

void TGraphSlamHandler::getParamsAsString(std::string* str) const {
	using namespace std;

	ASSERT_(str);

	stringstream ss_out("");

	// general configuration parameters
	ss_out << "Generate .graph file?           = "
		<< ( save_graph? "TRUE" : "FALSE" )  << std::endl;
	ss_out << "Generate .3DScene file?         = "
		<< ( save_3DScene? "TRUE" : "FALSE" ) << std::endl;
	if (save_graph) {
		ss_out << "Generated .graph filename       = "
			<< save_graph_fname << std::endl;
	}
	if (save_3DScene) {
		ss_out << "Generated .3DScene filename     = "
			<< save_3DScene_fname << std::endl;
	}

	*str = ss_out.str();
}

std::string TGraphSlamHandler::getParamsAsString() const {
	std::string str;
	this->getParamsAsString(&str);
	return str;
}
