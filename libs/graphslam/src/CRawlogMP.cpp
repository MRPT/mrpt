#include "graphslam-precomp.h"  // Precompiled headers
#include <mrpt/graphslam/CRawlogMP.h>

namespace mrpt { namespace graphslam { namespace measurement_providers {

CRawlogMP::CRawlogMP() {
	this->init();
}

CRawlogMP::~CRawlogMP() { 
	MRPT_LOG_DEBUG_STREAM << "In class Destructor";

	if (m_rawlog_file.fileOpenCorrectly()) {
		MRPT_LOG_DEBUG_STREAM << "Closing rawlog file: " << m_rawlog_fname;
		m_rawlog_file.close();
	}

}

void CRawlogMP::init() {
	// configure the current provider
	m_class_name = "CRawlogMP";
	this->setLoggerName(m_class_name);

	run_online = false;
	provider_ready = false;
}

bool CRawlogMP::getActionObservationPairOrObservation(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry ) {
	using namespace mrpt::obs;

	ASSERTMSG_(provider_ready,"getActionObservationPairOrObservation was called even though provider is not ready yet.");

	// just use the corresponding CRawlog method
	bool success = CRawlog::getActionObservationPairOrObservation(
			m_rawlog_file,
			action,
			observations,
			observation,
			rawlog_entry );

	return success;
}

void CRawlogMP::setRawlogFname(std::string rawlog_fname) {
	m_rawlog_fname = rawlog_fname;
	ASSERTMSG_(mrpt::system::fileExists(m_rawlog_fname),
			format("\nRawlog file: %s was not found\n", m_rawlog_fname.c_str() ));

	bool did_open = m_rawlog_file.open(rawlog_fname);
	ASSERTMSG_(did_open,
			format("Rawlog file %s could not be opened", m_rawlog_fname.c_str() ));

	// initial setup is now complete...
	MRPT_LOG_DEBUG_STREAM << "Successfully read the rawlog file: " << m_rawlog_fname;
	provider_ready = true;
}

bool CRawlogMP::providerIsReady() {
	return provider_ready;
}
bool CRawlogMP::providerRunsOnline() {
	return run_online;
}

} } } // end of namespaces

