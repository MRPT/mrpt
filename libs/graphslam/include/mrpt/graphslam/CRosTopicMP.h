/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CROSTOPICMP_H
#define CROSTOPICMP_H

#include "link_pragmas.h"
#include "CMeasurementProvider.h"


#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CMessage.h>
#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>

#include <string>
#include <sstream>
#include <map>

namespace mrpt { namespace graphslam { namespace measurement_providers {

class GRAPHSLAM_IMPEXP CRosTopicMP : public CMeasurementProvider
{
public:

	typedef CMeasurementProvider super;
	typedef CRosTopicMP provider_t;

	CRosTopicMP();
	~CRosTopicMP();
	void init();

	bool getActionObservationPairOrObservation(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);

	void loadParams(const std::string& source_fname);
	void printParams() const;

	/** Getter method in sake of polymorphic behavior. */
	bool providerIsReady();
	/** Getter method in sake of polymorphic behavior. */
	bool providerRunsOnline();
	
private:
	std::string m_class_name;
	std::string m_ini_section_name;
	bool run_online;
	bool provider_ready;

	mrpt::utils::CClientTCPSocket* client;

	/** Given a client object, initialize a connection to the remote part
	 * based on the parameters of the TClientParams.
	 */
	void initClient(mrpt::utils::CClientTCPSocket* cl);

	/**\brief Parameters structure for managing the relevant to the decider
	 * variables in a compact manner
	 */
	struct TClientParams: public mrpt::utils::CLoadableOptions {
		public:
			TClientParams(provider_t& p);
			~TClientParams();

			void loadFromConfigFile(
					const mrpt::utils::CConfigFileBase &source,
					const std::string &section);
			void dumpToTextStream(mrpt::utils::CStream &out) const;
			/**\brief Return a string with the configuration parameters
			 */
			void getAsString(std::string* params_out) const;
			std::string getAsString() const;
			

			/**\brief Reference to the outer provider class.
			 *
			 * Handy for using its logging functions
			 */
			provider_t& provider;

			// TCP communication parameters
			int server_port_no;
			std::string server_addr;
			int client_timeout_ms;


			/**\brief Types indicating the content of the MRPT class instance that
			 * is contained in the message
			 */
			std::map<std::string, unsigned int> msg_types;
			

	} client_params;


};


} } } // end of namespaces

#endif /* end of include guard: CROSTOPICMP_H */
