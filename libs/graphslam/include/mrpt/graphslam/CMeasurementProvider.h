/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CMEASUREMENTPROVIDER_H
#define CMEASUREMENTPROVIDER_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/utils/COutputLogger.h>

#include <string>

namespace mrpt { namespace graphslam { namespace measurement_providers {

/**
 * \brief Formats that can be used by the specific implementations of the CMeasurementProvider
 * interface
 */
enum RawlogFormat { 
	ACTION_OBSERVATIONS=1,
	OBSERVATIONS_ONLY,
	UNDEFINED
};

/**\brief Inteface for implementing measurement provider classes that are to
 * be used when running graph-slam. For an example of inheriting from
 * this class see CRawlogMP.
 *
 * \note As a naming convention, all the implemented measurement providers
 * are suffixed with the MP acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
class CMeasurementProvider : public mrpt::utils::COutputLogger {
public:
	// 
	// public function definitions
	//

	CMeasurementProvider() {

		// just some default values - to be changed in the interface
		// implementations

		this->init();
	}
	virtual ~CMeasurementProvider() { }

	virtual void init() {
		m_class_name = "CMeasurementProvider";
		this->setLoggerName(m_class_name);

		run_online = false;
		provider_ready = false;
		rawlog_format = UNDEFINED;


	};

	/**\brief Main method that can be used to query the provider for the next
	 * measurement(s).
	 *
	 * \return False if there is no other measurement to be read, otherwise True
	 */
	virtual bool getActionObservationPairOrObservation(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry )=0;

	/**\brief Load the parameters related to the current class from an external
	 * .ini file.
	 */
	virtual void loadParams(const std::string& source_fname) {}
	/**\brief Print the class parameters to the console - handy for debugging
	 */
	virtual void printParams() const { }

	//
	// public variables
	//

	/**\brief Indicates whether the provider is ready to be querried for
	 * measurements
	 */
	bool provider_ready;
	/**\brief Variable indicating whether the current provider class is used in
	 * an online fashion.
	 */
	bool run_online;

	/**\brief Format that is used by the current provider class.
	 */
	RawlogFormat rawlog_format;

private:

	std::string m_class_name;

};

} } } // end of namespaces

#endif /* end of include guard: CMEASUREMENTPROVIDER_H */
