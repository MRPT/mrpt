/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CRAWLOGMP_H
#define CRAWLOGMP_H

#include "link_pragmas.h"
#include "CMeasurementProvider.h"

#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/CFileGZInputStream.h>

namespace mrpt { namespace graphslam { namespace measurement_providers {

class GRAPHSLAM_IMPEXP CRawlogMP : public CMeasurementProvider
{
public:

	typedef CMeasurementProvider super;

	CRawlogMP();
	~CRawlogMP();
	void init();

	bool getActionObservationPairOrObservation(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);
	void setRawlogFname(std::string rawlog_fname);

private:

	std::string m_class_name;
	std::string m_rawlog_fname;
	mrpt::utils::CFileGZInputStream m_rawlog_file;
};
 

} } } // end of namespaces


#endif /* end of include guard: CRAWLOGMP_H */
