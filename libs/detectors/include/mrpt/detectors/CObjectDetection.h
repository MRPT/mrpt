/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/img/CImage.h>

namespace mrpt::detectors
{
using vector_detectable_object = std::vector<CDetectableObject::Ptr>;

/** \ingroup mrpt_detectors_grp */
class CObjectDetection
{
   public:
	/** Initialize the object with parameters loaded from the given config file.
	 */
	inline void init(const std::string& configFile)
	{
		mrpt::config::CConfigFile cfg(configFile);
		init(cfg);
	}

	/** Initialize the object with parameters loaded from the given config
	 * source. */
	virtual void init(const mrpt::config::CConfigFileBase& cfg) = 0;

	inline void detectObjects(
		const mrpt::obs::CObservation::Ptr obs,
		vector_detectable_object& detected)
	{
		detectObjects_Impl(obs.get(), detected);
	};

	inline void detectObjects(
		const mrpt::obs::CObservation* obs, vector_detectable_object& detected)
	{
		detectObjects_Impl(obs, detected);
	};

	void detectObjects(
		const mrpt::img::CImage* img, vector_detectable_object& detected);

   protected:
	virtual void detectObjects_Impl(
		const mrpt::obs::CObservation* obs,
		vector_detectable_object& detected) = 0;

};  // End of class
}  // namespace mrpt::detectors
