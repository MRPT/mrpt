/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/detectors/CObjectDetection.h>

namespace mrpt::detectors
{
/**
 * \ingroup mrpt_detectors_grp
 */
class CCascadeClassifierDetection : virtual public CObjectDetection
{
   public:
	CCascadeClassifierDetection();

	virtual ~CCascadeClassifierDetection();

	/** Initialize cascade classifier detection */
	void init(const mrpt::config::CConfigFileBase& cfg) override;

   protected:
	/** Detect objects in a *CObservation
	 * \return A vector with detected objects
	 */

	void detectObjects_Impl(
		const mrpt::obs::CObservation* obs,
		vector_detectable_object& detected) override;

	/** Cascade classifier object */
	void* m_cascade;

	struct TOptions
	{
		std::string cascadeFileName;
		double scaleFactor;
		int minNeighbors;
		int flags;
		int minSize;
		/** Cascade classifier options */
	} m_options;

};  // End of class
}  // namespace mrpt::detectors
