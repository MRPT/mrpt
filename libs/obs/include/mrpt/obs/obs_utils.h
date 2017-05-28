/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef OBS_UTILS_H
#define OBS_UTILS_H

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>

namespace mrpt { namespace obs { namespace utils {

	/**\brief Given an mrpt::obs::CSensoryFrame and a mrpt::obs::CObservation
	 * pointer if a OBSERVATION_T type observation is included and return a
	 * pointer to that instance.
	 *
	 * \note Pointer to a single instance is going to be returned. If a suitable
	 * observation exists in both the CSensoryFrame and CObservation the outcome
	 * is decided by the \a priority_to_sf flag 
	 *
	 * \return Pointer to the observation of the given type. Otherwise, an empty
	 * SmartPtr object is returned if a valid observation is not found.
	 *
	 * \ingroup mrpt_obs_grp 
	 */
	template<class OBSERVATION_T>
	typename OBSERVATION_T::SmartPtr getObservation(
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		bool priority_to_sf=true) {

		typedef typename OBSERVATION_T::SmartPtr obs_t;

		obs_t cobs_ptr;
		obs_t sf_ptr;
		obs_t obs_out;

		// CObservation
		const mrpt::utils::TRuntimeClassId*	class_ID = OBSERVATION_T::classinfo;
		if (observation.present() &&
				observation->GetRuntimeClass()->derivedFrom(class_ID)) {
			cobs_ptr = 
				static_cast<obs_t>(observation);
		}

		// CSensoryFrame
		if (observations.present()) {
			cobs_ptr = observations->getObservationByClass<OBSERVATION_T>();
		}

		// decide on which one to return
		if (cobs_ptr.present() && sf_ptr.present()) {
			obs_out = priority_to_sf? sf_ptr : cobs_ptr;
		}
		else if (cobs_ptr.present()) {
			obs_out = cobs_ptr;
		}
		else if (sf_ptr.present()) {
			obs_out = sf_ptr;
		}
		else {
			obs_out = typename OBSERVATION_T::SmartPtr();
		}

		return obs_out;
	}

} } } // end of namespaces

#endif /* end of include guard: OBS_UTILS_H */

