/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>

namespace mrpt::obs::utils
{
/**\brief Given an mrpt::obs::CSensoryFrame and a mrpt::obs::CObservation
 * pointer if a OBSERVATION_T type observation is included and return a
 * pointer to that instance.
 *
 * \note Pointer to a single instance is going to be returned. If a suitable
 * observation exists in both the CSensoryFrame and CObservation the outcome
 * is decided by the \a priority_to_sf flag
 *
 * \return Pointer to the observation of the given type. Otherwise, an empty
 * Ptr object is returned if a valid observation is not found.
 *
 * \ingroup mrpt_obs_grp
 */
template <class OBSERVATION_T>
typename OBSERVATION_T::Ptr getObservation(
	mrpt::obs::CSensoryFrame::Ptr& observations,
	mrpt::obs::CObservation::Ptr& observation, bool priority_to_sf = true)
{
	using obs_t = typename OBSERVATION_T::Ptr;

	obs_t cobs_ptr;
	obs_t sf_ptr;
	obs_t obs_out;

	// CObservation
	const mrpt::rtti::TRuntimeClassId* class_ID =
		&OBSERVATION_T::GetRuntimeClassIdStatic();
	if (observation && observation->GetRuntimeClass()->derivedFrom(class_ID))
	{
		cobs_ptr = std::dynamic_pointer_cast<OBSERVATION_T>(observation);
	}

	// CSensoryFrame
	if (observations)
	{
		cobs_ptr = observations->getObservationByClass<OBSERVATION_T>();
	}

	// decide on which one to return
	if (cobs_ptr && sf_ptr)
	{
		obs_out = priority_to_sf ? sf_ptr : cobs_ptr;
	}
	else if (cobs_ptr)
	{
		obs_out = cobs_ptr;
	}
	else if (sf_ptr)
	{
		obs_out = sf_ptr;
	}
	else
	{
		obs_out = typename OBSERVATION_T::Ptr();
	}

	return obs_out;
}
}  // namespace mrpt::obs::utils
