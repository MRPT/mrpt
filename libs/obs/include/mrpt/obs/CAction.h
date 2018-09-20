/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/datetime.h>

namespace mrpt
{
/** \ingroup mrpt_obs_grp */
namespace obs
{
/** Declares a class for storing a robot action. It is used in
 * mrpt::obs::CRawlog,
 *    for logs storage and particle filter based simulations.
 *  See derived classes for implementations.
 *
 * \sa CActionCollection, CRawlog
 * \ingroup mrpt_obs_grp
 */
class CAction : public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CAction)

	/** Default ctor */
	CAction() = default;
	~CAction() override = default;

	/** The associated time-stamp.*/
	mrpt::system::TTimeStamp timestamp{INVALID_TIMESTAMP};

};  // End of class def.

}  // namespace obs
}  // namespace mrpt
