/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/datetime.h>

namespace mrpt::obs
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
   public:
	/** Default ctor */
	CAction() = default;
	~CAction() override = default;

	/** The associated time-stamp.*/
	mrpt::system::TTimeStamp timestamp{INVALID_TIMESTAMP};

	/** Build a detailed, multi-line textual description of the action
	 * contents and dump it to the output stream.
	 * \note If overried by derived classes, call base
	 * CAction::getDescriptionAsText() first to show common information.
	 */
	virtual void getDescriptionAsText(std::ostream& o) const;

	/** Return by value version of getDescriptionAsText(std::ostream&) */
	std::string getDescriptionAsTextValue() const;

};  // End of class def.

}  // namespace mrpt::obs
