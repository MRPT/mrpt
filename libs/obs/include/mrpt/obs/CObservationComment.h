/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** This "observation" is actually a placeholder for a text block with comments
 * or additional parameters attached to a given rawlog file.
 *   There should be only one of this observations in a rawlog file, and it's
 * recommended to insert/modify them from the application RawlogViewer.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationComment : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationComment, mrpt::obs)

   public:
	/** Constructor.
	 */
	CObservationComment() : text() {}
	/** Destructor
	 */
	~CObservationComment() override = default;
	/** The text block. */
	std::string text;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D&) const override {}
	void setSensorPose(const mrpt::poses::CPose3D&) override {}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
