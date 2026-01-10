/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
