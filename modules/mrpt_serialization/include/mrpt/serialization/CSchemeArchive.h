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

#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::serialization
{
/** Returns an archive for reading/writing in JSON format.
 *
 * The document is held in a mrpt::containers::yaml tree, so no external JSON
 * library is needed. Writing to a stream emits JSON; reading from a stream
 * accepts JSON (or YAML) text, which requires mrpt_containers to be built with
 * libfyaml (the default).
 *
 * Reading a key or index missing from the document gives 0, `false` or an
 * empty string, depending on the requested type.
 *
 * See \ref mrpt_serialization_grp for examples of use.
 * \ingroup mrpt_serialization_grp */
CSchemeArchiveBase archiveJSON();

}  // namespace mrpt::serialization
